#!/usr/bin/env python3
"""
Lightweight HTTP server wrapping recompute_hulls.py.

Endpoints:
  POST /recompute  — accepts A-class params JSON, returns hull URLs
  GET  /parametric/... — serves static hull files from data/parametric/

Listens on port 8765.
"""

import json
import os
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse

SCRIPT_DIR = Path(__file__).resolve().parent
VIEWER_ROOT = SCRIPT_DIR.parent
DATA_DIR = VIEWER_ROOT / "data"

# Import recompute logic
sys.path.insert(0, str(SCRIPT_DIR))
from recompute_hulls import compute_hash, generate_hull, mirror_hull
from compute_roi_manipulability import compute_roi_manipulability
import numpy as np
from datetime import datetime, timezone


class HullRequestHandler(SimpleHTTPRequestHandler):
    """Handle /recompute POST and serve static parametric files."""

    def __init__(self, *args, **kwargs):
        # Serve files from data/ directory
        super().__init__(*args, directory=str(DATA_DIR), **kwargs)

    def do_OPTIONS(self):
        """Handle CORS preflight."""
        self.send_response(200)
        self.end_headers()

    def do_POST(self):
        path = urlparse(self.path).path

        if path == "/recompute":
            self._handle_recompute()
        elif path == "/roi-manipulability":
            self._handle_roi_manipulability()
        else:
            self.send_error(404, "Not Found")

    def _set_cors_headers(self):
        """Set CORS headers for all responses."""
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _handle_recompute(self):
        """Handle POST /recompute request."""
        try:
            # Parse request body
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            params = json.loads(body)

            # Extract A-class parameters
            arm_upper_z = params.get("arm_upper_z_total", 0.334)
            arm_elbow_y = params.get("arm_elbow_y_offset", 0.025)
            arm_forearm_z = params.get("arm_forearm_z_total", 0.334)
            arm_wrist_z = params.get("arm_wrist_z", 0.050)
            sobol_m = params.get("sobol_m", 12)

            # Compute hash
            hash_str = compute_hash(arm_upper_z, arm_elbow_y, arm_forearm_z, arm_wrist_z)
            output_dir = DATA_DIR / "parametric" / "variants" / hash_str

            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Recompute request:")
            print(f"  Parameters: upper_z={arm_upper_z}, elbow_y={arm_elbow_y}, "
                  f"forearm_z={arm_forearm_z}, wrist_z={arm_wrist_z}")
            print(f"  Hash: {hash_str}")
            print(f"  Sobol_m: {sobol_m}")

            # Versioned file names
            left_fname = f"left_arm_hull_2p{sobol_m}.json"
            right_fname = f"right_arm_hull_2p{sobol_m}.json"

            # Check if cached result exists with same or higher sobol_m
            config_path = output_dir / "config.json"
            if config_path.exists():
                with open(config_path) as f:
                    cached_config = json.load(f)
                cached_m = cached_config.get("sobol_m", 0)
                if cached_m >= sobol_m:
                    # Use highest available version
                    best_left = f"left_arm_hull_2p{cached_m}.json"
                    best_right = f"right_arm_hull_2p{cached_m}.json"
                    if (output_dir / best_left).exists():
                        print(f"  Cache hit! (cached 2^{cached_m} >= requested 2^{sobol_m})")
                        response = {
                            "status": "ok",
                            "hash": hash_str,
                            "left_hull_url": f"/parametric/variants/{hash_str}/{best_left}",
                            "right_hull_url": f"/parametric/variants/{hash_str}/{best_right}",
                            "sobol_m": cached_m,
                            "cached": True,
                        }
                        self._send_json_response(response)
                        return

            # Recompute: only left arm, then mirror for right
            print(f"  Cache miss. Computing left arm + mirror...")
            left_data = generate_hull("left", arm_upper_z, arm_elbow_y, arm_forearm_z,
                                      arm_wrist_z, sobol_m, output_dir)
            right_data = mirror_hull(left_data)

            # Write right hull
            right_path = Path(output_dir) / right_fname
            right_path.parent.mkdir(parents=True, exist_ok=True)
            with open(right_path, "w") as f:
                json.dump(right_data, f, indent=2)
            print(f"  Wrote mirrored right hull: {right_path}")

            # Rename left hull to versioned name if needed
            old_left = Path(output_dir) / "left_arm_hull.json"
            new_left = Path(output_dir) / left_fname
            if old_left.exists() and not new_left.exists():
                old_left.rename(new_left)
            elif old_left.exists() and new_left.exists():
                old_left.unlink()

            # Write config.json
            config = {
                "hash": hash_str,
                "parameters": {
                    "arm_upper_z_total": arm_upper_z,
                    "arm_elbow_y_offset": arm_elbow_y,
                    "arm_forearm_z_total": arm_forearm_z,
                    "arm_wrist_z": arm_wrist_z,
                },
                "sobol_m": sobol_m,
                "generated_at": datetime.now(timezone.utc).isoformat(),
            }
            with open(config_path, "w") as f:
                json.dump(config, f, indent=2)

            # Return response
            response = {
                "status": "ok",
                "hash": hash_str,
                "left_hull_url": f"/parametric/variants/{hash_str}/{left_fname}",
                "right_hull_url": f"/parametric/variants/{hash_str}/{right_fname}",
                "sobol_m": sobol_m,
                "cached": False,
            }
            self._send_json_response(response)
            print(f"  Done!")

        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
            self.send_error(500, f"Internal Server Error: {e}")

    def _handle_roi_manipulability(self):
        """Handle POST /roi-manipulability request."""
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            params = json.loads(body)

            roi_id = params.get("roi_id", "roi_default")
            arm = params.get("arm", "left")
            roi = params.get("roi", {})
            resolution = float(params.get("resolution", 0.05))
            ee_orientation = np.array(params.get("ee_orientation",
                [[1, 0, 0], [0, 0, 1], [0, -1, 0]]), dtype=float)
            waist_angles = list(params.get("waist_angles", [0.0, 0.0, 0.0]))
            n_ik_attempts = int(params.get("n_ik_attempts", 20))

            urdf_path = str((VIEWER_ROOT.parent / "moz2_simplified.urdf").resolve())

            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] ROI manipulability request:")
            print(f"  roi_id: {roi_id}, arm: {arm}")
            print(f"  roi: {roi}")
            print(f"  resolution: {resolution}, n_ik_attempts: {n_ik_attempts}")
            print(f"  URDF: {urdf_path}")

            result = compute_roi_manipulability(
                urdf_path=urdf_path,
                roi_bounds=roi,
                resolution=resolution,
                ee_orientation=ee_orientation,
                waist_angles=waist_angles,
                arm=arm,
                n_ik_attempts=n_ik_attempts,
            )
            result['roi_id'] = roi_id

            output_dir = DATA_DIR / "roi"
            output_dir.mkdir(parents=True, exist_ok=True)
            output_path = output_dir / f"{roi_id}.json"
            with open(output_path, "w") as f:
                json.dump(result, f, indent=2)
            print(f"  Wrote: {output_path}")

            response = {
                "status": "ok",
                "roi_id": roi_id,
                "url": f"/roi/{roi_id}.json",
            }
            self._send_json_response(response)

        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
            self.send_error(500, f"Internal Server Error: {e}")

    def _handle_roi_manipulability(self):
        """Handle POST /roi-manipulability request."""
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            params = json.loads(body)

            roi_id = params.get("roi_id", "roi_default")
            arm = params.get("arm", "left")
            roi = params.get("roi", {})
            resolution = float(params.get("resolution", 0.05))
            ee_orientation = np.array(params.get("ee_orientation",
                [[1, 0, 0], [0, 0, 1], [0, -1, 0]]), dtype=float)
            waist_angles = list(params.get("waist_angles", [0.0, 0.0, 0.0]))
            n_ik_attempts = int(params.get("n_ik_attempts", 20))

            urdf_path = str((VIEWER_ROOT.parent / "moz2_simplified.urdf").resolve())

            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] ROI manipulability request:")
            print(f"  roi_id: {roi_id}, arm: {arm}")
            print(f"  roi: {roi}")
            print(f"  resolution: {resolution}, n_ik_attempts: {n_ik_attempts}")
            print(f"  URDF: {urdf_path}")

            result = compute_roi_manipulability(
                urdf_path=urdf_path,
                roi_bounds=roi,
                resolution=resolution,
                ee_orientation=ee_orientation,
                waist_angles=waist_angles,
                arm=arm,
                n_ik_attempts=n_ik_attempts,
            )
            result['roi_id'] = roi_id

            output_dir = DATA_DIR / "roi"
            output_dir.mkdir(parents=True, exist_ok=True)
            output_path = output_dir / f"{roi_id}.json"
            with open(output_path, "w") as f:
                json.dump(result, f, indent=2)
            print(f"  Wrote: {output_path}")

            response = {
                "status": "ok",
                "roi_id": roi_id,
                "url": f"/roi/{roi_id}.json",
            }
            self._send_json_response(response)

        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
            self.send_error(500, f"Internal Server Error: {e}")

    def _send_json_response(self, data):
        """Send JSON response with CORS headers."""
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def end_headers(self):
        """Override to add CORS headers to all responses."""
        self._set_cors_headers()
        super().end_headers()


def main():
    port = 8765
    server = HTTPServer(("localhost", port), HullRequestHandler)
    print(f"=== Hull Server ===")
    print(f"Listening on http://localhost:{port}")
    print(f"Serving files from: {DATA_DIR}")
    print(f"\nEndpoints:")
    print(f"  POST /recompute  — recompute parametric hulls")
    print(f"  POST /roi-manipulability  — compute ROI manipulability voxel grid")
    print(f"  GET  /parametric/... — serve hull files")
    print(f"  GET  /roi/... — serve ROI manipulability files")
    print(f"\nPress Ctrl+C to stop.\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        server.shutdown()


if __name__ == "__main__":
    main()
