# Parametric Hull Backend

Backend implementation for Phase 3.5 parametric geometry what-if analysis.

## Overview

This backend enables recomputation of arm convex hulls with modified A-class geometry parameters:
- `arm_upper_z_total` — total upper arm z length (default: 0.334m)
- `arm_elbow_y_offset` — elbow y offset (default: 0.025m)
- `arm_forearm_z_total` — total forearm z length (default: 0.334m)
- `arm_wrist_z` — wrist z length (default: 0.050m)

## Files

### 1. `recompute_hulls.py`

CLI script for parametric hull recomputation.

**Usage:**
```bash
python3 scripts/recompute_hulls.py \
  --arm-upper-z 0.334 \
  --arm-elbow-y 0.025 \
  --arm-forearm-z 0.334 \
  --arm-wrist-z 0.050 \
  --sobol-m 12
```

**Parameters:**
- `--arm-upper-z` — Total upper arm z length (default: 0.334)
- `--arm-elbow-y` — Elbow y offset (default: 0.025)
- `--arm-forearm-z` — Total forearm z length (default: 0.334)
- `--arm-wrist-z` — Wrist z length (default: 0.050)
- `--sobol-m` — Sobol exponent (samples = 2^m). 12=quick preview, 18=production
- `--output-dir` — Output directory (default: auto-generated from hash)

**Output:**
```
data/parametric/variants/<hash>/
  config.json
  left_arm_hull.json
  right_arm_hull.json
```

**Implementation:**
- Loads URDFKinematics from existing `single_arm_workspace.py`
- Modifies joint origins in-place based on A-class parameters
- Preserves URDF ratios when splitting aggregated dimensions
- Reuses Sobol sampling + FK + ConvexHull logic from `export_arm_hulls.py`
- Outputs hull JSONs matching `moz2_arm_hull_v1` schema

### 2. `hull_server.py`

HTTP server wrapping `recompute_hulls.py` for frontend integration.

**Usage:**
```bash
python3 scripts/hull_server.py
```

Server listens on `http://localhost:8765`.

**Endpoints:**

#### POST /recompute

Accepts A-class parameters, returns hull URLs.

**Request:**
```json
{
  "arm_upper_z_total": 0.334,
  "arm_elbow_y_offset": 0.025,
  "arm_forearm_z_total": 0.334,
  "arm_wrist_z": 0.050,
  "sobol_m": 12
}
```

**Response:**
```json
{
  "status": "ok",
  "hash": "e9bb9f136bdd",
  "left_hull_url": "/parametric/variants/e9bb9f136bdd/left_arm_hull.json",
  "right_hull_url": "/parametric/variants/e9bb9f136bdd/right_arm_hull.json",
  "sobol_m": 12,
  "cached": false
}
```

**Features:**
- Caching: checks if hull already exists for given parameters + sobol_m
- CORS enabled for frontend integration
- Serves static files from `data/` directory
- Progress logging to stdout

### 3. `viewer/vite.config.ts`

Vite dev server proxy configuration.

**Proxies:**
- `/api` → `http://localhost:8765` (for /recompute endpoint)
- `/parametric` → `http://localhost:8765` (for static hull files)

## Testing

### Test CLI script

```bash
cd /home/dreaming/cal_ws_moz2/interactive_workspace_viewer

# Test with default parameters (should complete in ~2s for sobol_m=12)
python3 scripts/recompute_hulls.py \
  --arm-upper-z 0.334 \
  --arm-elbow-y 0.025 \
  --arm-forearm-z 0.334 \
  --arm-wrist-z 0.050 \
  --sobol-m 12
```

### Test HTTP server

**Terminal 1 (start server):**
```bash
cd /home/dreaming/cal_ws_moz2/interactive_workspace_viewer
python3 scripts/hull_server.py
```

**Terminal 2 (test with curl):**
```bash
# Test cached request (should return immediately)
curl -X POST http://localhost:8765/recompute \
  -H 'Content-Type: application/json' \
  -d '{"arm_upper_z_total":0.334,"arm_elbow_y_offset":0.025,"arm_forearm_z_total":0.334,"arm_wrist_z":0.050,"sobol_m":12}'

# Test new parameters (will recompute)
curl -X POST http://localhost:8765/recompute \
  -H 'Content-Type: application/json' \
  -d '{"arm_upper_z_total":0.400,"arm_elbow_y_offset":0.025,"arm_forearm_z_total":0.334,"arm_wrist_z":0.050,"sobol_m":12}'

# Test static file serving
curl http://localhost:8765/parametric/variants/e9bb9f136bdd/config.json
```

## Integration with Frontend

Frontend should:
1. Start hull_server.py before running vite dev server
2. POST to `/api/recompute` with A-class parameters
3. Fetch hull JSONs from returned URLs
4. Replace existing hull meshes with new data

**Example frontend code:**
```typescript
async function recomputeHulls(params: ParametricParams) {
  const resp = await fetch('/api/recompute', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(params),
  });
  const result = await resp.json();
  
  const [left, right] = await Promise.all([
    fetch(result.left_hull_url).then(r => r.json()),
    fetch(result.right_hull_url).then(r => r.json()),
  ]);
  
  return { left, right };
}
```

## Performance

- **sobol_m=12** (4096 samples): ~2s per arm, suitable for preview
- **sobol_m=18** (262144 samples): ~30s per arm, production quality

## Dependencies

All dependencies already available in the workspace:
- numpy
- scipy (ConvexHull, qmc.Sobol)
- Python standard library (http.server, json, pathlib, etc.)

No additional packages required.
