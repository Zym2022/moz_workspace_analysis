import * as THREE from 'three';
import { SceneManager } from './scene/SceneManager.ts';
import { RobotSkeleton } from './scene/RobotSkeleton.ts';
import { HullMesh } from './scene/HullMesh.ts';
import { FrameAxes } from './scene/FrameAxes.ts';
import { ZUpGrid } from './scene/GridHelper.ts';
import { TaskPointMarker } from './scene/TaskPointMarker.ts';
import { HullProjectionDimensions } from './scene/HullProjectionDimensions.ts';
import { ProjectionGrid } from './scene/ProjectionGrid.ts';
import { ControlPanel } from './ui/ControlPanel.ts';
import { StatusPanel } from './ui/StatusPanel.ts';
import { CameraPresets } from './ui/CameraPresets.ts';
import { TaskPointPanel } from './ui/TaskPointPanel.ts';
import { ProjectionDimensionPanel } from './ui/ProjectionDimensionPanel.ts';
import { loadSkeleton } from './loaders/SkeletonLoader.ts';
import { loadHull } from './loaders/HullLoader.ts';
import { loadJointLimits } from './loaders/JointLimitsLoader.ts';
import { ForwardKinematics } from './kinematics/ForwardKinematics.ts';
import { ParametricSkeleton, DEFAULT_PARAMS } from './kinematics/ParametricSkeleton.ts';
import { ParametricPanel } from './ui/ParametricPanel.ts';
import { ROIVoxelGrid } from './scene/ROIVoxelGrid.ts';
import { ROIPanel } from './ui/ROIPanel.ts';
import { loadStaticROI, computeROI } from './loaders/ROILoader.ts';
import type { JointState, ParametricParams, SkeletonData, HullData } from './kinematics/types.ts';

const COLOR_LEFT = 0x4488ff;
const COLOR_RIGHT = 0x44bb44;

function withCacheBust(url: string, token: string): string {
  const sep = url.includes('?') ? '&' : '?';
  return `${url}${sep}v=${encodeURIComponent(token)}`;
}

async function fetchFreshJson<T>(url: string, token: string): Promise<T> {
  const resp = await fetch(withCacheBust(url, token), { cache: 'no-store' });
  if (!resp.ok) {
    throw new Error(`Failed to fetch ${url}: ${resp.status} ${resp.statusText}`);
  }
  return resp.json() as Promise<T>;
}

async function main() {
  const container = document.getElementById('app');
  if (!container) {
    console.error('No #app container found');
    return;
  }

  // 1. Scene setup
  const sceneManager = new SceneManager(container);

  // 2. Grid
  const grid = new ZUpGrid();
  sceneManager.scene.add(grid.group);

  // 4. Load data
  const [skeleton, jointLimits, leftHull, rightHull] = await Promise.all([
    loadSkeleton(),
    loadJointLimits(),
    loadHull('left'),
    loadHull('right'),
  ]);

  if (!skeleton) {
    console.warn('[Viewer] No skeleton data loaded. Showing grid only.');
    sceneManager.start();
    return;
  }

  // 5. FK engine + parametric skeleton
  const baseSkeleton = skeleton;
  const paramSkeleton = new ParametricSkeleton(baseSkeleton);
  let activeParams: ParametricParams = { ...DEFAULT_PARAMS };
  paramSkeleton.verifyDefaults();

  let fk = new ForwardKinematics(skeleton);

  // 6. Joint state (Phase 2: waist only)
  let jointState: JointState = {
    'LegWaist-1': 0,
    'LegWaist-2': 0,
    'LegWaist-3': 0,
  };

  // 7. Scene objects
  let robot = new RobotSkeleton(skeleton);
  sceneManager.scene.add(robot.group);

  let axes = new FrameAxes(skeleton);
  sceneManager.scene.add(axes.group);

  let leftMesh: HullMesh | null = null;
  let rightMesh: HullMesh | null = null;
  let leftProjection: HullProjectionDimensions | null = null;
  let rightProjection: HullProjectionDimensions | null = null;
  if (leftHull) {
    leftMesh = new HullMesh(leftHull);
    leftMesh.setVisible(false);
    sceneManager.scene.add(leftMesh.group);
    leftProjection = new HullProjectionDimensions(leftMesh, COLOR_LEFT);
    leftProjection.group.visible = false;
    sceneManager.scene.add(leftProjection.group);
  }
  if (rightHull) {
    rightMesh = new HullMesh(rightHull);
    rightMesh.setVisible(false);
    sceneManager.scene.add(rightMesh.group);
    rightProjection = new HullProjectionDimensions(rightMesh, COLOR_RIGHT, true);
    rightProjection.group.visible = false;
    sceneManager.scene.add(rightProjection.group);
  }

  const projectionGrid = new ProjectionGrid();
  projectionGrid.group.visible = false;
  sceneManager.scene.add(projectionGrid.group);

  // 8. Status panel
  const statusPanel = new StatusPanel();

  // 8a. Projection dimension panel
  const projectionPanel = new ProjectionDimensionPanel();
  projectionPanel.setProjections(leftProjection, rightProjection);
  projectionPanel.onChange = (mode) => {
    leftProjection?.setMode(mode);
    rightProjection?.setMode(mode);
    projectionGrid.setMode(mode);
    updateAll();
  };
  projectionPanel.onHullVisibilityChange = (visible) => {
    leftMesh?.setVisible(visible);
    rightMesh?.setVisible(visible);
  };

  // 8b. Task target point: marker (scene) + panel (top-left)
  const taskMarker = new TaskPointMarker(
    sceneManager.camera,
    sceneManager.renderer,
    sceneManager.controls,
    sceneManager.scene,
  );
  sceneManager.scene.add(taskMarker.group);
  taskMarker.setPosition([0, 0, 0.8]);
  taskMarker.setVisible(true);
  taskMarker.setPreviewMode(true);

  const taskPanel = new TaskPointPanel();

  // 8c. ROI manipulability grid + panel (created early so updateAll can reference them)
  const roiGrid = new ROIVoxelGrid();
  sceneManager.scene.add(roiGrid.group);
  const roiPanel = new ROIPanel();

  // 9. Update function
  function updateAll() {
    const result = fk.compute(jointState);
    robot.update(result);
    axes.update(result);
    const w01 = result.linkTransforms['waist01'];
    if (w01) {
      leftMesh?.update(w01);
      rightMesh?.update(w01);
      roiGrid.update(w01);
      // Update ROI panel bounds for all loaded ROIs
      for (const roiId of roiGrid.getROIIds()) {
        const bounds = roiGrid.getTransformedBounds(roiId);
        if (bounds) roiPanel.updateBounds(roiId, bounds);
      }
    }
    leftProjection?.update();
    rightProjection?.update();

    const leftBounds = new THREE.Box3();
    const rightBounds = new THREE.Box3();
    if (leftMesh) leftMesh.getWorldAabb(leftBounds);
    if (rightMesh) rightMesh.getWorldAabb(rightBounds);
    projectionGrid.update(leftProjection, rightProjection, leftBounds, rightBounds);

    projectionPanel.updateDisplay();
    const taskPos = taskPanel.isPlaced() ? taskPanel.getPosition() : null;
    statusPanel.update(result, jointState, taskPos);
  }

  // Bidirectional sync between panel and 3D marker
  let syncing = false;
  taskPanel.onPositionChange = (pos) => {
    if (syncing) return;
    syncing = true;
    taskMarker.setPosition(pos);
    syncing = false;
    updateAll();
  };
  taskMarker.onDrag = (pos) => {
    if (syncing) return;
    syncing = true;
    taskPanel.updatePosition(pos);
    syncing = false;
    updateAll();
  };
  taskPanel.onPlace = () => {
    taskMarker.setVisible(true);
    taskMarker.setPreviewMode(false);
    taskMarker.setDraggable(true);
    updateAll();
  };
  taskPanel.onRemove = () => {
    taskMarker.setVisible(false);
    taskMarker.setDraggable(false);
    updateAll();
  };

  // 10. Control panel
  if (jointLimits) {
    const controlPanel = new ControlPanel(jointLimits);
    controlPanel.onChange = (state) => {
      jointState = state;
      updateAll();
    };
  } else {
    console.warn('[Viewer] No joint limits loaded; ControlPanel disabled.');
  }

  // 11. Camera presets
  const cameraPresets = new CameraPresets(sceneManager.camera, sceneManager.controls);
  sceneManager.onBeforeRender = () => {
    cameraPresets.update();
  };

  // Store default hull data for reset
  const defaultLeftHull = leftHull;
  const defaultRightHull = rightHull;

  // Helper: rebuild skeleton/FK/axes from a SkeletonData
  function applySkeleton(sk: SkeletonData) {
    fk = new ForwardKinematics(sk);
    sceneManager.scene.remove(robot.group);
    robot = new RobotSkeleton(sk);
    sceneManager.scene.add(robot.group);
    sceneManager.scene.remove(axes.group);
    axes = new FrameAxes(sk);
    sceneManager.scene.add(axes.group);
  }

  // 11b. Parametric geometry panel
  const parametricPanel = new ParametricPanel(DEFAULT_PARAMS);
  let currentRequestId = 0;

  parametricPanel.onApply = async (params: ParametricParams) => {
    const armChanged = paramSkeleton.hasArmShapeChanged(params, activeParams);
    const requestId = ++currentRequestId;

    if (!armChanged) {
      // B-class only: apply immediately, no recompute
      const newSkeleton = paramSkeleton.apply(params);
      applySkeleton(newSkeleton);
      const delta = paramSkeleton.getShoulderDelta(params);
      leftMesh?.setShoulderDelta(delta);
      rightMesh?.setShoulderDelta(-delta);
      activeParams = { ...params };
      parametricPanel.setStatus('Applied');
      updateAll();
      return;
    }

    // A-class changed: mark stale but keep current skeleton until preview succeeds
    leftMesh?.setStale(true);
    rightMesh?.setStale(true);
    parametricPanel.setHullStatus('Recomputing preview...');

    const aClassBody = {
      arm_upper_z_total: params.arm_upper_z_total,
      arm_elbow_y_offset: params.arm_elbow_y_offset,
      arm_forearm_z_total: params.arm_forearm_z_total,
      arm_wrist_z: params.arm_wrist_z,
    };

    try {
      const previewResp = await fetch('/api/recompute', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ...aClassBody, sobol_m: 12 }),
      });
      if (requestId !== currentRequestId) return;

      if (!previewResp.ok) {
        parametricPanel.setHullStatus('Recompute failed — is hull_server running?');
        leftMesh?.setStale(false);
        rightMesh?.setStale(false);
        return;
      }

      const previewResult = await previewResp.json();
      const [previewLeft, previewRight] = await Promise.all([
        fetchFreshJson<HullData>(previewResult.left_hull_url, `${requestId}-preview-left`),
        fetchFreshJson<HullData>(previewResult.right_hull_url, `${requestId}-preview-right`),
      ]);
      if (requestId !== currentRequestId) return;

      // Preview succeeded: now commit skeleton + hull + params atomically
      const newSkeleton = paramSkeleton.apply(params);
      applySkeleton(newSkeleton);
      const delta = paramSkeleton.getShoulderDelta(params);
      leftMesh?.setShoulderDelta(delta);
      rightMesh?.setShoulderDelta(-delta);
      leftMesh?.replaceHull(previewLeft);
      rightMesh?.replaceHull(previewRight);
      leftMesh?.setStale(false);
      rightMesh?.setStale(false);
      activeParams = { ...params };
      parametricPanel.setStatus('Applied');
      updateAll();

      if (previewResult.sobol_m >= 18) {
        parametricPanel.setHullStatus(`Final (2^${previewResult.sobol_m})`);
      } else {
        parametricPanel.setHullStatus(`Preview (2^${previewResult.sobol_m}) — computing final...`);

        fetch('/api/recompute', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ ...aClassBody, sobol_m: 18 }),
        }).then(async (finalResp) => {
          if (requestId !== currentRequestId) return;
          if (!finalResp.ok) return;
          const finalResult = await finalResp.json();
          const [finalLeft, finalRight] = await Promise.all([
            fetchFreshJson<HullData>(finalResult.left_hull_url, `${requestId}-final-left`),
            fetchFreshJson<HullData>(finalResult.right_hull_url, `${requestId}-final-right`),
          ]);
          if (requestId !== currentRequestId) return;
          leftMesh?.replaceHull(finalLeft);
          rightMesh?.replaceHull(finalRight);
          parametricPanel.setHullStatus(`Final (2^${finalResult.sobol_m})`);
          updateAll();
        }).catch(() => {
          if (requestId === currentRequestId) {
            parametricPanel.setHullStatus(`Preview only — final computation failed`);
          }
        });
      }
    } catch {
      if (requestId !== currentRequestId) return;
      parametricPanel.setHullStatus('Server unreachable — hull unchanged');
      leftMesh?.setStale(false);
      rightMesh?.setStale(false);
    }
  };

  parametricPanel.onReset = () => {
    currentRequestId++;
    fk = new ForwardKinematics(baseSkeleton);

    sceneManager.scene.remove(robot.group);
    robot = new RobotSkeleton(baseSkeleton);
    sceneManager.scene.add(robot.group);

    sceneManager.scene.remove(axes.group);
    axes = new FrameAxes(baseSkeleton);
    sceneManager.scene.add(axes.group);

    leftMesh?.setShoulderDelta(0);
    rightMesh?.setShoulderDelta(0);
    leftMesh?.setStale(false);
    rightMesh?.setStale(false);

    // Restore default hull meshes
    if (defaultLeftHull) leftMesh?.replaceHull(defaultLeftHull);
    if (defaultRightHull) rightMesh?.replaceHull(defaultRightHull);

    activeParams = { ...DEFAULT_PARAMS };
    parametricPanel.setStatus('Default params');
    parametricPanel.setHullStatus('Current (2^18)');
    updateAll();
  };

  // 12. Initial update + FK verification
  updateAll();
  verifyZeroPoseFK(fk);

  // 13. ROI manipulability wiring
  // Set ROI zero-pose reference for waist tracking
  const zeroResult = fk.compute({ 'LegWaist-1': 0, 'LegWaist-2': 0, 'LegWaist-3': 0 });
  const zeroW01 = zeroResult.linkTransforms['waist01'];
  if (zeroW01) {
    roiGrid.setZeroPoseTransform(zeroW01);
  }

  roiPanel.onVisibilityChange = (roiId, visible) => roiGrid.setVisible(roiId, visible);
  roiPanel.onOpacityChange = (roiId, opacity) => roiGrid.setOpacity(roiId, opacity);
  roiPanel.onShowUnreachableChange = (roiId, show) => roiGrid.setShowUnreachable(roiId, show);
  roiPanel.onShowBestConfig = (_roiId, bestQ) => {
    const armJointNames = ['LeftArm-0', 'LeftArm-1', 'LeftArm-2', 'LeftArm-3', 'LeftArm-4', 'LeftArm-5', 'LeftArm-6'];
    for (let i = 0; i < 7; i++) {
      jointState[armJointNames[i]] = bestQ[i];
    }
    updateAll();
  };
  roiPanel.onCompute = async (params) => {
    roiPanel.setStatus('Computing ROI...');
    const data = await computeROI(params);
    if (data) {
      roiGrid.addROI(data);
      roiPanel.addROI(data);
      roiPanel.setStatus('Computed');
    } else {
      roiPanel.setStatus('Compute failed — is hull_server running?');
    }
  };

  // Auto-load chest_front (case4) + chest_front_down (case5)
  for (const id of ['chest_front', 'chest_front_down']) {
    loadStaticROI(id).then((data) => {
      if (data) {
        roiGrid.addROI(data);
        roiPanel.addROI(data);
      }
    });
  }

  // 14. Start render loop
  sceneManager.start();
  console.log('[Viewer] MOZ2 Workspace Viewer Phase 3.1 started.');
}

function verifyZeroPoseFK(fk: ForwardKinematics) {
  const result = fk.compute({});
  const checks: [string, [number, number, number], number][] = [
    ['waist01', [0.0, 0.1575, 1.041], 1e-5],
    ['left_ee_site', [0.212, 0.1425, 0.556], 1e-4],
    ['right_ee_site', [-0.212, 0.1425, 0.556], 1e-4],
  ];
  for (const [name, expected, tol] of checks) {
    const t = result.linkTransforms[name];
    if (!t) {
      console.error(`[FK Verify] MISSING: ${name}`);
      continue;
    }
    const p = t.position;
    const err = Math.hypot(p[0] - expected[0], p[1] - expected[1], p[2] - expected[2]);
    const status = err < tol ? 'PASS' : 'FAIL';
    console.log(
      `[FK Verify] ${name}: [${p[0].toFixed(6)}, ${p[1].toFixed(6)}, ${p[2].toFixed(6)}] ` +
      `expected [${expected.join(', ')}] err=${err.toExponential(3)} ${status}`,
    );
  }
}

main();
