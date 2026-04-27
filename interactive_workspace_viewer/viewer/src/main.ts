import { SceneManager } from './scene/SceneManager.ts';
import { RobotSkeleton } from './scene/RobotSkeleton.ts';
import { HullMesh } from './scene/HullMesh.ts';
import { FrameAxes } from './scene/FrameAxes.ts';
import { ZUpGrid } from './scene/GridHelper.ts';
import { TaskPointMarker } from './scene/TaskPointMarker.ts';
import { InfoOverlay } from './ui/InfoOverlay.ts';
import { ControlPanel } from './ui/ControlPanel.ts';
import { StatusPanel } from './ui/StatusPanel.ts';
import { CameraPresets } from './ui/CameraPresets.ts';
import { TaskPointPanel } from './ui/TaskPointPanel.ts';
import { loadSkeleton } from './loaders/SkeletonLoader.ts';
import { loadHull } from './loaders/HullLoader.ts';
import { loadJointLimits } from './loaders/JointLimitsLoader.ts';
import { ForwardKinematics } from './kinematics/ForwardKinematics.ts';
import type { JointState } from './kinematics/types.ts';

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

  // 3. Warning overlay
  new InfoOverlay();

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

  // 5. FK engine
  const fk = new ForwardKinematics(skeleton);

  // 6. Joint state (Phase 2: waist only)
  let jointState: JointState = {
    'LegWaist-1': 0,
    'LegWaist-2': 0,
    'LegWaist-3': 0,
  };

  // 7. Scene objects
  const robot = new RobotSkeleton(skeleton);
  sceneManager.scene.add(robot.group);

  const axes = new FrameAxes(skeleton);
  sceneManager.scene.add(axes.group);

  let leftMesh: HullMesh | null = null;
  let rightMesh: HullMesh | null = null;
  if (leftHull) {
    leftMesh = new HullMesh(leftHull);
    sceneManager.scene.add(leftMesh.group);
  }
  if (rightHull) {
    rightMesh = new HullMesh(rightHull);
    sceneManager.scene.add(rightMesh.group);
  }

  // 8. Status panel
  const statusPanel = new StatusPanel();

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

  // 9. Update function
  function updateAll() {
    const result = fk.compute(jointState);
    robot.update(result);
    axes.update(result);
    const w03 = result.linkTransforms['waist03'];
    if (w03) {
      leftMesh?.update(w03);
      rightMesh?.update(w03);
    }
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

  // 12. Initial update + FK verification
  updateAll();
  verifyZeroPoseFK(fk);

  // 13. Start render loop
  sceneManager.start();
  console.log('[Viewer] MOZ2 Workspace Viewer Phase 3.1 started.');
}

function verifyZeroPoseFK(fk: ForwardKinematics) {
  const result = fk.compute({});
  const checks: [string, [number, number, number], number][] = [
    ['waist03', [0.0, 0.1114, 1.3595], 1e-5],
    ['left_ee_site', [0.212, 0.0864, 0.5415], 1e-4],
    ['right_ee_site', [-0.212, 0.0864, 0.5415], 1e-4],
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
