import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const staticBasePath = new URL(".", import.meta.url).pathname.replace(/\/$/, "");
const APP_BASE = staticBasePath.endsWith("/static")
  ? staticBasePath.slice(0, -"/static".length)
  : "";
const API = {
  createJob: `${APP_BASE}/api/jobs`,
};

const STATUS_POLL_MS = 1000;
const JOB_STORAGE_KEY = `batchJobId:${APP_BASE || "root"}`;
const VFD = {
  background: 0x07110d,
  ambientSky: 0xa6ffe7,
  ambientGround: 0x021f17,
  sun: 0x7fffd6,
  gridMajor: 0x2af0b8,
  gridMinor: 0x0b4f3f,
  trail: 0x58ffd2,
  body: 0x10352a,
  bodyEmissive: 0x17b48b,
  arm: 0xf0b34a,
  rotorA: 0x71ffe3,
  rotorB: 0x1f8f74,
  velocity: 0xffc85c,
};

const sceneHost = document.querySelector("#scene3d");
const sceneCanvas = document.querySelector("#sceneCanvas");
const startButton = document.querySelector("#startSimulation");
const restartButton = document.querySelector("#restartSimulation");
const saveConditionsButton = document.querySelector("#saveConditions");
const playPauseButton = document.querySelector("#playPause");
const centerCameraButton = document.querySelector("#centerCamera");
const togglePathButton = document.querySelector("#togglePath");
const trackObjectButton = document.querySelector("#trackObject");
const statusValue = document.querySelector("#statusValue");
const timeValue = document.querySelector("#timeValue");
const truePositionValue = document.querySelector("#truePositionValue");
const positionValue = document.querySelector("#positionValue");
const gpsPositionValue = document.querySelector("#gpsPositionValue");
const gpsVelocityValue = document.querySelector("#gpsVelocityValue");
const trueVelocityValue = document.querySelector("#trueVelocityValue");
const velocityValue = document.querySelector("#velocityValue");
const trueQuaternionValue = document.querySelector("#trueQuaternionValue");
const estQuaternionValue = document.querySelector("#estQuaternionValue");
const sensedAccelValue = document.querySelector("#sensedAccelValue");
const baroAltitudeValue = document.querySelector("#baroAltitudeValue");
const rotorValue = document.querySelector("#rotorValue");
const target1zInput = document.querySelector("#target1z");

let telemetry = [];
let shape = null;
let conditions = null;
let simulationRunning = false;
let playing = false;
let playbackIndex = 0;
let playbackTime = 0;
let lastTickAt = performance.now();
let refreshTimer = null;
let trackingObject = false;
let lastTrackedPosition = null;
let droneGroup = null;
let velocityArrow = null;
let trailLine = null;
let pathVisible = false;
let bounds = null;
let sceneInitialized = false;
let lastShapeSignature = "";
let currentJobId = null;

function jobApi(path) {
  return `${APP_BASE}/api/jobs/${currentJobId}${path}`;
}

function startStatusPolling() {
  if (refreshTimer !== null) {
    return;
  }
  refreshTimer = window.setInterval(pollStatus, STATUS_POLL_MS);
}

function stopStatusPolling() {
  if (refreshTimer === null) {
    return;
  }
  window.clearInterval(refreshTimer);
  refreshTimer = null;
}

function setPlayPauseButtonState() {
  playPauseButton.classList.toggle("active", playing && telemetry.length > 0);
  playPauseButton.setAttribute("aria-pressed", (playing && telemetry.length > 0) ? "true" : "false");
}

const scene = new THREE.Scene();
scene.background = new THREE.Color(VFD.background);
scene.up.set(0, 0, 1);

const camera = new THREE.PerspectiveCamera(46, 1, 0.1, 10000);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true, canvas: sceneCanvas });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.screenSpacePanning = true;
controls.maxPolarAngle = Math.PI;
controls.minPolarAngle = 0;
controls.mouseButtons = {
  LEFT: THREE.MOUSE.ROTATE,
  MIDDLE: THREE.MOUSE.PAN,
  RIGHT: THREE.MOUSE.ROTATE,
};

scene.add(new THREE.HemisphereLight(VFD.ambientSky, VFD.ambientGround, 1.9));
const sunLight = new THREE.DirectionalLight(VFD.sun, 1.7);
sunLight.position.set(120, -160, 220);
scene.add(sunLight);

const worldGroup = new THREE.Group();
scene.add(worldGroup);

renderer.domElement.addEventListener("mousedown", (event) => {
  if (event.button === 1) {
    event.preventDefault();
  }
});
renderer.domElement.addEventListener("auxclick", (event) => {
  if (event.button === 1) {
    event.preventDefault();
  }
});

function setStatus(message) {
  statusValue.textContent = message;
}

function apiFetch(url, options = {}) {
  return fetch(url, { cache: "no-store", ...options });
}

async function createJob() {
  const response = await apiFetch(API.createJob, { method: "POST" });
  if (!response.ok) {
    throw new Error("Failed to create job");
  }
  const payload = await response.json();
  currentJobId = payload.job_id;
  window.sessionStorage.setItem(JOB_STORAGE_KEY, currentJobId);
  return payload;
}

async function ensureJob() {
  if (currentJobId) {
    return currentJobId;
  }
  const savedJobId = window.sessionStorage.getItem(JOB_STORAGE_KEY);
  if (savedJobId) {
    currentJobId = savedJobId;
    try {
      const response = await apiFetch(jobApi("/simulation/status"));
      if (response.ok) {
        return currentJobId;
      }
    } catch (error) {
      console.error(error);
    }
  }

  await createJob();
  return currentJobId;
}

async function apiFetchForJob(path, options = {}, allowRetry = true) {
  await ensureJob();
  const response = await apiFetch(jobApi(path), options);
  if (response.status !== 404 || !allowRetry) {
    return response;
  }

  currentJobId = null;
  window.sessionStorage.removeItem(JOB_STORAGE_KEY);
  await ensureJob();
  return apiFetch(jobApi(path), { cache: "no-store", ...options });
}

function optionalNumberFromInput(input) {
  const value = input.value.trim();
  return value === "" ? 0 : Number(value);
}

function setOptionalInputValue(input, value) {
  input.value = value == null ? "" : String(value);
}

function buildConditionsPayload(extra = {}) {
  return {
    target_1_z: optionalNumberFromInput(target1zInput),
    ...extra,
  };
}

function syncConditionsForm() {
  if (!conditions) {
    return;
  }
  setOptionalInputValue(target1zInput, conditions.target_1_z);
  updateConditionsHud();
}

function updateConditionsHud() {
  updatePausedIndicator();
}

function truePositionOf(row) {
  return {
    x: row.true_pos_x ?? row.pos_x,
    y: row.true_pos_y ?? row.pos_y,
    z: row.true_pos_z ?? row.pos_z,
  };
}

function estPositionOf(row) {
  return {
    x: row.est_pos_x ?? row.true_pos_x ?? row.pos_x,
    y: row.est_pos_y ?? row.true_pos_y ?? row.pos_y,
    z: row.est_pos_z ?? row.true_pos_z ?? row.pos_z,
  };
}

function estVelocityOf(row) {
  return {
    x: row.est_vel_x ?? row.vel_x,
    y: row.est_vel_y ?? row.vel_y,
    z: row.est_vel_z ?? row.vel_z,
  };
}

function trueQuaternionOf(row) {
  return {
    w: row.true_quat_w ?? row.quat_w,
    x: row.true_quat_x ?? row.quat_x,
    y: row.true_quat_y ?? row.quat_y,
    z: row.true_quat_z ?? row.quat_z,
  };
}

function estQuaternionOf(row) {
  return {
    w: row.est_quat_w ?? row.true_quat_w ?? row.quat_w,
    x: row.est_quat_x ?? row.true_quat_x ?? row.quat_x,
    y: row.est_quat_y ?? row.true_quat_y ?? row.quat_y,
    z: row.est_quat_z ?? row.true_quat_z ?? row.quat_z,
  };
}

function updatePausedIndicator() {
  setPlayPauseButtonState();
}

async function loadStatus() {
  const response = await apiFetchForJob("/simulation/status");
  if (!response.ok) {
    throw new Error("Failed to load status");
  }
  const status = await response.json();
  simulationRunning = Boolean(status.running);
  if (status.conditions) {
    conditions = { ...(conditions ?? {}), ...status.conditions };
    updateConditionsHud();
  }
  return status;
}

async function loadCompletedData() {
  const [telemetryResponse, shapeResponse] = await Promise.all([
    apiFetchForJob("/telemetry"),
    apiFetchForJob("/shape"),
  ]);

  if (!telemetryResponse.ok || !shapeResponse.ok) {
    throw new Error("Failed to load batch outputs");
  }

  telemetry = await telemetryResponse.json();
  shape = await shapeResponse.json();

  if (!telemetry.length) {
    clearPlayback();
    return;
  }

  bounds = makeBounds(telemetry);
  refreshSceneGeometry(true);
  resetPlayback();
}

async function initialize() {
  await ensureJob();
  const [status, conditionsResponse] = await Promise.all([
    loadStatus(),
    apiFetchForJob("/conditions"),
  ]);
  if (!conditionsResponse.ok) {
    throw new Error("Failed to load config");
  }
  conditions = await conditionsResponse.json();
  syncConditionsForm();
  if (!status.running) {
    await loadCompletedData();
    setStatus(telemetry.length ? "Visualizing Batch" : "Ready");
  } else {
    startStatusPolling();
    setStatus("Batch Calculating");
  }
}

async function saveConditions() {
  saveConditionsButton.disabled = true;
  setStatus("Saving config...");
  try {
    const response = await apiFetchForJob("/conditions", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(buildConditionsPayload()),
    });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.detail ?? "Failed to save config");
    }
    conditions = payload;
    syncConditionsForm();
    setStatus("Batch config saved.");
  } catch (error) {
    console.error(error);
    setStatus(error.message);
  } finally {
    saveConditionsButton.disabled = false;
  }
}

async function startSimulation() {
  startButton.disabled = true;
  restartButton.disabled = true;
  setStatus("Preparing batch...");
  try {
    await saveConditions();
    clearPlayback();
    const response = await apiFetchForJob("/simulation/start", { method: "POST" });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload?.detail?.step ? `${payload.detail.step} failed` : (payload.detail ?? "Failed to start batch"));
    }
    simulationRunning = true;
    conditions = payload.conditions ?? conditions;
    syncConditionsForm();
    playing = false;
    setPlayPauseButtonState();
    updatePausedIndicator();
    startStatusPolling();
    setStatus("Batch Calculating");
  } catch (error) {
    console.error(error);
    setStatus(error.message);
  } finally {
    startButton.disabled = false;
    restartButton.disabled = false;
  }
}

function restartPlayback() {
  if (simulationRunning) {
    setStatus("Batch is still calculating.");
    return;
  }
  if (!telemetry.length) {
    setStatus("No batch telemetry loaded.");
    return;
  }

  resetPlayback();
  playing = true;
  setPlayPauseButtonState();
  updatePausedIndicator();
  lastTickAt = performance.now();
  setStatus("Visualizing Batch");
}

async function pollStatus() {
  try {
    const previousRunning = simulationRunning;
    const status = await loadStatus();
    if (status.running) {
      setStatus("Batch Calculating");
      return;
    }

    if (previousRunning && !status.running) {
      stopStatusPolling();
      const conditionsResponse = await apiFetchForJob("/conditions");
      if (conditionsResponse.ok) {
        conditions = await conditionsResponse.json();
        syncConditionsForm();
      }
      await loadCompletedData();
      if (telemetry.length) {
        playing = true;
        setPlayPauseButtonState();
      }
      updatePausedIndicator();
      setStatus(telemetry.length ? "Visualizing Batch" : "Batch complete.");
    }
  } catch (error) {
    console.error(error);
    setStatus(error.message);
  }
}

function clearPlayback() {
  telemetry = [];
  shape = null;
  bounds = null;
  playbackIndex = 0;
  playbackTime = 0;
  playing = false;
  lastShapeSignature = "";
  lastTrackedPosition = null;
  timeValue.textContent = "--";
  truePositionValue.textContent = "--";
  positionValue.textContent = "--";
  gpsPositionValue.textContent = "--";
  gpsVelocityValue.textContent = "--";
  trueVelocityValue.textContent = "--";
  velocityValue.textContent = "--";
  trueQuaternionValue.textContent = "--";
  estQuaternionValue.textContent = "--";
  sensedAccelValue.textContent = "--";
  baroAltitudeValue.textContent = "--";
  rotorValue.textContent = "--";
  updatePausedIndicator();
  refreshSceneGeometry(true);
}

function resetPlayback() {
  if (!telemetry.length) {
    return;
  }
  playbackIndex = 0;
  playbackTime = Number(telemetry[0].time);
  lastTickAt = performance.now();
  updateFrame(rowAtIndex(playbackIndex));
  updatePausedIndicator();
}

function rowAtIndex(index) {
  if (!telemetry.length) {
    return null;
  }
  const clampedIndex = Math.max(0, Math.min(index, telemetry.length - 1));
  return telemetry[clampedIndex];
}

function currentRow() {
  return rowAtIndex(playbackIndex);
}

function advancePlayback(deltaSeconds) {
  if (!telemetry.length || playbackIndex >= telemetry.length - 1) {
    playing = false;
    setPlayPauseButtonState();
    updatePausedIndicator();
    return;
  }

  playbackTime += deltaSeconds;
  while (playbackIndex < telemetry.length - 1 && Number(telemetry[playbackIndex + 1].time) <= playbackTime) {
    playbackIndex += 1;
  }

  if (playbackIndex >= telemetry.length - 1) {
    playing = false;
    setPlayPauseButtonState();
    updatePausedIndicator();
  }
}

function makeBounds(rows) {
  const xs = rows.map((row) => truePositionOf(row).x);
  const ys = rows.map((row) => truePositionOf(row).y);
  const zs = rows.map((row) => truePositionOf(row).z);
  return {
    minX: Math.min(...xs),
    maxX: Math.max(...xs),
    minY: Math.min(...ys),
    maxY: Math.max(...ys),
    minZ: Math.min(...zs),
    maxZ: Math.max(...zs),
  };
}

function refreshSceneGeometry(forceRebuild) {
  const hasSceneData = Boolean(shape) || telemetry.length > 0;
  if (!hasSceneData && sceneInitialized && !forceRebuild) {
    return;
  }

  const nextShapeSignature = JSON.stringify(shape ?? {});
  const shouldRebuild = forceRebuild || !sceneInitialized || nextShapeSignature !== lastShapeSignature;
  if (shouldRebuild) {
    rebuildScene(!sceneInitialized);
    sceneInitialized = true;
    lastShapeSignature = nextShapeSignature;
    return;
  }

  updateTrailGeometry();
}

function rebuildScene(resetCamera) {
  clearWorld();

  const span = new THREE.Vector3(
    Math.max(1, (bounds?.maxX ?? 0) - (bounds?.minX ?? 0)),
    Math.max(1, (bounds?.maxY ?? 0) - (bounds?.minY ?? 0)),
    Math.max(1, (bounds?.maxZ ?? 0) - (bounds?.minZ ?? 0))
  );
  const radius = Math.max(span.x, span.y, span.z, 1);

  const grid = new THREE.GridHelper(12, 24, VFD.gridMajor, VFD.gridMinor);
  grid.rotation.x = Math.PI / 2;
  worldGroup.add(grid);

  if (telemetry.length) {
    const points = telemetry.map(centeredPosition);
    trailLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color: VFD.trail })
    );
    trailLine.visible = pathVisible;
    worldGroup.add(trailLine);
  }

  makeDroneModel(shape);
  if (!telemetry.length && droneGroup) {
    droneGroup.position.set(0, 0, 0);
  }
  if (resetCamera) {
    frameCamera(radius);
  }
}

function updateTrailGeometry() {
  if (!trailLine) {
    return;
  }
  const points = telemetry.map(centeredPosition);
  const nextGeometry = new THREE.BufferGeometry().setFromPoints(points);
  trailLine.geometry.dispose();
  trailLine.geometry = nextGeometry;
  trailLine.visible = pathVisible;
}

function clearWorld() {
  while (worldGroup.children.length) {
    const child = worldGroup.children.pop();
    disposeNode(child);
  }
  droneGroup = null;
  velocityArrow = null;
  trailLine = null;
}

function disposeNode(node) {
  node.traverse?.((part) => {
    if (part.geometry) {
      part.geometry.dispose();
    }
    if (part.material) {
      if (Array.isArray(part.material)) {
        part.material.forEach((material) => material.dispose());
      } else {
        part.material.dispose();
      }
    }
  });
}

function makeDroneModel(shapePayload) {
  const bodySize = shapePayload?.body_size ?? [0.2, 0.2, 0.05];
  const rotorPositions = shapePayload?.rotor_positions ?? [
    [0.141421, 0.141421, 0.0],
    [-0.141421, 0.141421, 0.0],
    [-0.141421, -0.141421, 0.0],
    [0.141421, -0.141421, 0.0],
  ];

  droneGroup = new THREE.Group();

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(bodySize[0], bodySize[1], bodySize[2]),
    new THREE.MeshStandardMaterial({
      color: VFD.body,
      emissive: VFD.bodyEmissive,
      emissiveIntensity: 0.38,
      roughness: 0.46,
      metalness: 0.28,
    })
  );
  droneGroup.add(body);

  const armMaterial = new THREE.MeshStandardMaterial({
    color: VFD.arm,
    emissive: 0x3c2200,
    emissiveIntensity: 0.18,
    roughness: 0.3,
    metalness: 0.18,
  });

  rotorPositions.forEach((position, index) => {
    const rotorVector = new THREE.Vector3(position[0], position[1], position[2]);
    const armLength = rotorVector.length();
    const arm = new THREE.Mesh(
      new THREE.CylinderGeometry(0.008, 0.008, armLength, 12),
      armMaterial
    );

    const midpoint = rotorVector.clone().multiplyScalar(0.5);
    arm.position.copy(midpoint);
    arm.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), rotorVector.clone().normalize());
    droneGroup.add(arm);

    const rotor = new THREE.Mesh(
      new THREE.CylinderGeometry(0.045, 0.045, 0.008, 28),
      new THREE.MeshStandardMaterial({
        color: index % 2 === 0 ? VFD.rotorA : VFD.rotorB,
        emissive: index % 2 === 0 ? 0x0d6651 : 0x06251d,
        emissiveIntensity: 0.2,
        roughness: 0.22,
        metalness: 0.22,
      })
    );
    rotor.position.set(position[0], position[1], position[2] + 0.02);
    rotor.rotation.x = Math.PI / 2;
    droneGroup.add(rotor);
  });

  const nose = new THREE.Mesh(
    new THREE.BoxGeometry(bodySize[0] * 0.26, bodySize[1] * 0.18, bodySize[2] * 0.4),
    armMaterial
  );
  nose.position.set(bodySize[0] * 0.55, 0, 0);
  droneGroup.add(nose);

  velocityArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.6, VFD.velocity, 0.16, 0.08);
  worldGroup.add(velocityArrow);
  worldGroup.add(droneGroup);
}

function frameCamera(radius, focus = new THREE.Vector3(0, 0, 0)) {
  camera.position.set(focus.x + 1.4, focus.y - 1.8, focus.z + 2.4);
  camera.near = 0.01;
  camera.far = Math.max(2000, radius * 8);
  camera.updateProjectionMatrix();
  controls.target.copy(focus);
  controls.maxDistance = 50;
  controls.update();
  resizeRenderer();
}

function centerCamera() {
  const spanX = bounds ? Math.max(1, bounds.maxX - bounds.minX) : 1;
  const spanY = bounds ? Math.max(1, bounds.maxY - bounds.minY) : 1;
  const spanZ = bounds ? Math.max(1, bounds.maxZ - bounds.minZ) : 1;
  frameCamera(Math.max(spanX, spanY, spanZ, 1), new THREE.Vector3(0, 0, 0));
  lastTrackedPosition = null;
}

function focusDrone() {
  const spanX = bounds ? Math.max(1, bounds.maxX - bounds.minX) : 1;
  const spanY = bounds ? Math.max(1, bounds.maxY - bounds.minY) : 1;
  const spanZ = bounds ? Math.max(1, bounds.maxZ - bounds.minZ) : 1;
  const focus = telemetry.length ? centeredPosition(currentRow()) : new THREE.Vector3(0, 0, 0);
  frameCamera(Math.max(spanX, spanY, spanZ, 1), focus);
  lastTrackedPosition = null;
}

function centeredPosition(row) {
  const position = truePositionOf(row);
  return new THREE.Vector3(position.x, position.y, position.z);
}

function setTrackingButtonState() {
  trackObjectButton.classList.toggle("active", trackingObject);
  trackObjectButton.setAttribute("aria-pressed", trackingObject ? "true" : "false");
}

function setPathButtonState() {
  togglePathButton.classList.toggle("active", pathVisible);
  togglePathButton.setAttribute("aria-pressed", pathVisible ? "true" : "false");
  if (trailLine) {
    trailLine.visible = pathVisible;
  }
  if (!pathVisible && velocityArrow) {
    velocityArrow.visible = false;
  } else if (pathVisible && telemetry.length) {
    updateFrame(currentRow());
  }
}

function toggleTrackObject() {
  trackingObject = !trackingObject;
  if (trackingObject) {
    focusDrone();
    lastTrackedPosition = telemetry.length ? centeredPosition(currentRow()) : null;
  } else {
    lastTrackedPosition = null;
  }
  setTrackingButtonState();
}

function togglePath() {
  pathVisible = !pathVisible;
  setPathButtonState();
}

function updateFrame(row) {
  if (!row || !droneGroup) {
    return;
  }

  const position = centeredPosition(row);
  if (trackingObject) {
    if (lastTrackedPosition) {
      const delta = position.clone().sub(lastTrackedPosition);
      camera.position.add(delta);
      controls.target.add(delta);
      controls.update();
    }
    lastTrackedPosition = position.clone();
  } else {
    lastTrackedPosition = null;
  }

  droneGroup.position.copy(position);
  const trueQuat = trueQuaternionOf(row);
  const estQuat = estQuaternionOf(row);
  droneGroup.quaternion.set(trueQuat.x, trueQuat.y, trueQuat.z, trueQuat.w).normalize();

  const estVelocity = estVelocityOf(row);
  const velocity = new THREE.Vector3(row.vel_x, row.vel_y, row.vel_z);
  const speedMagnitude = velocity.length();
  if (pathVisible && speedMagnitude > 1e-6) {
    velocityArrow.position.copy(position);
    velocityArrow.setDirection(velocity.clone().normalize());
    velocityArrow.setLength(Math.max(0.25, Math.min(1.5, speedMagnitude * 0.05)));
    velocityArrow.visible = true;
  } else {
    velocityArrow.visible = false;
  }

  timeValue.textContent = `${row.time.toFixed(2)}s`;
  const truePosition = truePositionOf(row);
  const estPosition = estPositionOf(row);
  truePositionValue.textContent = `${truePosition.x.toFixed(2)}, ${truePosition.y.toFixed(2)}, ${truePosition.z.toFixed(2)}`;
  positionValue.textContent = `${estPosition.x.toFixed(2)}, ${estPosition.y.toFixed(2)}, ${estPosition.z.toFixed(2)}`;
  gpsPositionValue.textContent = row.gps_pos_x != null && row.gps_pos_y != null && row.gps_pos_z != null
    ? `${row.gps_pos_x.toFixed(2)}, ${row.gps_pos_y.toFixed(2)}, ${row.gps_pos_z.toFixed(2)}`
    : "--";
  gpsVelocityValue.textContent = row.gps_vel_x != null && row.gps_vel_y != null && row.gps_vel_z != null
    ? `${row.gps_vel_x.toFixed(2)}, ${row.gps_vel_y.toFixed(2)}, ${row.gps_vel_z.toFixed(2)}`
    : "--";
  trueVelocityValue.textContent = `${row.vel_x.toFixed(2)}, ${row.vel_y.toFixed(2)}, ${row.vel_z.toFixed(2)}`;
  velocityValue.textContent = `${estVelocity.x.toFixed(2)}, ${estVelocity.y.toFixed(2)}, ${estVelocity.z.toFixed(2)}`;
  trueQuaternionValue.textContent = `${trueQuat.w.toFixed(3)}, ${trueQuat.x.toFixed(3)}, ${trueQuat.y.toFixed(3)}, ${trueQuat.z.toFixed(3)}`;
  estQuaternionValue.textContent = `${estQuat.w.toFixed(3)}, ${estQuat.x.toFixed(3)}, ${estQuat.y.toFixed(3)}, ${estQuat.z.toFixed(3)}`;
  sensedAccelValue.textContent = row.sensed_accel_x != null && row.sensed_accel_y != null && row.sensed_accel_z != null
    ? `${row.sensed_accel_x.toFixed(2)}, ${row.sensed_accel_y.toFixed(2)}, ${row.sensed_accel_z.toFixed(2)}`
    : "--";
  baroAltitudeValue.textContent = row.baro_altitude != null ? row.baro_altitude.toFixed(2) : "--";
  rotorValue.textContent = `${row.omega_1.toFixed(0)}, ${row.omega_2.toFixed(0)}, ${row.omega_3.toFixed(0)}, ${row.omega_4.toFixed(0)}`;
}

function tick(now) {
  const deltaSeconds = Math.max(0, now - lastTickAt) / 1000;
  lastTickAt = now;

  if (playing && telemetry.length) {
    advancePlayback(deltaSeconds);
    updateFrame(currentRow());
  }

  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

function resizeRenderer() {
  const width = sceneHost.clientWidth;
  const height = sceneHost.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

startButton.addEventListener("click", startSimulation);
restartButton.addEventListener("click", restartPlayback);
saveConditionsButton.addEventListener("click", saveConditions);
centerCameraButton.addEventListener("click", centerCamera);
togglePathButton.addEventListener("click", togglePath);
trackObjectButton.addEventListener("click", toggleTrackObject);
playPauseButton.addEventListener("click", () => {
  if (simulationRunning) {
    setStatus("Batch is still calculating.");
    return;
  }
  if (!telemetry.length) {
    setStatus("No batch telemetry loaded.");
    return;
  }
  playing = !playing;
  setPlayPauseButtonState();
  updatePausedIndicator();
  lastTickAt = performance.now();
  setStatus("Visualizing Batch");
});
window.addEventListener("resize", resizeRenderer);

resizeRenderer();
setTrackingButtonState();
setPathButtonState();
setPlayPauseButtonState();
initialize()
  .catch((error) => {
    console.error(error);
    setStatus(error.message);
  })
  .finally(() => {
    requestAnimationFrame(tick);
  });
