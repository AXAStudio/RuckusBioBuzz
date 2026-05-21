const video = document.getElementById("video");
const overlay = document.getElementById("overlay");
const ctx = overlay.getContext("2d", { willReadFrequently: true });
const emptyState = document.getElementById("emptyState");
const cameraSelect = document.getElementById("cameraSelect");
const startButton = document.getElementById("startButton");
const snapshotButton = document.getElementById("snapshotButton");
const showMask = document.getElementById("showMask");
const mirrorVideo = document.getElementById("mirrorVideo");
const videoWrap = document.querySelector(".video-wrap");

const controls = {
  minArea: bindRange("minArea"),
  singleBallArea: bindRange("singleBallArea"),
  mergeGap: bindRange("mergeGap"),
};

const HSV_LO_A = [15, 90, 70];
const HSV_HI_A = [38, 255, 255];
const HSV_LO_B = [12, 60, 45];
const HSV_HI_B = [42, 255, 255];
const OPEN_RADIUS = 3;
const CLOSE_H_GAP = 16;
const CLOSE_V_RADIUS = 12;
const MAX_AREA = 100000;
const MAX_ASPECT = 5;
const MAX_RUNS = 32000;

const clumpCount = document.getElementById("clumpCount");
const bestLabel = document.getElementById("bestLabel");
const steeringLabel = document.getElementById("steeringLabel");
const fpsLabel = document.getElementById("fpsLabel");

let stream = null;
let sourceCanvas = document.createElement("canvas");
let sourceCtx = sourceCanvas.getContext("2d", { willReadFrequently: true });
let animationId = 0;
let lastFpsTime = performance.now();
let frames = 0;

function bindRange(id) {
  const input = document.getElementById(id);
  const output = document.getElementById(`${id}Value`);
  const sync = () => {
    output.value = input.value;
  };
  input.addEventListener("input", sync);
  sync();
  return input;
}

async function listCameras() {
  cameraSelect.innerHTML = "";
  const devices = await navigator.mediaDevices.enumerateDevices();
  const cameras = devices.filter((device) => device.kind === "videoinput");

  cameras.forEach((device, index) => {
    const option = document.createElement("option");
    option.value = device.deviceId;
    option.textContent = device.label || `Camera ${index + 1}`;
    cameraSelect.appendChild(option);
  });

  if (cameras.length === 0) {
    const option = document.createElement("option");
    option.textContent = "No cameras found";
    cameraSelect.appendChild(option);
  }
}

async function startCamera() {
  stopCamera();
  const deviceId = cameraSelect.value;
  const constraints = {
    video: {
      width: { ideal: 640 },
      height: { ideal: 480 },
      frameRate: { ideal: 30 },
      ...(deviceId ? { deviceId: { exact: deviceId } } : {}),
    },
    audio: false,
  };

  stream = await navigator.mediaDevices.getUserMedia(constraints);
  video.srcObject = stream;
  await video.play();
  await listCameras();
  snapshotButton.disabled = false;
  emptyState.style.display = "none";
  startButton.textContent = "Restart Camera";
  resizeCanvases();
  animationId = requestAnimationFrame(processFrame);
}

function stopCamera() {
  cancelAnimationFrame(animationId);
  if (stream) {
    stream.getTracks().forEach((track) => track.stop());
    stream = null;
  }
}

function resizeCanvases() {
  const width = video.videoWidth || 640;
  const height = video.videoHeight || 480;
  if (overlay.width !== width || overlay.height !== height) {
    overlay.width = width;
    overlay.height = height;
    sourceCanvas.width = width;
    sourceCanvas.height = height;
  }
}

function processFrame(now) {
  resizeCanvases();
  const width = overlay.width;
  const height = overlay.height;

  sourceCtx.drawImage(video, 0, 0, width, height);
  const frame = sourceCtx.getImageData(0, 0, width, height);
  const mask = buildMask(frame.data, width, height);
  const runs = rleClose(rleOpen(buildRle(mask, width, height), width), width);
  const labeled = labelRuns(runs);
  const clumps = buildClumps(labeled.components, width);
  const displayMask = showMask.checked ? runsToMask(runs, width, height) : mask;
  draw(frame, displayMask, clumps, width, height);
  updateMetrics(clumps, now);

  animationId = requestAnimationFrame(processFrame);
}

function buildMask(data, width, height) {
  const mask = new Uint8Array(width * height);

  for (let i = 0, p = 0; i < data.length; i += 4, p++) {
    const [h, s, v] = rgbToOpenCvHsv(data[i], data[i + 1], data[i + 2]);
    if (inRange([h, s, v], HSV_LO_A, HSV_HI_A) || inRange([h, s, v], HSV_LO_B, HSV_HI_B)) {
      mask[p] = 1;
    }
  }
  return mask;
}

function inRange(hsv, low, high) {
  return hsv[0] >= low[0] && hsv[0] <= high[0] && hsv[1] >= low[1] && hsv[1] <= high[1] && hsv[2] >= low[2] && hsv[2] <= high[2];
}

function rgbToOpenCvHsv(r, g, b) {
  const rf = r / 255;
  const gf = g / 255;
  const bf = b / 255;
  const max = Math.max(rf, gf, bf);
  const min = Math.min(rf, gf, bf);
  const delta = max - min;
  let h = 0;

  if (delta !== 0) {
    if (max === rf) h = 60 * (((gf - bf) / delta) % 6);
    else if (max === gf) h = 60 * ((bf - rf) / delta + 2);
    else h = 60 * ((rf - gf) / delta + 4);
  }

  if (h < 0) h += 360;
  const s = max === 0 ? 0 : delta / max;
  return [Math.round(h / 2), Math.round(s * 255), Math.round(max * 255)];
}

function buildRle(mask, width, height) {
  const runs = [];
  for (let y = 0; y < height && runs.length < MAX_RUNS - 1; y++) {
    const base = y * width;
    let inside = false;
    let start = 0;
    for (let x = 0; x < width; x++) {
      const foreground = mask[base + x] !== 0;
      if (foreground && !inside) {
        start = x;
        inside = true;
      } else if (!foreground && inside) {
        runs.push({ row: y, start, end: x - 1 });
        inside = false;
      }
    }
    if (inside) runs.push({ row: y, start, end: width - 1 });
  }
  return runs;
}

function rleOpen(inputRuns, width) {
  if (inputRuns.length === 0) return [];
  const runs = sortRuns(inputRuns);
  const opened = [];

  for (let i = 0; i < runs.length; i++) {
    const run = runs[i];
    const nextStart = run.start + OPEN_RADIUS;
    const nextEnd = run.end - OPEN_RADIUS;
    if (nextStart > nextEnd) continue;

    let hasNeighbor = false;
    for (let j = 0; j < runs.length && !hasNeighbor; j++) {
      const other = runs[j];
      const dy = other.row - run.row;
      if (dy === 0 || dy < -OPEN_RADIUS) continue;
      if (dy > OPEN_RADIUS) break;
      if (other.start <= nextEnd && other.end >= nextStart) hasNeighbor = true;
    }
    if (!hasNeighbor) continue;

    opened.push({
      row: run.row,
      start: Math.max(0, nextStart - OPEN_RADIUS),
      end: Math.min(width - 1, nextEnd + OPEN_RADIUS),
    });
  }

  return opened;
}

function rleClose(inputRuns, width) {
  if (inputRuns.length === 0) return [];
  const dilated = inputRuns.map((run) => ({
    row: run.row,
    start: Math.max(0, run.start - CLOSE_H_GAP),
    end: Math.min(width - 1, run.end + CLOSE_H_GAP),
  }));

  const runs = sortRuns(dilated);
  const merged = [];
  for (const run of runs) {
    if (merged.length === 0) {
      merged.push({ ...run });
      continue;
    }

    const previous = merged[merged.length - 1];
    const dy = run.row - previous.row;
    if (dy <= CLOSE_V_RADIUS && run.start <= previous.end + 1) {
      if (run.end > previous.end) previous.end = run.end;
    } else {
      merged.push({ ...run });
    }
  }

  const closed = [];
  for (const run of merged) {
    const nextStart = run.start + CLOSE_H_GAP;
    const nextEnd = run.end - CLOSE_H_GAP;
    if (nextStart <= nextEnd) {
      closed.push({ row: run.row, start: nextStart, end: nextEnd });
    }
  }
  return closed;
}

function labelRuns(inputRuns) {
  const runs = sortRuns(inputRuns);
  const parent = runs.map((_, index) => index);
  const rank = runs.map(() => 0);

  let previousStart = 0;
  for (let i = 0; i < runs.length; i++) {
    const currentRow = runs[i].row;
    while (previousStart < i && runs[previousStart].row < currentRow - 1) previousStart++;
    for (let j = previousStart; j < i; j++) {
      if (runs[j].row !== currentRow - 1) continue;
      if (runs[j].start > runs[i].end || runs[j].end < runs[i].start) continue;
      union(parent, rank, i, j);
    }
  }

  const components = new Map();
  for (let i = 0; i < runs.length; i++) {
    const root = find(parent, i);
    const run = runs[i];
    const length = run.end - run.start + 1;
    const midX = run.start + Math.floor(length / 2);
    const component = components.get(root) || {
      sumX: 0,
      sumY: 0,
      area: 0,
      minX: Infinity,
      minY: Infinity,
      maxX: -Infinity,
      maxY: -Infinity,
    };
    component.area += length;
    component.sumX += midX * length;
    component.sumY += run.row * length;
    component.minX = Math.min(component.minX, run.start);
    component.minY = Math.min(component.minY, run.row);
    component.maxX = Math.max(component.maxX, run.end);
    component.maxY = Math.max(component.maxY, run.row);
    components.set(root, component);
  }

  return { runs, components: [...components.values()] };
}

function buildClumps(components, frameWidth) {
  const minArea = Number(controls.minArea.value);
  const gap = Number(controls.mergeGap.value);
  const singleBallArea = Number(controls.singleBallArea.value);
  const valid = [];

  for (const component of components) {
    const area = component.area;
    if (area < minArea || area > MAX_AREA) continue;
    const boxW = component.maxX - component.minX + 1;
    const boxH = component.maxY - component.minY + 1;
    if (boxH === 0) continue;
    const aspect = boxW / boxH;
    if (aspect > MAX_ASPECT || aspect < 1 / MAX_ASPECT) continue;
    valid.push({
      centerX: component.sumX / area,
      centerY: component.sumY / area,
      area,
      x1: component.minX,
      y1: component.minY,
      x2: component.maxX,
      y2: component.maxY,
    });
  }

  const parent = valid.map((_, index) => index);
  const rank = valid.map(() => 0);

  for (let i = 0; i < valid.length; i++) {
    for (let j = i + 1; j < valid.length; j++) {
      if (boxesClose(valid[i], valid[j], gap)) union(parent, rank, i, j);
    }
  }

  const groups = new Map();
  valid.forEach((component, index) => {
    const root = find(parent, index);
    const group = groups.get(root) || {
      weightedX: 0,
      weightedY: 0,
      area: 0,
      x1: Infinity,
      y1: Infinity,
      x2: -Infinity,
      y2: -Infinity,
    };
    group.weightedX += component.centerX * component.area;
    group.weightedY += component.centerY * component.area;
    group.area += component.area;
    group.x1 = Math.min(group.x1, component.x1);
    group.y1 = Math.min(group.y1, component.y1);
    group.x2 = Math.max(group.x2, component.x2);
    group.y2 = Math.max(group.y2, component.y2);
    groups.set(root, group);
  });

  return [...groups.values()]
    .map((group) => {
      const centerX = group.weightedX / group.area;
      const centerY = group.weightedY / group.area;
      return {
        ...group,
        centerX,
        centerY,
        estimatedBallCount: Math.max(1, Math.round(group.area / singleBallArea)),
        steeringError: (centerX - frameWidth / 2) / (frameWidth / 2),
      };
    })
    .sort((a, b) => b.estimatedBallCount - a.estimatedBallCount);
}

function boxesClose(a, b, gap) {
  return a.x1 - gap < b.x2 && a.x2 + gap > b.x1 && a.y1 - gap < b.y2 && a.y2 + gap > b.y1;
}

function sortRuns(runs) {
  return [...runs].sort((a, b) => a.row - b.row || a.start - b.start);
}

function find(parent, index) {
  while (parent[index] !== index) {
    parent[index] = parent[parent[index]];
    index = parent[index];
  }
  return index;
}

function union(parent, rank, a, b) {
  const rootA = find(parent, a);
  const rootB = find(parent, b);
  if (rootA === rootB) return;
  if (rank[rootA] < rank[rootB]) parent[rootA] = rootB;
  else if (rank[rootA] > rank[rootB]) parent[rootB] = rootA;
  else {
    parent[rootB] = rootA;
    rank[rootA]++;
  }
}

function runsToMask(runs, width, height) {
  const mask = new Uint8Array(width * height);
  for (const run of runs) {
    const base = run.row * width;
    for (let x = run.start; x <= run.end && x < width; x++) {
      if (run.row >= 0 && run.row < height) mask[base + x] = 1;
    }
  }
  return mask;
}

function draw(frame, mask, clumps, width, height) {
  ctx.clearRect(0, 0, width, height);

  if (showMask.checked) {
    const maskImage = ctx.createImageData(width, height);
    for (let p = 0, i = 0; p < mask.length; p++, i += 4) {
      const on = mask[p] ? 255 : 0;
      maskImage.data[i] = on;
      maskImage.data[i + 1] = on;
      maskImage.data[i + 2] = on;
      maskImage.data[i + 3] = 180;
    }
    ctx.putImageData(maskImage, 0, 0);
  }

  ctx.strokeStyle = "rgba(255,255,255,0.8)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(width / 2, 0);
  ctx.lineTo(width / 2, height);
  ctx.stroke();

  clumps.forEach((clump, index) => {
    const primary = index === 0;
    const color = primary ? "#5ee27a" : "#ffc247";
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = primary ? 3 : 1.5;
    ctx.strokeRect(clump.x1, clump.y1, clump.x2 - clump.x1 + 1, clump.y2 - clump.y1 + 1);

    ctx.beginPath();
    ctx.arc(clump.centerX, clump.centerY, 6, 0, Math.PI * 2);
    ctx.fill();

    const label = `~${clump.estimatedBallCount} ${clump.estimatedBallCount === 1 ? "ball" : "balls"}`;
    ctx.font = "16px system-ui, sans-serif";
    const textWidth = ctx.measureText(label).width;
    const labelY = Math.max(22, clump.y1 - 8);
    ctx.fillStyle = "rgba(10,12,14,0.85)";
    ctx.fillRect(clump.x1 + 3, labelY - 18, textWidth + 9, 23);
    ctx.fillStyle = color;
    ctx.fillText(label, clump.x1 + 8, labelY);
  });

  if (clumps[0]) {
    const best = clumps[0];
    const y = height - 24;
    ctx.strokeStyle = "#5aa7ff";
    ctx.fillStyle = "#5aa7ff";
    ctx.lineWidth = 2;
    drawArrow(width / 2, y, best.centerX, y);
    ctx.font = "14px system-ui, sans-serif";
    ctx.fillText(`err=${best.steeringError.toFixed(3)}`, 8, height - 8);
  }
}

function drawArrow(x1, y1, x2, y2) {
  const angle = Math.atan2(y2 - y1, x2 - x1);
  const headLength = 12;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.lineTo(x2 - headLength * Math.cos(angle - Math.PI / 6), y2 - headLength * Math.sin(angle - Math.PI / 6));
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - headLength * Math.cos(angle + Math.PI / 6), y2 - headLength * Math.sin(angle + Math.PI / 6));
  ctx.stroke();
}

function updateMetrics(clumps, now) {
  const best = clumps[0];
  clumpCount.textContent = String(clumps.length);
  bestLabel.textContent = best ? `~${best.estimatedBallCount} / ${Math.round(best.area)}px` : "none";
  steeringLabel.textContent = best ? best.steeringError.toFixed(3) : "0.000";

  frames++;
  if (now - lastFpsTime >= 500) {
    fpsLabel.textContent = String(Math.round((frames * 1000) / (now - lastFpsTime)));
    frames = 0;
    lastFpsTime = now;
  }
}

function saveSnapshot() {
  const capture = document.createElement("canvas");
  capture.width = overlay.width;
  capture.height = overlay.height;
  const captureCtx = capture.getContext("2d");
  captureCtx.drawImage(video, 0, 0, capture.width, capture.height);
  captureCtx.drawImage(overlay, 0, 0);
  const link = document.createElement("a");
  link.download = `pollen-test-${new Date().toISOString().replace(/[:.]/g, "-")}.png`;
  link.href = capture.toDataURL("image/png");
  link.click();
}

startButton.addEventListener("click", () => {
  startCamera().catch((error) => {
    emptyState.style.display = "grid";
    emptyState.innerHTML = `<h1>Camera blocked</h1><p>${error.message}</p>`;
  });
});
snapshotButton.addEventListener("click", saveSnapshot);
mirrorVideo.addEventListener("change", () => {
  videoWrap.classList.toggle("mirrored", mirrorVideo.checked);
});

if (!navigator.mediaDevices?.getUserMedia) {
  emptyState.innerHTML = "<h1>Camera unavailable</h1><p>This browser does not support webcam access.</p>";
} else {
  listCameras().catch(() => {});
}
