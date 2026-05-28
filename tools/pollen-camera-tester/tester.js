const video = document.getElementById("video");
const overlay = document.getElementById("overlay");
const ctx = overlay.getContext("2d", { willReadFrequently: true });
const emptyState = document.getElementById("emptyState");
const cameraSelect = document.getElementById("cameraSelect");
const startButton = document.getElementById("startButton");
const snapshotButton = document.getElementById("snapshotButton");
const pipelineSelect = document.getElementById("pipelineSelect");
const showMask = document.getElementById("showMask");
const showGrid = document.getElementById("showGrid");
const mirrorVideo = document.getElementById("mirrorVideo");
const videoWrap = document.querySelector(".video-wrap");
const APP_VERSION = "20260521-4";

const controls = {
  minArea: bindRange("minArea"),
  singleBallArea: bindRange("singleBallArea"),
  mergeGap: bindRange("mergeGap"),
  roiTop: bindRange("roiTop"),
  minBrightness: bindRange("minBrightness"),
  minSaturation: bindRange("minSaturation"),
  yellowScore: bindRange("yellowScore"),
};

const TEAMCODE_GRID = 15;
const TEAMCODE_R_MIN = 150;
const TEAMCODE_G_MIN = 150;
const TEAMCODE_B_MAX = 50;
const MAX_AREA = 160000;
const MAX_ASPECT = 6;
const MAX_COMPONENTS = 4096;

const clumpCount = document.getElementById("clumpCount");
const bestLabel = document.getElementById("bestLabel");
const steeringLabel = document.getElementById("steeringLabel");
const fpsLabel = document.getElementById("fpsLabel");
const pipelineLabel = document.getElementById("pipelineLabel");
const maskLabel = document.getElementById("maskLabel");
const bestPixelLabel = document.getElementById("bestPixelLabel");
const sampleLabel = document.getElementById("sampleLabel");
const statusLabel = document.getElementById("statusLabel");

let stream = null;
let sourceCanvas = document.createElement("canvas");
let sourceCtx = sourceCanvas.getContext("2d", { willReadFrequently: true });
let animationId = 0;
let lastFpsTime = performance.now();
let frames = 0;
let lastFrame = null;
let lastResult = null;
let sampledPixel = null;

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
      width: { ideal: 960 },
      height: { ideal: 720 },
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
  const width = video.videoWidth || 960;
  const height = video.videoHeight || 720;
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
  lastFrame = frame;
  const result = pipelineSelect.value === "teamcode"
    ? runTeamCodePipeline(frame.data, width, height)
    : runImprovedPipeline(frame.data, width, height);

  lastResult = result;
  draw(frame, result, width, height);
  updateMetrics(result, now);
  animationId = requestAnimationFrame(processFrame);
}

function runImprovedPipeline(data, width, height) {
  const threshold = buildImprovedMask(data, width, height);
  const components = labelMaskComponents(threshold.mask, width, height);
  const clumps = buildClumps(components, width);
  return {
    name: "Ruckus Improved",
    type: "improved",
    mask: threshold.mask,
    clumps,
    grid: [],
    maskPixels: threshold.maskPixels,
    debug: threshold.debug,
  };
}

function runTeamCodePipeline(data, width, height) {
  const mask = new Uint8Array(width * height);
  const clumps = [];
  const cellW = width / TEAMCODE_GRID;
  const cellH = height / TEAMCODE_GRID;

  for (let row = 0; row < TEAMCODE_GRID; row++) {
    for (let col = 0; col < TEAMCODE_GRID; col++) {
      const x1 = Math.floor((width * col) / TEAMCODE_GRID);
      const y1 = Math.floor((height * row) / TEAMCODE_GRID);
      const x2 = Math.min(width, Math.floor((width * (col + 1)) / TEAMCODE_GRID));
      const y2 = Math.min(height, Math.floor((height * (row + 1)) / TEAMCODE_GRID));
      const mean = meanRgb(data, width, x1, y1, x2, y2);
      const detected = mean.r > TEAMCODE_R_MIN && mean.g > TEAMCODE_G_MIN && mean.b < TEAMCODE_B_MAX;

      if (!detected) continue;
      fillMaskRect(mask, width, x1, y1, x2, y2);
      const area = (x2 - x1) * (y2 - y1);
      const centerX = x1 + (x2 - x1) / 2;
      const centerY = y1 + (y2 - y1) / 2;
      clumps.push({
        x1,
        y1,
        x2: x2 - 1,
        y2: y2 - 1,
        centerX,
        centerY,
        area,
        estimatedBallCount: 1,
        steeringError: (centerX - width / 2) / (width / 2),
        label: `${col},${row}`,
        mean,
      });
    }
  }

  return {
    name: "TeamCode blobDetection",
    type: "teamcode",
    mask,
    clumps: mergeTeamCodeCells(clumps, width),
    grid: clumps,
    maskPixels: countMask(mask),
    debug: findBestYellowPixel(data, width, height),
    cellW,
    cellH,
  };
}

function buildImprovedMask(data, width, height) {
  const mask = new Uint8Array(width * height);
  const minBrightness = Number(controls.minBrightness.value);
  const minSaturation = Number(controls.minSaturation.value);
  const minYellowScore = Number(controls.yellowScore.value);
  const roiTop = Math.floor(height * Number(controls.roiTop.value) / 100);
  const best = { score: -Infinity, x: 0, y: 0, r: 0, g: 0, b: 0, h: 0, s: 0, v: 0 };

  for (let i = 0, p = 0; i < data.length; i += 4, p++) {
    const r = data[i];
    const g = data[i + 1];
    const b = data[i + 2];
    const [h, s, v] = rgbToOpenCvHsv(r, g, b);
    const yellowScore = computeYellowScore(r, g, b, s);
    if (yellowScore > best.score) {
      best.score = yellowScore;
      best.x = p % width;
      best.y = Math.floor(p / width);
      best.r = r;
      best.g = g;
      best.b = b;
      best.h = h;
      best.s = s;
      best.v = v;
    }

    const y = Math.floor(p / width);
    if (y < roiTop) continue;

    const yellowDominant = r >= 150 && g >= 115 && b <= 140 && Math.min(r, g) - b >= 45;
    const saturatedYellow = h >= 16 && h <= 31 && s >= minSaturation && v >= minBrightness;
    const rgbYellow = yellowDominant && yellowScore >= minYellowScore;

    if (saturatedYellow && rgbYellow) {
      mask[p] = 1;
    }
  }

  const cleaned = suppressTinyNoise(mask, width, height);
  const cleanedPixels = countMask(cleaned);
  const rawPixels = countMask(mask);

  return {
    mask: cleanedPixels > 0 || rawPixels === 0 ? cleaned : mask,
    maskPixels: rawPixels,
    debug: best,
  };
}

function labelMaskComponents(mask, width, height) {
  const visited = new Uint8Array(mask.length);
  const components = [];
  const queue = new Int32Array(mask.length);
  const minArea = Math.max(1, Math.floor(Number(controls.minArea.value) / 3));

  for (let start = 0; start < mask.length && components.length < MAX_COMPONENTS; start++) {
    if (!mask[start] || visited[start]) continue;

    let head = 0;
    let tail = 0;
    let area = 0;
    let sumX = 0;
    let sumY = 0;
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    visited[start] = 1;
    queue[tail++] = start;

    while (head < tail) {
      const p = queue[head++];
      const x = p % width;
      const y = Math.floor(p / width);
      area++;
      sumX += x;
      sumY += y;
      if (x < minX) minX = x;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (y > maxY) maxY = y;

      enqueueNeighbor(p - 1, x > 0);
      enqueueNeighbor(p + 1, x < width - 1);
      enqueueNeighbor(p - width, y > 0);
      enqueueNeighbor(p + width, y < height - 1);
    }

    if (area >= minArea) {
      components.push({ area, sumX, sumY, minX, minY, maxX, maxY });
    }
  }

  return components;

  function enqueueNeighbor(index, inBounds) {
    if (!inBounds || visited[index] || !mask[index]) return;
    visited[index] = 1;
    queue[tail++] = index;
  }
}

function computeYellowScore(r, g, b, saturation) {
  const yellow = Math.min(r, g) - b;
  const balanceBonus = Math.max(0, 80 - Math.abs(r - g)) * 0.12;
  const saturationBonus = saturation * 0.08;
  return Math.round(yellow + balanceBonus + saturationBonus);
}

function findBestYellowPixel(data, width, height) {
  const best = { score: -Infinity, x: 0, y: 0, r: 0, g: 0, b: 0, h: 0, s: 0, v: 0 };
  for (let i = 0, p = 0; i < data.length; i += 4, p++) {
    const r = data[i];
    const g = data[i + 1];
    const b = data[i + 2];
    const [h, s, v] = rgbToOpenCvHsv(r, g, b);
    const score = computeYellowScore(r, g, b, s);
    if (score > best.score) {
      best.score = score;
      best.x = p % width;
      best.y = Math.floor(p / width);
      best.r = r;
      best.g = g;
      best.b = b;
      best.h = h;
      best.s = s;
      best.v = v;
    }
  }
  return best;
}

function suppressTinyNoise(mask, width, height) {
  const cleaned = new Uint8Array(mask.length);
  for (let y = 1; y < height - 1; y++) {
    const row = y * width;
    for (let x = 1; x < width - 1; x++) {
      const p = row + x;
      if (!mask[p]) continue;
      const neighbors =
        mask[p - 1] + mask[p + 1] +
        mask[p - width] + mask[p + width] +
        mask[p - width - 1] + mask[p - width + 1] +
        mask[p + width - 1] + mask[p + width + 1];
      if (neighbors >= 2) cleaned[p] = 1;
    }
  }
  return cleaned;
}

function meanRgb(data, width, x1, y1, x2, y2) {
  let r = 0;
  let g = 0;
  let b = 0;
  let count = 0;

  for (let y = y1; y < y2; y++) {
    for (let x = x1; x < x2; x++) {
      const i = (y * width + x) * 4;
      r += data[i];
      g += data[i + 1];
      b += data[i + 2];
      count++;
    }
  }

  return {
    r: Math.round(r / count),
    g: Math.round(g / count),
    b: Math.round(b / count),
  };
}

function fillMaskRect(mask, width, x1, y1, x2, y2) {
  for (let y = y1; y < y2; y++) {
    const base = y * width;
    for (let x = x1; x < x2; x++) {
      mask[base + x] = 1;
    }
  }
}

function mergeTeamCodeCells(cells, frameWidth) {
  const gap = Number(controls.mergeGap.value);
  const parent = cells.map((_, index) => index);
  const rank = cells.map(() => 0);

  for (let i = 0; i < cells.length; i++) {
    for (let j = i + 1; j < cells.length; j++) {
      if (boxesClose(cells[i], cells[j], gap)) union(parent, rank, i, j);
    }
  }

  const groups = new Map();
  cells.forEach((cell, index) => {
    const root = find(parent, index);
    const group = groups.get(root) || {
      weightedX: 0,
      weightedY: 0,
      area: 0,
      x1: Infinity,
      y1: Infinity,
      x2: -Infinity,
      y2: -Infinity,
      cells: 0,
    };
    group.weightedX += cell.centerX * cell.area;
    group.weightedY += cell.centerY * cell.area;
    group.area += cell.area;
    group.x1 = Math.min(group.x1, cell.x1);
    group.y1 = Math.min(group.y1, cell.y1);
    group.x2 = Math.max(group.x2, cell.x2);
    group.y2 = Math.max(group.y2, cell.y2);
    group.cells++;
    groups.set(root, group);
  });

  return [...groups.values()].map((group) => {
    const centerX = group.weightedX / group.area;
    const centerY = group.weightedY / group.area;
    return {
      ...group,
      centerX,
      centerY,
      estimatedBallCount: group.cells,
      steeringError: (centerX - frameWidth / 2) / (frameWidth / 2),
    };
  }).sort((a, b) => b.area - a.area);
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

  return [...groups.values()].map((group) => {
    const centerX = group.weightedX / group.area;
    const centerY = group.weightedY / group.area;
    return {
      ...group,
      centerX,
      centerY,
      estimatedBallCount: Math.max(1, Math.round(group.area / singleBallArea)),
      steeringError: (centerX - frameWidth / 2) / (frameWidth / 2),
    };
  }).sort((a, b) => b.area - a.area);
}

function boxesClose(a, b, gap) {
  return a.x1 - gap <= b.x2 && a.x2 + gap >= b.x1 && a.y1 - gap <= b.y2 && a.y2 + gap >= b.y1;
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

function countMask(mask) {
  let count = 0;
  for (const value of mask) count += value ? 1 : 0;
  return count;
}

function draw(frame, result, width, height) {
  ctx.clearRect(0, 0, width, height);

  if (showMask.checked) {
    drawMask(result.mask, width, height);
  }

  drawCenterLine(width, height);
  if (result.type === "improved") {
    drawRoiLine(width, height);
  }
  if (showGrid.checked || result.type === "teamcode") {
    drawGrid(width, height, result.grid);
  }
  drawClumps(result.clumps, width, height, result.type);
}

function drawRoiLine(width, height) {
  const y = Math.floor(height * Number(controls.roiTop.value) / 100);
  ctx.strokeStyle = "rgba(90,167,255,0.95)";
  ctx.lineWidth = 2;
  ctx.setLineDash([10, 7]);
  ctx.beginPath();
  ctx.moveTo(0, y);
  ctx.lineTo(width, y);
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawMask(mask, width, height) {
  const maskImage = ctx.createImageData(width, height);
  for (let p = 0, i = 0; p < mask.length; p++, i += 4) {
    if (mask[p]) {
      maskImage.data[i] = 255;
      maskImage.data[i + 1] = 220;
      maskImage.data[i + 2] = 40;
      maskImage.data[i + 3] = 145;
    }
  }
  ctx.putImageData(maskImage, 0, 0);
}

function drawCenterLine(width, height) {
  ctx.strokeStyle = "rgba(255,255,255,0.72)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(width / 2, 0);
  ctx.lineTo(width / 2, height);
  ctx.stroke();
}

function drawGrid(width, height, detectedCells) {
  const detected = new Set(detectedCells.map((cell) => `${cell.x1}:${cell.y1}`));
  ctx.lineWidth = 1;
  for (let row = 0; row < TEAMCODE_GRID; row++) {
    for (let col = 0; col < TEAMCODE_GRID; col++) {
      const x = Math.floor((width * col) / TEAMCODE_GRID);
      const y = Math.floor((height * row) / TEAMCODE_GRID);
      const x2 = Math.floor((width * (col + 1)) / TEAMCODE_GRID);
      const y2 = Math.floor((height * (row + 1)) / TEAMCODE_GRID);
      ctx.strokeStyle = detected.has(`${x}:${y}`) ? "rgba(94,226,122,0.95)" : "rgba(255,255,255,0.18)";
      ctx.strokeRect(x, y, x2 - x, y2 - y);
    }
  }
}

function drawClumps(clumps, width, height, type) {
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

    const unit = type === "teamcode" ? "cells" : "balls";
    const count = clump.estimatedBallCount;
    const label = `${count} ${count === 1 ? unit.replace(/s$/, "") : unit}`;
    drawLabel(label, clump.x1, Math.max(22, clump.y1 - 8), color);
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

function drawLabel(label, x, y, color) {
  ctx.font = "16px system-ui, sans-serif";
  const textWidth = ctx.measureText(label).width;
  ctx.fillStyle = "rgba(10,12,14,0.86)";
  ctx.fillRect(x + 3, y - 18, textWidth + 9, 23);
  ctx.fillStyle = color;
  ctx.fillText(label, x + 8, y);
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

function updateMetrics(result, now) {
  const best = result.clumps[0];
  clumpCount.textContent = String(result.clumps.length);
  bestLabel.textContent = best ? `${best.estimatedBallCount} / ${Math.round(best.area)}px` : "none";
  steeringLabel.textContent = best ? best.steeringError.toFixed(3) : "0.000";
  pipelineLabel.textContent = result.name;
  maskLabel.textContent = `${result.maskPixels} px`;
  bestPixelLabel.textContent = result.debug ? formatPixelDebug(result.debug) : "none";
  sampleLabel.textContent = sampledPixel ? formatPixelDebug(sampledPixel) : "click video";
  statusLabel.textContent = buildStatus(result);

  frames++;
  if (now - lastFpsTime >= 500) {
    fpsLabel.textContent = String(Math.round((frames * 1000) / (now - lastFpsTime)));
    frames = 0;
    lastFpsTime = now;
  }
}

function buildStatus(result) {
  if (result.clumps.length > 0) {
    return `${result.name} active. Version ${APP_VERSION}.`;
  }
  if (result.maskPixels > 0) {
    return `${result.name} sees yellow-colored pixels, but none passed the area/shape filters. Lower Minimum Area or enable Show threshold mask. Version ${APP_VERSION}.`;
  }
  if (result.debug && Number.isFinite(result.debug.score)) {
    return `${result.name} sees no pixels passing the threshold. Best yellow score is ${result.debug.score}; lower Yellow Score, Brightness, or Saturation if the target is visible. Version ${APP_VERSION}.`;
  }
  return `${result.name} active. Version ${APP_VERSION}.`;
}

function formatPixelDebug(pixel) {
  return `rgb ${pixel.r},${pixel.g},${pixel.b} hsv ${pixel.h},${pixel.s},${pixel.v} score ${pixel.score}`;
}

function sampleAtClientPoint(event) {
  if (!lastFrame) return;
  const rect = videoWrap.getBoundingClientRect();
  let x = Math.round(((event.clientX - rect.left) / rect.width) * lastFrame.width);
  const y = Math.round(((event.clientY - rect.top) / rect.height) * lastFrame.height);
  if (mirrorVideo.checked) x = lastFrame.width - x;
  x = Math.max(0, Math.min(lastFrame.width - 1, x));
  const yy = Math.max(0, Math.min(lastFrame.height - 1, y));
  const i = (yy * lastFrame.width + x) * 4;
  const r = lastFrame.data[i];
  const g = lastFrame.data[i + 1];
  const b = lastFrame.data[i + 2];
  const [h, s, v] = rgbToOpenCvHsv(r, g, b);
  sampledPixel = {
    x,
    y: yy,
    r,
    g,
    b,
    h,
    s,
    v,
    score: computeYellowScore(r, g, b, s),
  };
  sampleLabel.textContent = formatPixelDebug(sampledPixel);
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
pipelineSelect.addEventListener("change", () => {
  const activeName = pipelineSelect.value === "teamcode" ? "TeamCode blobDetection" : "Ruckus Improved";
  pipelineLabel.textContent = activeName;
  statusLabel.textContent = `${activeName} selected. Version ${APP_VERSION}.`;
  if (lastFrame) {
    const result = pipelineSelect.value === "teamcode"
      ? runTeamCodePipeline(lastFrame.data, lastFrame.width, lastFrame.height)
      : runImprovedPipeline(lastFrame.data, lastFrame.width, lastFrame.height);
    lastResult = result;
    draw(lastFrame, result, lastFrame.width, lastFrame.height);
    updateMetrics(result, performance.now());
  }
});
mirrorVideo.addEventListener("change", () => {
  videoWrap.classList.toggle("mirrored", mirrorVideo.checked);
});
videoWrap.addEventListener("click", sampleAtClientPoint);
statusLabel.textContent = `Loaded Pollen Camera Tester ${APP_VERSION}.`;
window.__pollenCameraTesterVersion = APP_VERSION;

if (!navigator.mediaDevices?.getUserMedia) {
  emptyState.innerHTML = "<h1>Camera unavailable</h1><p>This browser does not support webcam access.</p>";
} else {
  listCameras().catch(() => {});
}
