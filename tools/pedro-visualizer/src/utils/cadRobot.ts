/**
 * Turns a robot CAD export into the top-down picture the field draws.
 *
 * Teams already have an accurate model of the robot; drawing the real footprint
 * beats a stock icon for judging clearances by eye. STL and OBJ cover what
 * Onshape, Fusion and SolidWorks export without asking anyone to install
 * anything, and both are simple enough to read here rather than pulling in a
 * mesh library.
 *
 * A robot export runs to hundreds of thousands of triangles, so the work is
 * split in two: everything that does not depend on which way the model is
 * facing is done once at upload behind a progress bar, and the redraw that runs
 * while someone drags the rotation slider is a scanline rasteriser whose cost
 * is bounded by the pixels it fills rather than by the size of the mesh.
 */

import { convexHull } from "./geometry";

export type UpAxis = "x" | "y" | "z";
export type CadUnit = "mm" | "cm" | "in";

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface Mesh {
  /** Flat triangle list: every three vertices make one face. */
  vertices: Float32Array;
  triangleCount: number;
}

/** A mesh with everything orientation-independent already worked out. */
export interface PreparedMesh {
  /** Vertices moved so the footprint centre sits at the origin. */
  vertices: Float32Array;
  /** Unit normal per triangle, three floats each. */
  normals: Float32Array;
  triangleCount: number;
}

export interface Orientation {
  /** Model axis pointing at the ceiling. */
  upAxis: UpAxis;
  /** +1 when that axis points up, -1 when the model is stored upside down. */
  upSign: 1 | -1;
  /** Spin in the top-down plane, degrees, to point the robot's forward right. */
  rotationDegrees: number;
}

export interface Projection {
  /** Screen x per vertex, three per triangle. Canvas orientation, unscaled. */
  screenX: Float32Array;
  screenY: Float32Array;
  /** Height up the view axis per vertex. Larger is nearer the camera. */
  depth: Float32Array;
  /** 0..1 per triangle: how squarely the face points at the camera. */
  shade: Float32Array;
  triangleCount: number;
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

const UNIT_TO_INCHES: Record<CadUnit, number> = {
  mm: 1 / 25.4,
  cm: 1 / 2.54,
  in: 1,
};

export function unitToInches(unit: CadUnit): number {
  return UNIT_TO_INCHES[unit] ?? 1;
}

/* -------------------------------------------------------------------------
 * Parsing
 * ---------------------------------------------------------------------- */

function isBinaryStl(data: ArrayBuffer): boolean {
  if (data.byteLength < 84) return false;

  // A binary STL states its triangle count, and the file is exactly the header
  // plus 50 bytes per triangle. ASCII files almost never match that by chance.
  const view = new DataView(data);
  const declared = view.getUint32(80, true);
  if (84 + declared * 50 === data.byteLength) return true;

  // Otherwise fall back to sniffing for the ASCII keyword.
  const head = new TextDecoder().decode(
    new Uint8Array(data, 0, Math.min(512, data.byteLength)),
  );
  return !/^\s*solid/i.test(head) || !/facet/i.test(head);
}

function parseBinaryStl(data: ArrayBuffer): Mesh {
  const view = new DataView(data);
  const count = view.getUint32(80, true);
  const usable = Math.min(count, Math.max(0, Math.floor((data.byteLength - 84) / 50)));
  const vertices = new Float32Array(usable * 9);

  for (let i = 0; i < usable; i++) {
    // 12 bytes of normal, then three vertices, then a 2-byte attribute count.
    const base = 84 + i * 50 + 12;
    for (let v = 0; v < 9; v++) {
      vertices[i * 9 + v] = view.getFloat32(base + v * 4, true);
    }
  }

  return { vertices, triangleCount: usable };
}

function parseAsciiStl(text: string): Mesh {
  const values: number[] = [];
  const vertexPattern = /vertex\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)/g;
  let match: RegExpExecArray | null;

  while ((match = vertexPattern.exec(text)) !== null) {
    values.push(Number(match[1]), Number(match[2]), Number(match[3]));
  }

  const triangleCount = Math.floor(values.length / 9);
  return {
    vertices: Float32Array.from(values.slice(0, triangleCount * 9)),
    triangleCount,
  };
}

function parseObj(text: string): Mesh {
  const points: number[] = [];
  const values: number[] = [];

  const pushVertex = (index: number) => {
    // OBJ is 1-based, and negative indices count back from the end.
    const resolved = index < 0 ? points.length / 3 + index : index - 1;
    const base = resolved * 3;
    if (base < 0 || base + 2 >= points.length + 3) return false;
    values.push(points[base], points[base + 1], points[base + 2]);
    return true;
  };

  for (const rawLine of text.split(/\r?\n/)) {
    const line = rawLine.trim();
    if (!line || line.startsWith("#")) continue;

    if (line.startsWith("v ")) {
      const parts = line.split(/\s+/);
      points.push(Number(parts[1]), Number(parts[2]), Number(parts[3]));
      continue;
    }

    if (!line.startsWith("f ")) continue;

    // A face may be a polygon and each corner may be v, v/vt or v/vt/vn.
    const corners = line
      .split(/\s+/)
      .slice(1)
      .map((corner) => Number.parseInt(corner.split("/")[0], 10))
      .filter((index) => Number.isFinite(index));

    // Fan-triangulate, which is right for the convex faces CAD exports.
    for (let i = 1; i + 1 < corners.length; i++) {
      const before = values.length;
      const ok =
        pushVertex(corners[0]) && pushVertex(corners[i]) && pushVertex(corners[i + 1]);
      if (!ok) values.length = before;
    }
  }

  const triangleCount = Math.floor(values.length / 9);
  return {
    vertices: Float32Array.from(values.slice(0, triangleCount * 9)),
    triangleCount,
  };
}

/** Reads an STL or OBJ export. Throws with a readable reason when it cannot. */
export function parseMesh(fileName: string, data: ArrayBuffer): Mesh {
  const extension = fileName.toLowerCase().split(".").pop() || "";

  if (extension === "obj") {
    const mesh = parseObj(new TextDecoder().decode(data));
    if (mesh.triangleCount === 0) throw new Error("No faces found in this OBJ.");
    return mesh;
  }

  if (extension === "stl") {
    const mesh = isBinaryStl(data)
      ? parseBinaryStl(data)
      : parseAsciiStl(new TextDecoder().decode(data));
    if (mesh.triangleCount === 0) throw new Error("No triangles found in this STL.");
    return mesh;
  }

  throw new Error(`Unsupported file type ".${extension}". Export an STL or OBJ.`);
}

/* -------------------------------------------------------------------------
 * Preparation — done once per upload
 * ---------------------------------------------------------------------- */

/**
 * Centres the mesh and works out every face normal.
 *
 * Neither depends on which way the model ends up facing, so doing it here means
 * a rotation only has to re-project, not re-derive the geometry.
 */
export function prepareMesh(mesh: Mesh): PreparedMesh {
  const count = mesh.triangleCount;
  const vertices = new Float32Array(mesh.vertices.subarray(0, count * 9));
  const normals = new Float32Array(count * 3);

  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;

  for (let i = 0; i < vertices.length; i += 3) {
    const x = vertices[i];
    const y = vertices[i + 1];
    const z = vertices[i + 2];
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
  }

  if (Number.isFinite(minX)) {
    const cx = (minX + maxX) / 2;
    const cy = (minY + maxY) / 2;
    const cz = (minZ + maxZ) / 2;
    for (let i = 0; i < vertices.length; i += 3) {
      vertices[i] -= cx;
      vertices[i + 1] -= cy;
      vertices[i + 2] -= cz;
    }
  }

  for (let t = 0; t < count; t++) {
    const base = t * 9;
    const ax = vertices[base + 3] - vertices[base];
    const ay = vertices[base + 4] - vertices[base + 1];
    const az = vertices[base + 5] - vertices[base + 2];
    const bx = vertices[base + 6] - vertices[base];
    const by = vertices[base + 7] - vertices[base + 1];
    const bz = vertices[base + 8] - vertices[base + 2];

    const nx = ay * bz - az * by;
    const ny = az * bx - ax * bz;
    const nz = ax * by - ay * bx;
    const length = Math.hypot(nx, ny, nz);

    if (length > 0) {
      normals[t * 3] = nx / length;
      normals[t * 3 + 1] = ny / length;
      normals[t * 3 + 2] = nz / length;
    }
  }

  return { vertices, normals, triangleCount: count };
}

export type ProgressReporter = (fraction: number, label: string) => void;

const yieldToUi = () => new Promise<void>((resolve) => setTimeout(resolve, 0));

/**
 * Reads and prepares a CAD file, reporting progress as it goes.
 *
 * Everything expensive happens here, once, so that changing the up axis or
 * dragging the rotation afterwards only re-projects an already-prepared mesh.
 */
export async function loadPreparedMesh(
  file: File,
  onProgress: ProgressReporter = () => {},
): Promise<{ mesh: PreparedMesh; triangleCount: number }> {
  onProgress(0.05, "Reading file…");
  const data = await file.arrayBuffer();

  onProgress(0.35, "Parsing mesh…");
  await yieldToUi();
  const parsed = parseMesh(file.name, data);

  onProgress(0.75, `Preparing ${parsed.triangleCount.toLocaleString()} triangles…`);
  await yieldToUi();
  const prepared = prepareMesh(parsed);

  onProgress(1, "Ready");
  return { mesh: prepared, triangleCount: prepared.triangleCount };
}

/* -------------------------------------------------------------------------
 * Projection
 * ---------------------------------------------------------------------- */

/**
 * Which model axes end up pointing right and up on screen when looking straight
 * down the given axis.
 *
 * Chosen so the view is never mirrored: right × up has to equal the axis being
 * looked down, otherwise the robot comes out as its own reflection and every
 * asymmetric feature ends up on the wrong side.
 */
function viewBasis(upAxis: UpAxis, upSign: 1 | -1): { right: Vec3; up: Vec3; view: Vec3 } {
  const axis: Record<UpAxis, Vec3> = {
    x: { x: 1, y: 0, z: 0 },
    y: { x: 0, y: 1, z: 0 },
    z: { x: 0, y: 0, z: 1 },
  };
  const scale = (v: Vec3, k: number): Vec3 => ({ x: v.x * k, y: v.y * k, z: v.z * k });

  const view = scale(axis[upAxis], upSign);

  if (upAxis === "z") {
    return { right: axis.x, up: scale(axis.y, upSign), view };
  }
  if (upAxis === "y") {
    return { right: axis.z, up: scale(axis.x, upSign), view };
  }
  return { right: axis.y, up: scale(axis.z, upSign), view };
}

/**
 * Flattens the mesh into screen-space triangles in model units.
 *
 * Writes into flat typed arrays and allocates nothing per triangle, because
 * this runs on every frame of a rotation drag.
 */
export function projectMesh(
  mesh: PreparedMesh,
  orientation: Orientation,
  reuse?: Projection,
): Projection {
  const { right, up, view } = viewBasis(orientation.upAxis, orientation.upSign);
  const angle = ((Number(orientation.rotationDegrees) || 0) * Math.PI) / 180;
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);

  const count = mesh.triangleCount;
  // Reuse the caller's buffers when they fit: a rotation drag would otherwise
  // throw away several megabytes of typed arrays every frame.
  const fits = reuse && reuse.triangleCount === count;
  const screenX = fits ? reuse!.screenX : new Float32Array(count * 3);
  const screenY = fits ? reuse!.screenY : new Float32Array(count * 3);
  const depth = fits ? reuse!.depth : new Float32Array(count * 3);
  const shade = fits ? reuse!.shade : new Float32Array(count);

  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;

  for (let t = 0; t < count; t++) {
    const vertexBase = t * 9;
    const outBase = t * 3;

    for (let v = 0; v < 3; v++) {
      const i = vertexBase + v * 3;
      const x = mesh.vertices[i];
      const y = mesh.vertices[i + 1];
      const z = mesh.vertices[i + 2];

      const rx = x * right.x + y * right.y + z * right.z;
      const ry = x * up.x + y * up.y + z * up.z;

      // Spin in the plane, then flip Y because canvas Y grows downward.
      const sx = rx * cos - ry * sin;
      const sy = -(rx * sin + ry * cos);

      screenX[outBase + v] = sx;
      screenY[outBase + v] = sy;
      depth[outBase + v] = x * view.x + y * view.y + z * view.z;

      if (sx < minX) minX = sx;
      if (sx > maxX) maxX = sx;
      if (sy < minY) minY = sy;
      if (sy > maxY) maxY = sy;
    }

    const nx = mesh.normals[outBase];
    const ny = mesh.normals[outBase + 1];
    const nz = mesh.normals[outBase + 2];
    shade[t] = Math.abs(nx * view.x + ny * view.y + nz * view.z);
  }

  if (!Number.isFinite(minX)) {
    minX = 0;
    maxX = 0;
    minY = 0;
    maxY = 0;
  }

  return { screenX, screenY, depth, shade, triangleCount: count, minX, maxX, minY, maxY };
}

/* -------------------------------------------------------------------------
 * Rasterising
 * ---------------------------------------------------------------------- */

export interface RobotImageOptions extends Orientation {
  unit: CadUnit;
  /** Output image size in pixels, square. */
  size?: number;
  color?: string;
  /** Extra resolution rasterised then scaled down, for smoother edges. */
  supersample?: number;
}

export interface RobotImageResult {
  dataUrl: string;
  /** Footprint along the robot's forward direction, in inches. */
  lengthInches: number;
  /** Footprint across the robot, in inches. */
  widthInches: number;
  triangleCount: number;
}

/**
 * The robot's actual footprint, in its own frame with `+x` forward, centred the
 * same way the picture is. Kept alongside the size it was measured at so it can
 * be rescaled if the robot's dimensions are edited afterwards.
 */
export interface RobotOutline {
  points: Array<{ x: number; y: number }>;
  lengthInches: number;
  widthInches: number;
}

/**
 * The convex outline of the top-down footprint, in inches.
 *
 * A bounding rectangle is the wrong shape to check clearances with: a robot with
 * a corner intake sweeps a hexagon, and the rectangle claims material where
 * there is only air, so every tight gap reads as a collision. The hull is what
 * the robot actually presents to an obstacle.
 *
 * A real export has over a million projected vertices, far too many to hand
 * straight to a Graham scan. Reducing first to the extreme points of each thin
 * column and row leaves a few thousand candidates in one pass, and cannot shrink
 * the shape: what it keeps is the outermost point in each strip, so every
 * discarded point lies inside what remains up to the strip width, and the hull
 * is grown by that much to stay on the safe side.
 *
 * Called once when the picture is applied rather than inside `renderRobotImage`,
 * because that runs on every frame of a rotation drag and this is a full pass
 * over the mesh.
 */
export function footprintHull(
  projection: Projection,
  unit: CadUnit,
  strips: number = 1024,
): RobotOutline {
  const perInch = unitToInches(unit);
  const spanX = projection.maxX - projection.minX;
  const spanY = projection.maxY - projection.minY;
  const lengthInches = spanX * perInch;
  const widthInches = spanY * perInch;

  const vertexCount = projection.triangleCount * 3;
  if (vertexCount < 3 || spanX <= 0 || spanY <= 0) {
    return { points: [], lengthInches, widthInches };
  }

  const stripCount = Math.max(16, Math.min(4096, Math.round(strips)));
  const columnLow = new Float32Array(stripCount).fill(Infinity);
  const columnHigh = new Float32Array(stripCount).fill(-Infinity);
  const rowLow = new Float32Array(stripCount).fill(Infinity);
  const rowHigh = new Float32Array(stripCount).fill(-Infinity);

  const xScale = (stripCount - 1) / spanX;
  const yScale = (stripCount - 1) / spanY;

  for (let i = 0; i < vertexCount; i++) {
    const x = projection.screenX[i];
    const y = projection.screenY[i];

    const column = ((x - projection.minX) * xScale) | 0;
    if (y < columnLow[column]) columnLow[column] = y;
    if (y > columnHigh[column]) columnHigh[column] = y;

    const row = ((y - projection.minY) * yScale) | 0;
    if (x < rowLow[row]) rowLow[row] = x;
    if (x > rowHigh[row]) rowHigh[row] = x;
  }

  const candidates: Array<{ x: number; y: number }> = [];
  for (let i = 0; i < stripCount; i++) {
    if (columnLow[i] !== Infinity) {
      const x = projection.minX + i / xScale;
      candidates.push({ x, y: columnLow[i] }, { x, y: columnHigh[i] });
    }
    if (rowLow[i] !== Infinity) {
      const y = projection.minY + i / yScale;
      candidates.push({ x: rowLow[i], y }, { x: rowHigh[i], y });
    }
  }

  const hull = convexHull(candidates);
  if (hull.length < 3) return { points: [], lengthInches, widthInches };

  // Centre on the footprint's box, which is what the picture is centred on, so
  // the outline and the drawn robot sit on top of each other.
  const centreX = (projection.minX + projection.maxX) / 2;
  const centreY = (projection.minY + projection.maxY) / 2;

  // Give back the strip width the reduction could have shaved off.
  const growX = spanX / (stripCount - 1);
  const growY = spanY / (stripCount - 1);

  return {
    points: hull.map((point) => {
      const dx = point.x - centreX;
      const dy = point.y - centreY;
      const lengthOut = Math.hypot(dx, dy);
      const grow = lengthOut > 0 ? Math.hypot(growX, growY) / lengthOut : 0;
      return {
        x: dx * (1 + grow) * perInch,
        y: dy * (1 + grow) * perInch,
      };
    }),
    lengthInches,
    widthInches,
  };
}

function parseColor(hex: string): [number, number, number] {
  const match = /^#?([\da-f]{6})$/i.exec((hex || "").trim());
  const value = match ? parseInt(match[1], 16) : 0x4f8ef7;
  return [(value >> 16) & 0xff, (value >> 8) & 0xff, value & 0xff];
}

/**
 * Paints the projection with a depth buffer.
 *
 * A painter's-algorithm pass would have to sort every face and then make a
 * canvas call per triangle, which is what made dragging the rotation crawl on a
 * real export. Rasterising into a pixel buffer instead needs no sort at all,
 * costs what the covered pixels cost rather than what the mesh weighs, and
 * resolves overlapping decks exactly instead of approximately.
 */
export interface RasterScratch {
  size: number;
  pixels: Uint8ClampedArray;
  zBuffer: Float32Array;
}

export function rasterizeProjection(
  projection: Projection,
  size: number,
  color: string,
  scratch?: RasterScratch,
): ImageData {
  const reusable = scratch && scratch.size === size;
  const pixels = reusable ? scratch!.pixels : new Uint8ClampedArray(size * size * 4);
  const zBuffer = reusable ? scratch!.zBuffer : new Float32Array(size * size);
  if (reusable) pixels.fill(0);
  zBuffer.fill(-Infinity);
  const [baseR, baseG, baseB] = parseColor(color);

  const spanX = projection.maxX - projection.minX;
  const spanY = projection.maxY - projection.minY;
  if (spanX <= 0 || spanY <= 0) return new ImageData(pixels, size, size);

  // Fit the footprint to the square without distorting it.
  const scale = Math.min(size / spanX, size / spanY);
  const offsetX = (size - spanX * scale) / 2;
  const offsetY = (size - spanY * scale) / 2;

  for (let t = 0; t < projection.triangleCount; t++) {
    const i = t * 3;
    const x0 = (projection.screenX[i] - projection.minX) * scale + offsetX;
    const y0 = (projection.screenY[i] - projection.minY) * scale + offsetY;
    const x1 = (projection.screenX[i + 1] - projection.minX) * scale + offsetX;
    const y1 = (projection.screenY[i + 1] - projection.minY) * scale + offsetY;
    const x2 = (projection.screenX[i + 2] - projection.minX) * scale + offsetX;
    const y2 = (projection.screenY[i + 2] - projection.minY) * scale + offsetY;

    const area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    if (area === 0) continue;
    // Accept either winding: normalise the edge tests by the signed area.
    const inverseArea = 1 / area;

    let left = Math.max(0, Math.floor(Math.min(x0, x1, x2)));
    let right = Math.min(size - 1, Math.ceil(Math.max(x0, x1, x2)));
    let top = Math.max(0, Math.floor(Math.min(y0, y1, y2)));
    let bottom = Math.min(size - 1, Math.ceil(Math.max(y0, y1, y2)));
    if (left > right || top > bottom) continue;

    const z0 = projection.depth[i];
    const z1 = projection.depth[i + 1];
    const z2 = projection.depth[i + 2];

    const brightness = 0.35 + 0.65 * projection.shade[t];
    const r = Math.min(255, baseR * brightness);
    const g = Math.min(255, baseG * brightness);
    const b = Math.min(255, baseB * brightness);

    for (let py = top; py <= bottom; py++) {
      const sampleY = py + 0.5;
      for (let px = left; px <= right; px++) {
        const sampleX = px + 0.5;

        // Barycentric coordinates via edge functions.
        const w0 =
          ((x1 - sampleX) * (y2 - sampleY) - (y1 - sampleY) * (x2 - sampleX)) *
          inverseArea;
        if (w0 < 0) continue;
        const w1 =
          ((x2 - sampleX) * (y0 - sampleY) - (y2 - sampleY) * (x0 - sampleX)) *
          inverseArea;
        if (w1 < 0) continue;
        const w2 = 1 - w0 - w1;
        if (w2 < 0) continue;

        const z = w0 * z0 + w1 * z1 + w2 * z2;
        const index = py * size + px;
        if (z <= zBuffer[index]) continue;

        zBuffer[index] = z;
        const p = index * 4;
        pixels[p] = r;
        pixels[p + 1] = g;
        pixels[p + 2] = b;
        pixels[p + 3] = 255;
      }
    }
  }

  return new ImageData(pixels, size, size);
}

/**
 * Renders the prepared mesh to a transparent square PNG, sized so the footprint
 * fills it exactly. The field scales the image to the robot's dimensions, so
 * any padding here would read as a robot larger than it is.
 */
export interface RenderScratch {
  projection?: Projection;
  raster?: RasterScratch;
}

export function renderRobotImage(
  mesh: PreparedMesh,
  options: RobotImageOptions,
  createCanvas: (width: number, height: number) => HTMLCanvasElement,
  scratch?: RenderScratch,
): RobotImageResult {
  const size = Math.max(64, Math.round(options.size ?? 512));
  const supersample = Math.max(1, Math.min(3, Math.round(options.supersample ?? 2)));
  const renderSize = size * supersample;

  const projection = projectMesh(mesh, options, scratch?.projection);
  if (scratch) scratch.projection = projection;

  if (scratch && scratch.raster?.size !== renderSize) {
    scratch.raster = {
      size: renderSize,
      pixels: new Uint8ClampedArray(renderSize * renderSize * 4),
      zBuffer: new Float32Array(renderSize * renderSize),
    };
  }
  const image = rasterizeProjection(
    projection,
    renderSize,
    options.color || "#4f8ef7",
    scratch?.raster,
  );

  // Rasterise large, then let the canvas scale it down: the averaging is what
  // gives the edges their smoothness, since the depth buffer itself is hard.
  const source = createCanvas(renderSize, renderSize);
  source.getContext("2d")?.putImageData(image, 0, 0);

  const output = createCanvas(size, size);
  const context = output.getContext("2d");
  if (context) {
    context.clearRect(0, 0, size, size);
    context.imageSmoothingEnabled = true;
    context.imageSmoothingQuality = "high";
    context.drawImage(source, 0, 0, size, size);
  }

  const perInch = unitToInches(options.unit);
  return {
    dataUrl: output.toDataURL("image/png"),
    lengthInches: (projection.maxX - projection.minX) * perInch,
    widthInches: (projection.maxY - projection.minY) * perInch,
    triangleCount: mesh.triangleCount,
  };
}
