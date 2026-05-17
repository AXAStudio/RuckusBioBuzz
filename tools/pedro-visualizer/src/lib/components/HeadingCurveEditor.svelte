<script lang="ts">
  import { createEventDispatcher } from "svelte";
  import { clampHeadingCurve, shapeHeadingProgress } from "../../utils/math";
  import type { Point } from "../../types";

  export let endPoint: Point;
  export let locked = false;

  const dispatch = createEventDispatcher();
  const width = 220;
  const height = 88;
  const padding = 12;
  const plotWidth = width - padding * 2;
  const plotHeight = height - padding * 2;
  const samples = 32;

  $: curve = clampHeadingCurve(
    endPoint.heading === "linear" ? endPoint.headingCurve : 1,
  );
  $: curvePoints = Array.from({ length: samples + 1 }, (_, index) => {
    const t = index / samples;
    const progress = shapeHeadingProgress(t, curve);
    return `${xFor(t)},${yFor(progress)}`;
  }).join(" ");
  $: diagonalPoints = `${xFor(0)},${yFor(0)} ${xFor(1)},${yFor(1)}`;
  $: handleX = xFor(0.5);
  $: handleY = yFor(shapeHeadingProgress(0.5, curve));
  $: curveLabel =
    curve > 1.05 ? "Turns later" : curve < 0.95 ? "Turns earlier" : "Even";

  function xFor(t: number): number {
    return padding + Math.max(0, Math.min(1, t)) * plotWidth;
  }

  function yFor(progress: number): number {
    return padding + (1 - Math.max(0, Math.min(1, progress))) * plotHeight;
  }

  function updateCurve(nextCurve: number) {
    if (endPoint.heading !== "linear") return;
    endPoint.headingCurve = clampHeadingCurve(nextCurve);
    dispatch("change");
  }

  function handleCurveInput(event: Event) {
    const target = event.currentTarget as HTMLInputElement;
    if (target.value.trim() === "") return;
    const nextCurve = Number(target.value);
    if (!Number.isFinite(nextCurve)) return;
    updateCurve(nextCurve);
  }

  function commit() {
    updateCurve(curve);
    dispatch("commit");
  }

  function resetCurve() {
    updateCurve(1);
    dispatch("commit");
  }
</script>

<div class="w-full max-w-md rounded-md border border-neutral-200 dark:border-neutral-700 bg-neutral-50 dark:bg-neutral-900 p-2">
  <div class="flex items-center justify-between gap-2 mb-2">
    <div class="flex items-center gap-2">
      <span class="text-xs font-semibold text-neutral-600 dark:text-neutral-300">Heading Curve</span>
      <span class="text-[11px] text-neutral-500 dark:text-neutral-400">{curveLabel}</span>
    </div>
    <button
      type="button"
      on:click={resetCurve}
      disabled={locked}
      class="px-2 py-0.5 text-xs rounded bg-neutral-200 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200 disabled:opacity-40"
    >
      Reset
    </button>
  </div>

  <svg
    viewBox={`0 0 ${width} ${height}`}
    class="w-full h-24 rounded bg-white dark:bg-neutral-950 border border-neutral-200 dark:border-neutral-800"
    aria-hidden="true"
  >
    <line
      x1={padding}
      y1={height - padding}
      x2={width - padding}
      y2={height - padding}
      stroke="currentColor"
      class="text-neutral-300 dark:text-neutral-700"
      stroke-width="1"
    />
    <line
      x1={padding}
      y1={padding}
      x2={padding}
      y2={height - padding}
      stroke="currentColor"
      class="text-neutral-300 dark:text-neutral-700"
      stroke-width="1"
    />
    <polyline
      points={diagonalPoints}
      fill="none"
      stroke="currentColor"
      class="text-neutral-300 dark:text-neutral-700"
      stroke-width="1"
      stroke-dasharray="4 4"
    />
    <polyline
      points={curvePoints}
      fill="none"
      stroke="#0ea5e9"
      stroke-width="3"
      stroke-linecap="round"
      stroke-linejoin="round"
    />
    <circle
      cx={handleX}
      cy={handleY}
      r="6"
      fill="#0ea5e9"
      stroke="white"
      stroke-width="2"
    />
    <text x={padding} y={height - 4} class="fill-neutral-500 dark:fill-neutral-400 text-[9px]">Start</text>
    <text x={width - padding - 22} y={height - 4} class="fill-neutral-500 dark:fill-neutral-400 text-[9px]">End</text>
  </svg>

  <div class="flex items-center gap-2 mt-2">
    <input
      type="range"
      min="0.25"
      max="4"
      step="0.05"
      value={curve}
      on:input={handleCurveInput}
      on:change={commit}
      disabled={locked}
      class="flex-1"
    />
    <input
      type="number"
      min="0.25"
      max="4"
      step="0.05"
      value={curve}
      on:input={handleCurveInput}
      on:blur={commit}
      disabled={locked}
      class="w-20 px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-white dark:bg-neutral-950"
    />
  </div>
</div>
