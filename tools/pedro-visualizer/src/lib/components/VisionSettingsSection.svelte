<script lang="ts">
  /**
   * Settings → Vision: the robot's camera and how PollenDetectionPipeline is
   * tuned. Every Pollen Pickup reads these, and the TeamCode export writes them
   * into the generated auto.
   *
   * Edits replace `settings.vision` rather than mutating it: the defaults object
   * is shared, and mutating it would quietly change every project at once.
   */
  import type { PollenPipelineSettings, Settings, VisionSettings } from "../../types";
  import { DEFAULT_POLLEN_PIPELINE } from "../../config/defaults";
  import { reliableRangeInches, visionOf } from "../../utils/pollenVision";

  export let settings: Settings;

  $: vision = visionOf(settings);
  $: reach = reliableRangeInches(vision);

  function patch(next: Partial<VisionSettings>) {
    settings = { ...settings, vision: visionOf({ vision: { ...vision, ...next } }) };
  }

  function patchPipeline(next: Partial<PollenPipelineSettings>) {
    patch({ pipeline: { ...vision.pipeline, ...next } });
  }

  function numberFrom(event: Event, fallback: number) {
    const value = Number((event.currentTarget as HTMLInputElement).value);
    return Number.isFinite(value) ? value : fallback;
  }

  type Triple = [number, number, number];
  type HsvKey = "hsvLowA" | "hsvHighA" | "hsvLowB" | "hsvHighB";

  function setHsv(key: HsvKey, index: number, event: Event) {
    const next = [...vision.pipeline[key]] as Triple;
    next[index] = numberFrom(event, next[index]);
    patchPipeline({ [key]: next } as Partial<PollenPipelineSettings>);
  }

  /** OpenCV HSV (H 0-179, S/V 0-255) as a CSS colour, for the swatches. */
  function hsvCss([h, s, v]: Triple): string {
    const hue = (h * 2) % 360;
    const sat = s / 255;
    const val = v / 255;
    const c = val * sat;
    const x = c * (1 - Math.abs(((hue / 60) % 2) - 1));
    const m = val - c;
    const [r, g, b] =
      hue < 60 ? [c, x, 0] : hue < 120 ? [x, c, 0] : hue < 180 ? [0, c, x]
        : hue < 240 ? [0, x, c] : hue < 300 ? [x, 0, c] : [c, 0, x];
    const to255 = (value: number) => Math.round((value + m) * 255);
    return `rgb(${to255(r)}, ${to255(g)}, ${to255(b)})`;
  }

  /** A strip across the hue band of a range, at its high S and V, so the range reads as colour. */
  function hueStrip(low: Triple, high: Triple): string {
    const stops = Array.from({ length: 6 }, (_, i) => {
      const hue = low[0] + ((high[0] - low[0]) * i) / 5;
      return hsvCss([hue, high[1], high[2]]);
    });
    return `linear-gradient(90deg, ${stops.join(", ")})`;
  }

  const inputClass =
    "w-full px-2 py-1 rounded-md border border-neutral-300 dark:border-neutral-600 bg-white dark:bg-neutral-800 text-sm focus:outline-none focus:ring-2 focus:ring-yellow-500";
  const labelClass = "block text-xs font-medium text-neutral-700 dark:text-neutral-300 mb-0.5";
  const hintClass = "text-[11px] text-neutral-500 dark:text-neutral-400";

  const mountFields: {
    key: "mountForwardInches" | "mountLeftInches" | "mountHeightInches" | "mountPitchDeg" | "mountYawDeg";
    label: string;
    hint: string;
    step: number;
  }[] = [
    { key: "mountForwardInches", label: "Forward (in)", hint: "Lens ahead of robot centre", step: 0.25 },
    { key: "mountLeftInches", label: "Left (in)", hint: "Lens left of robot centre", step: 0.25 },
    { key: "mountHeightInches", label: "Height (in)", hint: "Lens above the TILES", step: 0.25 },
    { key: "mountPitchDeg", label: "Tilt down (°)", hint: "Below horizontal", step: 0.5 },
    { key: "mountYawDeg", label: "Turn left (°)", hint: "From robot forward", step: 0.5 },
  ];

  const hsvRows: { low: HsvKey; high: HsvKey; label: string; hint: string }[] = [
    { low: "hsvLowA", high: "hsvHighA", label: "Range A", hint: "Warm fluorescent venues" },
    { low: "hsvLowB", high: "hsvHighB", label: "Range B", hint: "Wider net for dim venues" },
  ];
  const hsvEdges = (row: { low: HsvKey; high: HsvKey }): { edge: string; key: HsvKey }[] => [
    { edge: "low", key: row.low },
    { edge: "high", key: row.high },
  ];

  type PipelineNumberKey =
    | "openRadius"
    | "closeHGap"
    | "closeVRadius"
    | "clumpMergeGap"
    | "minArea"
    | "maxArea"
    | "maxAspect"
    | "singleBallAreaPx";

  const pipelineFields: { key: PipelineNumberKey; label: string; hint: string; step: number }[] = [
    { key: "openRadius", label: "Open radius", hint: "Kills specks (px)", step: 1 },
    { key: "closeHGap", label: "Close H gap", hint: "Joins touching balls (px)", step: 1 },
    { key: "closeVRadius", label: "Close V radius", hint: "Rows merged (px)", step: 1 },
    { key: "clumpMergeGap", label: "Clump merge gap", hint: "Blobs into one clump (px)", step: 1 },
    { key: "minArea", label: "Min area", hint: "Smaller is noise (px²)", step: 10 },
    { key: "maxArea", label: "Max area", hint: "Larger is not pollen (px²)", step: 1000 },
    { key: "maxAspect", label: "Max aspect", hint: "Rejects streaks", step: 0.1 },
    { key: "singleBallAreaPx", label: "One ball (px²)", hint: "Calibrate at intake range", step: 10 },
  ];

  const channelNames = ["H", "S", "V"];
  const channelMax = [179, 255, 255];

  $: pipelineIsDefault =
    JSON.stringify(vision.pipeline) === JSON.stringify(DEFAULT_POLLEN_PIPELINE);
</script>

<div class="space-y-4">
  <!-- Measured ------------------------------------------------------------ -->
  <label
    class="flex items-start gap-2 rounded-lg border p-2 {vision.measured
      ? 'border-green-300 dark:border-green-800 bg-green-50 dark:bg-green-950/30'
      : 'border-amber-300 dark:border-amber-700 bg-amber-50 dark:bg-amber-950/30'}"
  >
    <input
      type="checkbox"
      class="mt-0.5"
      checked={vision.measured}
      on:change={(e) => patch({ measured: e.currentTarget.checked })}
    />
    <span class="text-xs text-neutral-700 dark:text-neutral-200">
      <strong>Camera mount measured on the robot.</strong>
      {#if vision.measured}
        Pollen Pickups trust these numbers.
      {:else}
        Until this is ticked, every Pollen Pickup says the positions it plans are guesses, and
        the exported auto carries a NOT MEASURED comment. At the default mount a tilt measured
        1° off moves the estimate 1.3in for a ball 24in out, 3.1in at 40in and 6.7in at 60in —
        which is why the pickup keeps re-aiming as it closes in.
      {/if}
    </span>
  </label>

  <!-- Camera ------------------------------------------------------------------- -->
  <section>
    <h4 class="text-xs font-semibold uppercase tracking-wide text-neutral-500 mb-2">Camera</h4>
    <div class="grid grid-cols-2 gap-3">
      <div class="col-span-2">
        <label class={labelClass} for="vision-camera-name">Webcam name</label>
        <input
          id="vision-camera-name"
          class={inputClass}
          value={vision.cameraName}
          on:change={(e) => patch({ cameraName: e.currentTarget.value })}
        />
        <div class={hintClass}>The hardwareMap name in the robot configuration.</div>
      </div>
      <div>
        <label class={labelClass} for="vision-width">Width (px)</label>
        <input id="vision-width" type="number" step="1" class={inputClass} value={vision.imageWidth}
          on:change={(e) => patch({ imageWidth: numberFrom(e, vision.imageWidth) })} />
      </div>
      <div>
        <label class={labelClass} for="vision-height">Height (px)</label>
        <input id="vision-height" type="number" step="1" class={inputClass} value={vision.imageHeight}
          on:change={(e) => patch({ imageHeight: numberFrom(e, vision.imageHeight) })} />
      </div>
      <div>
        <label class={labelClass} for="vision-hfov">Horizontal FOV (°)</label>
        <input id="vision-hfov" type="number" step="0.5" class={inputClass} value={vision.horizontalFovDeg}
          on:change={(e) => patch({ horizontalFovDeg: numberFrom(e, vision.horizontalFovDeg) })} />
      </div>
      <div>
        <label class={labelClass} for="vision-vfov">Vertical FOV (°)</label>
        <input id="vision-vfov" type="number" step="0.5" class={inputClass} value={vision.verticalFovDeg}
          on:change={(e) => patch({ verticalFovDeg: numberFrom(e, vision.verticalFovDeg) })} />
      </div>
      <div class="col-span-2 {hintClass}">
        The pipeline's pixel thresholds are for this resolution. Field of view is at this
        resolution too — many webcams crop when switched from 16:9 to 4:3.
      </div>
    </div>
  </section>

  <!-- Mount ---------------------------------------------------------------------- -->
  <section>
    <h4 class="text-xs font-semibold uppercase tracking-wide text-neutral-500 mb-2">Mount</h4>
    <div class="grid grid-cols-2 sm:grid-cols-3 gap-3">
      {#each mountFields as field}
        <div>
          <label class={labelClass} for="vision-{field.key}">{field.label}</label>
          <input
            id="vision-{field.key}"
            type="number"
            step={field.step}
            class={inputClass}
            value={vision[field.key]}
            on:change={(e) => patch({ [field.key]: numberFrom(e, vision[field.key]) })}
          />
          <div class={hintClass}>{field.hint}</div>
        </div>
      {/each}
    </div>
  </section>

  <!-- Range ------------------------------------------------------------------------ -->
  <section>
    <h4 class="text-xs font-semibold uppercase tracking-wide text-neutral-500 mb-2">Range</h4>
    <div class="grid grid-cols-2 gap-3 items-end">
      <div>
        <label class={labelClass} for="vision-max-range">Max range (in)</label>
        <input id="vision-max-range" type="number" step="1" class={inputClass} value={vision.maxRangeInches}
          on:change={(e) => patch({ maxRangeInches: numberFrom(e, vision.maxRangeInches) })} />
        <div class={hintClass}>Detections further out are ignored.</div>
      </div>
      <div class="rounded-md bg-cyan-50 dark:bg-cyan-950/40 border border-cyan-200 dark:border-cyan-900 px-2 py-1.5">
        <div class="text-lg font-semibold tabular-nums text-cyan-900 dark:text-cyan-100">{reach.toFixed(0)} in</div>
        <div class={hintClass}>
          How far out a single POLLEN still passes MIN_AREA ({Math.round(vision.pipeline.minArea)}px),
          straight ahead of the lens. Shaded solid on the field.
        </div>
      </div>
    </div>
  </section>

  <!-- Pipeline ------------------------------------------------------------------------ -->
  <section>
    <div class="flex items-center justify-between mb-2">
      <h4 class="text-xs font-semibold uppercase tracking-wide text-neutral-500">PollenDetectionPipeline</h4>
      <button
        class="text-xs px-2 py-0.5 rounded bg-neutral-200 dark:bg-neutral-700 disabled:opacity-50"
        disabled={pipelineIsDefault}
        on:click={() => patch({ pipeline: JSON.parse(JSON.stringify(DEFAULT_POLLEN_PIPELINE)) })}
        title="Back to the values TeamCode's pipeline ships with"
      >
        Reset to TeamCode defaults
      </button>
    </div>
    <p class="{hintClass} mb-2">
      Tune these against a real webcam with <code>./gradlew pollenCameraTester</code>, then copy
      the numbers here. They are exported as a <code>PollenDetectionPipeline.Config</code>.
    </p>

    <div class="space-y-2">
      {#each hsvRows as row}
        <div class="rounded-md border border-neutral-200 dark:border-neutral-700 p-2">
          <div class="flex items-center justify-between mb-1">
            <span class="text-xs font-semibold text-neutral-700 dark:text-neutral-200">
              {row.label} <span class="font-normal {hintClass}">— {row.hint}</span>
            </span>
            <span class="h-3 w-24 rounded" style="background: {hueStrip(vision.pipeline[row.low], vision.pipeline[row.high])}" title="Hue band at its high saturation and value"></span>
          </div>
          <div class="grid grid-cols-[auto_repeat(3,minmax(0,1fr))_auto] gap-x-2 gap-y-1 items-center">
            {#each hsvEdges(row) as { edge, key }}
              <span class="text-[11px] text-neutral-500 w-8">{edge}</span>
              {#each channelNames as channel, index}
                <input
                  type="number"
                  min="0"
                  max={channelMax[index]}
                  step="1"
                  class={inputClass}
                  aria-label="{row.label} {edge} {channel}"
                  title="{channel} {edge} (0-{channelMax[index]})"
                  value={vision.pipeline[key][index]}
                  on:change={(e) => setHsv(key, index, e)}
                />
              {/each}
              <span class="h-5 w-5 rounded border border-neutral-300 dark:border-neutral-600" style="background: {hsvCss(vision.pipeline[key])}" title="This corner of the range"></span>
            {/each}
          </div>
        </div>
      {/each}

      <div class="grid grid-cols-2 sm:grid-cols-4 gap-3">
        {#each pipelineFields as field}
          <div>
            <label class={labelClass} for="vision-{field.key}">{field.label}</label>
            <input
              id="vision-{field.key}"
              type="number"
              step={field.step}
              class={inputClass}
              value={vision.pipeline[field.key]}
              on:change={(e) => patchPipeline({ [field.key]: numberFrom(e, vision.pipeline[field.key]) })}
            />
            <div class={hintClass}>{field.hint}</div>
          </div>
        {/each}
      </div>
    </div>
  </section>
</div>
