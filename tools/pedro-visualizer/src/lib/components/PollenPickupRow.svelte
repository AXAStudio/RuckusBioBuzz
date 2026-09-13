<script lang="ts">
  /**
   * One Pollen Pickup step in the sequence.
   *
   * Collapsed it is a single row that still says everything that matters -
   * how long the step takes, whether anything is wrong with it, and whether its
   * legs hit anything. Expanded it is the editor, with a live preview of what
   * the camera sees from where the step starts, drawn with the same projection
   * the generated Java drives with.
   */
  import type { SequencePollenItem, Variable, VisionSettings } from "../../types";
  import type { ClearanceSpan } from "../../utils/clearance";
  import { describeClearanceSpan } from "../../utils/clearance";
  import {
    fieldToPixel,
    type PollenFinding,
    type PollenStepPlan,
  } from "../../utils/pollenVision";
  import { scrubbable } from "../../utils/scrub";
  import ExpressionInput from "./ExpressionInput.svelte";

  export let item: SequencePollenItem;
  /** Null when the step is skipped: its Enabled-if is false, so it never runs. */
  export let plan: PollenStepPlan | null = null;
  export let vision: VisionSettings;
  export let clearanceSpans: ClearanceSpan[] = [];
  export let variables: Variable[] = [];
  export let expanded = false;
  /**
   * Open/closed lives with the caller. Kept here it would be reset whenever the
   * step changes - which is every edit, and every frame of dragging the marker.
   */
  export let onExpandedChange: (value: boolean) => void = () => {};
  const setExpanded = (value: boolean) => {
    expanded = value;
    onExpandedChange(value);
  };

  export let onChange: (patch: Partial<SequencePollenItem>) => void;
  export let onCommit: () => void = () => {};
  export let onEnabledExpressionChange: (raw: string) => void = () => {};
  export let onToggleLock: () => void = () => {};
  export let onRemove: () => void = () => {};
  export let onMoveUp: () => void = () => {};
  export let onMoveDown: () => void = () => {};
  export let onAddPathAfter: () => void = () => {};
  export let onAimAhead: () => void = () => {};
  export let canMoveUp = true;
  export let canMoveDown = true;
  export let onPointerDown: (event: PointerEvent) => void = () => {};

  $: locked = Boolean(item.locked);
  $: errors = plan?.findings.filter((f) => f.level === "error") ?? [];
  $: warnings = plan?.findings.filter((f) => f.level === "warning") ?? [];
  $: hits = clearanceSpans.filter((span) => span.severity === "hit");
  $: tight = clearanceSpans.filter((span) => span.severity === "tight");

  const num = (value: string | number, fallback: number) => {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : fallback;
  };

  function set<K extends keyof SequencePollenItem>(key: K, raw: string | number | boolean) {
    if (locked) return;
    const current = item[key];
    const value =
      typeof current === "number" ? num(raw as string, current as number) : raw;
    onChange({ [key]: value } as Partial<SequencePollenItem>);
  }

  const seconds = (value: number) => `${value.toFixed(1)}s`;

  // --- camera preview ---------------------------------------------------------
  const PREVIEW_W = 240;
  $: previewH = Math.round((PREVIEW_W * vision.imageHeight) / vision.imageWidth);
  $: scale = PREVIEW_W / vision.imageWidth;

  $: zonePixels = plan
    ? Array.from({ length: 36 }, (_, i) => {
        const a = (i / 36) * Math.PI * 2;
        return fieldToPixel(
          {
            x: item.targetX + item.zoneRadius * Math.cos(a),
            y: item.targetY + item.zoneRadius * Math.sin(a),
          },
          plan.searchPose,
          vision,
        );
      })
    : [];
  // A zone that wraps behind the lens cannot be drawn as one outline.
  $: zoneDrawable = zonePixels.length > 0 && zonePixels.every((p) => p !== null);
  $: zonePath = zoneDrawable
    ? zonePixels
        .map((p, i) => `${i === 0 ? "M" : "L"}${(p!.u * scale).toFixed(1)},${(p!.v * scale).toFixed(1)}`)
        .join(" ") + " Z"
    : "";
  $: ballRadiusPx =
    plan && plan.targetPixelArea > 0 ? Math.sqrt(plan.targetPixelArea / Math.PI) * scale : 0;
  $: minAreaRadiusPx = Math.sqrt(vision.pipeline.minArea / Math.PI) * scale;

  const findingClass = (finding: PollenFinding) =>
    finding.level === "error"
      ? "text-red-700 dark:text-red-300"
      : finding.level === "warning"
        ? "text-amber-700 dark:text-amber-300"
        : "text-neutral-600 dark:text-neutral-400";

  const findingMark = (finding: PollenFinding) =>
    finding.level === "error" ? "✗" : finding.level === "warning" ? "!" : "·";

  type NumericKey =
    | "targetX"
    | "targetY"
    | "zoneRadius"
    | "minBalls"
    | "standoffInches"
    | "retargetInches"
    | "approachSpeed"
    | "intakeDriveInches"
    | "intakeMs";

  interface NumericField {
    key: NumericKey;
    label: string;
    step: number;
    scrub: number;
    title: string;
  }

  const targetFields: NumericField[] = [
    { key: "targetX", label: "X", step: 0.5, scrub: 0.25, title: "Expected POLLEN, field inches" },
    { key: "targetY", label: "Y", step: 0.5, scrub: 0.25, title: "Expected POLLEN, field inches" },
    { key: "zoneRadius", label: "Zone r", step: 0.5, scrub: 0.25, title: "Clumps further than this from the expected point are ignored, inches" },
    { key: "minBalls", label: "Min balls", step: 1, scrub: 0.05, title: "Smallest clump worth driving to, in estimated balls" },
  ];

  const approachFields: NumericField[] = [
    { key: "standoffInches", label: "Standoff in", step: 0.5, scrub: 0.25, title: "Stop this far short of the POLLEN centre so the intake reaches it" },
    { key: "retargetInches", label: "Retarget in", step: 0.5, scrub: 0.25, title: "Re-plan the approach when the estimate moves further than this" },
    { key: "approachSpeed", label: "Speed", step: 0.05, scrub: 0.005, title: "Fraction of max velocity for the approach (0.05-1)" },
    { key: "intakeDriveInches", label: "Intake drive in", step: 0.5, scrub: 0.25, title: "Creep forward this far with the intake running" },
    { key: "intakeMs", label: "Intake ms", step: 50, scrub: 5, title: "Run the intake at least this long" },
  ];

  const inputClass =
    "w-full px-1.5 py-0.5 rounded-md bg-neutral-50 dark:bg-neutral-950 border border-neutral-300 dark:border-neutral-700 text-xs focus:outline-none focus:ring-1 focus:ring-yellow-500 disabled:opacity-60";
  const labelClass = "text-[11px] text-neutral-600 dark:text-neutral-400 whitespace-nowrap";
</script>

<div
  class="w-full rounded border bg-yellow-50/60 dark:bg-yellow-950/20 {errors.length || hits.length
    ? 'border-red-300 dark:border-red-800'
    : 'border-yellow-300 dark:border-yellow-800'}"
>
  <!-- ── Header ─────────────────────────────────────────────────────────── -->
  <div class="flex w-full flex-wrap items-center justify-between gap-2 px-2 py-1">
    <div class="flex min-w-0 flex-wrap items-center gap-2">
      <span
        role="button"
        tabindex={locked ? -1 : 0}
        draggable="false"
        on:pointerdown={onPointerDown}
        on:dragstart|preventDefault
        title="A Pollen Pickup stays at the top level of the sequence - reorder it with the arrows"
        aria-label="Drag pollen pickup"
        class="shrink-0 select-none touch-none rounded px-1 py-0.5 text-neutral-300 dark:text-neutral-600 cursor-default"
      >
        <svg viewBox="0 0 24 24" fill="currentColor" class="size-4 pointer-events-none">
          <circle cx="9" cy="6" r="1.6" /><circle cx="15" cy="6" r="1.6" />
          <circle cx="9" cy="12" r="1.6" /><circle cx="15" cy="12" r="1.6" />
          <circle cx="9" cy="18" r="1.6" /><circle cx="15" cy="18" r="1.6" />
        </svg>
      </span>

      <span
        class="px-1.5 py-0.5 text-xs font-semibold rounded bg-yellow-300 text-yellow-950 dark:bg-yellow-600 dark:text-yellow-50"
        title="Vision-guided pickup with PollenDetectionPipeline">Pollen</span
      >

      <input
        class="pl-1.5 rounded-md bg-neutral-50 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-36 text-sm"
        type="text"
        placeholder="Name"
        value={item.name}
        disabled={locked}
        on:input={(e) => set("name", e.currentTarget.value)}
        on:change={onCommit}
      />

      {#if plan}
        <span
          class="text-xs tabular-nums text-neutral-700 dark:text-neutral-300"
          title="Time charged to the route (search at its expected length) · worst case (search runs to its timeout)"
        >
          {seconds(plan.expectedSeconds)} <span class="text-neutral-400">· ≤{seconds(plan.worstSeconds)}</span>
        </span>
      {:else}
        <span
          class="px-1.5 py-0.5 text-[11px] font-semibold rounded bg-neutral-200 text-neutral-600 dark:bg-neutral-800 dark:text-neutral-300"
          title="Its Enabled-if condition is false, so this step is skipped">Skipped</span
        >
      {/if}

      {#if errors.length}
        <button
          class="px-1.5 py-0.5 text-[11px] font-semibold rounded bg-red-200 text-red-900 dark:bg-red-900 dark:text-red-100"
          title={errors.map((f) => f.message).join("\n")}
          on:click={() => setExpanded(true)}
        >
          {errors.length} problem{errors.length === 1 ? "" : "s"}
        </button>
      {/if}
      {#if warnings.length}
        <button
          class="px-1.5 py-0.5 text-[11px] font-semibold rounded bg-amber-200 text-amber-900 dark:bg-amber-900 dark:text-amber-100"
          title={warnings.map((f) => f.message).join("\n")}
          on:click={() => setExpanded(true)}
        >
          {warnings.length} warning{warnings.length === 1 ? "" : "s"}
        </button>
      {/if}
      {#if hits.length}
        <span
          class="px-1.5 py-0.5 text-[11px] font-semibold rounded bg-red-200 text-red-900 dark:bg-red-900 dark:text-red-100"
          title={hits.map((span) => describeClearanceSpan(span)).join("\n")}
          >{describeClearanceSpan(hits[0])}</span
        >
      {:else if tight.length}
        <span
          class="px-1.5 py-0.5 text-[11px] font-semibold rounded bg-amber-200 text-amber-900 dark:bg-amber-900 dark:text-amber-100"
          title={tight.map((span) => describeClearanceSpan(span)).join("\n")}
          >{describeClearanceSpan(tight[0])}</span
        >
      {/if}
    </div>

    <div class="flex items-center gap-1.5">
      <button
        title={expanded ? "Collapse" : "Edit this pickup"}
        on:click={() => setExpanded(!expanded)}
        class="p-1 rounded text-neutral-500 hover:text-neutral-800 dark:hover:text-neutral-200"
        aria-expanded={expanded}
      >
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" class="size-4 transition-transform {expanded ? 'rotate-180' : ''}">
          <path stroke-linecap="round" stroke-linejoin="round" d="m19 9-7 7-7-7" />
        </svg>
      </button>
      <button
        title={locked ? "Unlock" : "Lock"}
        on:click|stopPropagation={onToggleLock}
        class="p-1 rounded"
      >
        <svg viewBox="0 0 24 24" fill="none" stroke-width="2" stroke="currentColor" class="size-5 {locked ? 'stroke-yellow-500' : 'stroke-gray-400'}">
          {#if locked}
            <path stroke-linecap="round" stroke-linejoin="round" d="M16.5 10.5V6.75a4.5 4.5 0 1 0-9 0v3.75m-.75 11.25h10.5a2.25 2.25 0 0 0 2.25-2.25v-6.75a2.25 2.25 0 0 0-2.25-2.25H6.75a2.25 2.25 0 0 0-2.25 2.25v6.75a2.25 2.25 0 0 0 2.25 2.25Z" />
          {:else}
            <path stroke-linecap="round" stroke-linejoin="round" d="M13.5 10.5V6.75a4.5 4.5 0 1 1 9 0v3.75M3.75 21.75h10.5a2.25 2.25 0 0 0 2.25-2.25v-6.75a2.25 2.25 0 0 0-2.25-2.25H3.75a2.25 2.25 0 0 0-2.25 2.25v6.75a2.25 2.25 0 0 0 2.25 2.25Z" />
          {/if}
        </svg>
      </button>
      <button
        title="Move up"
        disabled={!canMoveUp || locked}
        on:click={onMoveUp}
        class="p-1 rounded-full text-neutral-500 bg-neutral-100/70 dark:bg-neutral-900/70 border border-neutral-200/70 dark:border-neutral-700/70 disabled:opacity-40 disabled:cursor-not-allowed"
      >
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" class="size-4"><path stroke-linecap="round" stroke-linejoin="round" d="m5 15 7-7 7 7" /></svg>
      </button>
      <button
        title="Move down"
        disabled={!canMoveDown || locked}
        on:click={onMoveDown}
        class="p-1 rounded-full text-neutral-500 bg-neutral-100/70 dark:bg-neutral-900/70 border border-neutral-200/70 dark:border-neutral-700/70 disabled:opacity-40 disabled:cursor-not-allowed"
      >
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" class="size-4"><path stroke-linecap="round" stroke-linejoin="round" d="m19 9-7 7-7-7" /></svg>
      </button>
      <button title="Add path after" disabled={locked} on:click={onAddPathAfter} class="text-green-500 hover:text-green-600 disabled:opacity-40">
        <svg viewBox="0 0 24 24" fill="none" stroke-width="2" stroke="currentColor" class="size-5"><path stroke-linecap="round" stroke-linejoin="round" d="M12 4.5v15m7.5-7.5h-15" /></svg>
      </button>
      <button title="Remove" disabled={locked} on:click={onRemove} class="text-red-500 hover:text-red-600 disabled:opacity-40">
        <svg viewBox="0 0 24 24" fill="none" stroke-width="2" stroke="currentColor" class="size-5"><path stroke-linecap="round" stroke-linejoin="round" d="m14.74 9-.346 9m-4.788 0L9.26 9m9.968-3.21c.342.052.682.107 1.022.166m-1.022-.165L18.16 19.673a2.25 2.25 0 0 1-2.244 2.077H8.084a2.25 2.25 0 0 1-2.244-2.077L4.772 5.79m14.456 0a48.108 48.108 0 0 0-3.478-.397m-12 .562c.34-.059.68-.114 1.022-.165m0 0a48.11 48.11 0 0 1 3.478-.397m7.5 0v-.916c0-1.18-.91-2.164-2.09-2.201a51.964 51.964 0 0 0-3.32 0c-1.18.037-2.09 1.022-2.09 2.201v.916m7.5 0a48.667 48.667 0 0 0-7.5 0" /></svg>
      </button>
    </div>
  </div>

  <!-- ── Editor ─────────────────────────────────────────────────────────── -->
  {#if expanded}
    <div class="border-t border-yellow-200 dark:border-yellow-900 px-2 py-2 flex flex-col gap-3">
      {#if !vision.measured}
        <div class="rounded border border-amber-300 dark:border-amber-700 bg-amber-50 dark:bg-amber-950/40 px-2 py-1 text-[11px] text-amber-800 dark:text-amber-200">
          <strong>Camera mount not measured.</strong> Every field position below comes from it —
          measure the lens height, tilt and offsets on the robot, enter them in
          <em>Settings → Vision</em>, and tick <em>Measured</em>.
        </div>
      {/if}

      <div class="grid grid-cols-1 gap-3 lg:grid-cols-[minmax(0,1fr)_auto]">
        <div class="flex flex-col gap-3 min-w-0">
          <!-- Target -->
          <section>
            <div class="flex items-center justify-between">
              <h4 class="text-[11px] font-semibold uppercase tracking-wide text-neutral-500">Where to look</h4>
              <button
                class="text-[11px] px-1.5 py-0.5 rounded bg-yellow-200 text-yellow-900 dark:bg-yellow-800 dark:text-yellow-50 disabled:opacity-50"
                disabled={locked || !plan}
                on:click={onAimAhead}
                title="Put the expected POLLEN straight ahead of the camera, well inside the range a single ball registers"
              >
                Aim straight ahead
              </button>
            </div>
            <p class="text-[11px] text-neutral-500 mb-1">Drag the yellow marker on the field, or type it.</p>
            <div class="grid grid-cols-[repeat(auto-fill,minmax(7.5rem,1fr))] gap-x-3 gap-y-1.5">
              {#each targetFields as field}
                <label class="flex items-center gap-1.5">
                  <span
                    class={labelClass}
                    title={field.title}
                    use:scrubbable={{
                      value: item[field.key],
                      step: field.scrub,
                      disabled: locked,
                      onInput: (raw) => set(field.key, raw),
                      onCommit: () => onCommit(),
                    }}>{field.label}</span
                  >
                  <input
                    type="number"
                    step={field.step}
                    class={inputClass}
                    value={item[field.key]}
                    disabled={locked}
                    title={field.title}
                    on:input={(e) => set(field.key, e.currentTarget.value)}
                    on:change={onCommit}
                  />
                </label>
              {/each}
            </div>
          </section>

          <!-- Search -->
          <section>
            <h4 class="text-[11px] font-semibold uppercase tracking-wide text-neutral-500 mb-1">Search</h4>
            <div class="grid grid-cols-[repeat(auto-fill,minmax(9.5rem,1fr))] gap-x-3 gap-y-1.5 items-center">
              <label class="flex items-center gap-1.5" title="Give up after this long without a clump in the zone">
                <span class={labelClass}>Timeout ms</span>
                <input type="number" step="100" class={inputClass} value={item.timeoutMs} disabled={locked}
                  on:input={(e) => set("timeoutMs", e.currentTarget.value)} on:change={onCommit} />
              </label>
              <label class="flex items-center gap-1.5" title="How long finding it usually takes - what the time estimate charges">
                <span class={labelClass}>Expected ms</span>
                <input type="number" step="50" class={inputClass} value={item.expectedSearchMs} disabled={locked}
                  on:input={(e) => set("expectedSearchMs", e.currentTarget.value)} on:change={onCommit} />
              </label>
              <div class="flex items-center gap-1.5 col-span-full" role="radiogroup" aria-label="On timeout">
                <span class={labelClass}>On timeout</span>
                <div class="inline-flex rounded-md border border-neutral-300 dark:border-neutral-700 overflow-hidden text-[11px]">
                  {#each [["skip", "Skip"], ["blind", "Blind pickup"]] as [value, text]}
                    <button
                      role="radio"
                      aria-checked={item.onTimeout === value}
                      disabled={locked}
                      class="px-2 py-0.5 whitespace-nowrap {item.onTimeout === value
                        ? 'bg-yellow-400 text-yellow-950 font-semibold'
                        : 'bg-white dark:bg-neutral-900 text-neutral-600 dark:text-neutral-300'}"
                      title={value === "skip"
                        ? "Carry on with the route without moving"
                        : "Drive to the expected point anyway"}
                      on:click={() => {
                        set("onTimeout", value);
                        onCommit();
                      }}>{text}</button
                    >
                  {/each}
                </div>
              </div>
            </div>
          </section>

          <!-- Approach & intake -->
          <section>
            <h4 class="text-[11px] font-semibold uppercase tracking-wide text-neutral-500 mb-1">Approach &amp; intake</h4>
            <div class="grid grid-cols-[repeat(auto-fill,minmax(9.5rem,1fr))] gap-x-3 gap-y-1.5 items-center">
              {#each approachFields as field}
                <label class="flex items-center gap-1.5" title={field.title}>
                  <span
                    class={labelClass}
                    use:scrubbable={{
                      value: item[field.key],
                      step: field.scrub,
                      disabled: locked,
                      onInput: (raw) => set(field.key, raw),
                      onCommit: () => onCommit(),
                    }}>{field.label}</span
                  >
                  <input type="number" step={field.step} class={inputClass} value={item[field.key]} disabled={locked}
                    on:input={(e) => set(field.key, e.currentTarget.value)} on:change={onCommit} />
                </label>
              {/each}
              <label class="flex items-center gap-1.5 col-span-full sm:col-span-1 min-w-[12rem]" title="Event started for the intake and finished after it (blank for none)">
                <span class={labelClass}>Intake event</span>
                <input type="text" class={inputClass} list="pollen-intake-events" value={item.intakeEvent} disabled={locked}
                  on:input={(e) => set("intakeEvent", e.currentTarget.value)} on:change={onCommit} />
                <datalist id="pollen-intake-events">
                  <option value="Intake" /><option value="Intake On" />
                </datalist>
              </label>
            </div>
          </section>

          <!-- After -->
          <section class="flex flex-wrap items-center gap-x-4 gap-y-1.5">
            <label class="flex items-center gap-1.5 text-xs text-neutral-700 dark:text-neutral-300"
              title="Drive back to where the step started, so every later path starts where it was planned from">
              <input type="checkbox" checked={item.returnToStart} disabled={locked}
                on:change={(e) => {
                  set("returnToStart", e.currentTarget.checked);
                  onCommit();
                }} />
              Return to the search pose afterwards
            </label>
            <div class="flex items-center gap-1.5 w-40" title="Optional boolean expression; when false this step is skipped">
              <span class={labelClass}>Enabled if</span>
              <ExpressionInput compact kind="boolean" {variables} disabled={locked} placeholder="if…"
                value={item.enabledExpression || ""} onInput={onEnabledExpressionChange} onCommit={() => onCommit()} />
            </div>
          </section>
        </div>

        <!-- Camera preview -->
        <aside class="flex flex-col gap-1 items-start">
          <h4 class="text-[11px] font-semibold uppercase tracking-wide text-neutral-500">What the camera sees</h4>
          {#if plan}
            <svg
              width={PREVIEW_W}
              height={previewH}
              viewBox="0 0 {PREVIEW_W} {previewH}"
              class="rounded border border-neutral-300 dark:border-neutral-700 bg-neutral-800"
              role="img"
              aria-label="Camera image preview from the search pose"
            >
              <line x1={PREVIEW_W / 2} y1="0" x2={PREVIEW_W / 2} y2={previewH} stroke="rgba(255,255,255,0.25)" stroke-dasharray="3 3" />
              <line x1="0" y1={previewH / 2} x2={PREVIEW_W} y2={previewH / 2} stroke="rgba(255,255,255,0.12)" />
              {#if zonePath}
                <path d={zonePath} fill="rgba(250,204,21,0.14)" stroke="rgba(250,204,21,0.9)" stroke-width="1" stroke-dasharray="4 3" />
              {/if}
              {#if plan.targetPixel}
                {@const cx = plan.targetPixel.u * scale}
                {@const cy = plan.targetPixel.v * scale}
                {@const detectable = plan.targetPixelArea >= vision.pipeline.minArea}
                <circle cx={cx} cy={cy} r={Math.max(1.5, minAreaRadiusPx)} fill="none" stroke="rgba(255,255,255,0.35)" stroke-dasharray="2 2">
                  <title>Smallest blob the pipeline keeps (MIN_AREA)</title>
                </circle>
                <circle cx={cx} cy={cy} r={Math.max(1.5, ballRadiusPx)} fill={detectable ? "#facc15" : "#f97316"} stroke="#111" stroke-width="1">
                  <title>One POLLEN at the expected point: ~{Math.round(plan.targetPixelArea)}px</title>
                </circle>
              {/if}
            </svg>
            <dl class="grid grid-cols-[auto_auto] gap-x-2 text-[11px] text-neutral-600 dark:text-neutral-400 tabular-nums">
              <dt>Expected POLLEN</dt>
              <dd class={plan.targetPixel?.inImage ? "" : "text-red-600 dark:text-red-400 font-semibold"}>
                {#if plan.targetPixel?.inImage}
                  ({Math.round(plan.targetPixel.u)}, {Math.round(plan.targetPixel.v)}) px
                {:else}
                  out of view
                {/if}
              </dd>
              <dt>One ball there</dt>
              <dd class={plan.targetPixelArea >= vision.pipeline.minArea ? "" : "text-amber-600 dark:text-amber-400 font-semibold"}>
                ~{Math.round(plan.targetPixelArea)}px (min {Math.round(vision.pipeline.minArea)})
              </dd>
              <dt>Registers out to</dt>
              <dd>{plan.reliableRange.toFixed(0)}in</dd>
              <dt>Reads as</dt>
              <dd>{plan.targetBallEstimate.toFixed(2)} ball{plan.targetBallEstimate >= 0.5 && plan.targetBallEstimate < 1.5 ? "" : "s"}</dd>
            </dl>
          {:else}
            <p class="text-[11px] text-neutral-500 w-60">Skipped by its Enabled-if condition, so there is no pose to look from.</p>
          {/if}
        </aside>
      </div>

      {#if plan && (plan.findings.length || clearanceSpans.length)}
        <ul class="flex flex-col gap-0.5 text-[11px]">
          {#each plan.findings as finding}
            <li class={findingClass(finding)}><span class="inline-block w-3 font-bold">{findingMark(finding)}</span>{finding.message}</li>
          {/each}
          {#each clearanceSpans as span}
            <li class={span.severity === "hit" ? "text-red-700 dark:text-red-300" : "text-amber-700 dark:text-amber-300"}>
              <span class="inline-block w-3 font-bold">{span.severity === "hit" ? "✗" : "!"}</span>{describeClearanceSpan(span)}
            </li>
          {/each}
        </ul>
      {/if}
    </div>
  {/if}
</div>
