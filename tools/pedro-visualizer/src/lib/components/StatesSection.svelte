<script lang="ts">
  /**
   * The States tab: every state the route passes through, and the visual effect
   * assigned to it. Assignments are stored by state name in `settings.stateVfx`,
   * so they persist, travel inside the .pp, and apply to every occurrence.
   */
  import type { Line, Point, SequenceItem, Settings, TimePrediction, Variable } from "../../types";
  import { buildEventTimingWindows } from "../../utils/timeCalculator";
  import {
    STATE_GROUP_LABELS,
    VFX_KINDS,
    activeStatesAt,
    routeStatesOf,
    stateWindowsOf,
    normalizeVfxKind,
    suggestVfx,
    type RouteState,
    type StateGroup,
    type VfxKind,
  } from "../../utils/robotStates";

  export let settings: Settings;
  export let timePrediction: TimePrediction | null = null;
  export let startPoint: Point;
  export let lines: Line[] = [];
  export let sequence: SequenceItem[] = [];
  export let variables: Variable[] = [];
  export let percent = 0;
  export let handleSeek: (percent: number) => void = () => {};
  export let recordChange: () => void = () => {};

  $: windows = stateWindowsOf(
    timePrediction?.timeline || [],
    buildEventTimingWindows(startPoint, lines, timePrediction, settings, sequence, variables),
  );
  $: states = routeStatesOf(windows);
  $: totalTime = timePrediction?.totalTime || 0;
  $: nowSeconds = (totalTime * percent) / 100;
  $: activeKeys = new Set(activeStatesAt(windows, nowSeconds).map((w) => `${w.group}|${w.key}`));

  $: assignments = Object.fromEntries(
    Object.entries(settings.stateVfx || {}).map(([key, kind]) => [key, normalizeVfxKind(kind)]),
  ) as Record<string, VfxKind>;
  $: grouped = (["event", "marker", "pollen", "motion"] as StateGroup[])
    .map((group) => ({ group, states: states.filter((state) => state.group === group) }))
    .filter((entry) => entry.states.length > 0);
  $: routeKeys = new Set(states.map((state) => state.key));
  $: orphaned = Object.entries(assignments).filter(
    ([key, kind]) => kind && kind !== "none" && !routeKeys.has(key),
  );
  $: assignedInRoute = states.filter((state) => (assignments[state.key] ?? "none") !== "none").length;
  $: suggestible = states.filter(
    (state) => (assignments[state.key] ?? "none") === "none" && suggestVfx(state.key) !== "none",
  );

  function assign(key: string, kind: VfxKind) {
    const next = { ...assignments };
    if (kind === "none") delete next[key];
    else next[key] = kind;
    settings = { ...settings, stateVfx: next };
    recordChange();
  }

  function applySuggestions() {
    const next = { ...assignments };
    for (const state of suggestible) next[state.key] = suggestVfx(state.key);
    settings = { ...settings, stateVfx: next };
    recordChange();
  }

  function clearAll() {
    settings = { ...settings, stateVfx: {} };
    recordChange();
  }

  /** Jump playback into the state's first window, just past its start. */
  function preview(state: RouteState) {
    if (!(totalTime > 0)) return;
    const window = windows
      .filter((w) => w.key === state.key && w.group === state.group)
      .sort((a, b) => a.startTime - b.startTime)[0];
    if (!window) return;
    const into = Math.min(window.startTime + (window.endTime - window.startTime) * 0.25, window.endTime);
    handleSeek((into / totalTime) * 100);
  }

  const groupDot: Record<StateGroup, string> = {
    event: "bg-purple-500",
    marker: "bg-pink-500",
    pollen: "bg-yellow-400",
    motion: "bg-sky-500",
  };

  const kindTone: Record<VfxKind, string> = {
    none: "bg-neutral-200 text-neutral-700 dark:bg-neutral-700 dark:text-neutral-200",
    shoot: "bg-orange-500 text-white",
    roll: "bg-cyan-500 text-white",
    flip: "bg-fuchsia-500 text-white",
  };

  const EFFECT_OPTIONS: { id: VfxKind; label: string }[] = [{ id: "none", label: "None" }, ...VFX_KINDS];

  const seconds = (value: number) => `${value.toFixed(1)}s`;
</script>

<div class="w-full flex flex-col gap-3">
  <!-- Header -->
  <div class="rounded-md border border-neutral-200 dark:border-neutral-700 bg-white dark:bg-neutral-800 p-3">
    <div class="flex flex-wrap items-center justify-between gap-2">
      <div>
        <h3 class="text-sm font-semibold text-neutral-800 dark:text-neutral-100">State effects</h3>
        <p class="text-xs text-neutral-500 dark:text-neutral-400">
          Pick an effect for anything the robot does. It plays on the robot during playback, every time that state comes up.
        </p>
      </div>
      <div class="flex items-center gap-2">
        <button
          class="px-2 py-1 text-xs font-semibold rounded bg-amber-100 text-amber-800 dark:bg-amber-900 dark:text-amber-100 disabled:opacity-50"
          disabled={suggestible.length === 0}
          title={suggestible.length
            ? suggestible.map((state) => `${state.key} → ${suggestVfx(state.key)}`).join("\n")
            : "Nothing to suggest for unassigned states"}
          on:click={applySuggestions}
        >
          Suggest{suggestible.length ? ` (${suggestible.length})` : ""}
        </button>
        <button
          class="px-2 py-1 text-xs font-semibold rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-700 dark:text-neutral-200 disabled:opacity-50"
          disabled={Object.keys(assignments).length === 0}
          on:click={clearAll}
        >
          Clear all
        </button>
      </div>
    </div>

    <!-- Effect legend with live previews -->
    <div class="mt-3 grid grid-cols-3 gap-2">
      {#each VFX_KINDS as kind}
        <div class="flex items-center gap-2 rounded-md border border-neutral-200 dark:border-neutral-700 p-2">
          <div class="vfx-demo vfx-demo-{kind.id}" aria-hidden="true">
            <div class="vfx-demo-robot"></div>
            {#if kind.id === "shoot"}
              <span class="demo-goal"></span>
              <span class="demo-ball b1"></span><span class="demo-ball b2"></span>
            {/if}
          </div>
          <div class="min-w-0">
            <div class="text-xs font-semibold text-neutral-800 dark:text-neutral-100">{kind.label}</div>
            <div class="text-[10px] leading-tight text-neutral-500 dark:text-neutral-400">{kind.hint}</div>
          </div>
        </div>
      {/each}
    </div>
  </div>

  {#if states.length === 0}
    <div class="rounded-md border border-dashed border-neutral-300 dark:border-neutral-700 p-4 text-xs text-neutral-500 text-center">
      This route has no states yet. Add paths, events, waits or Pollen Pickups on the Route tab.
    </div>
  {:else}
    <div class="text-[11px] text-neutral-500 dark:text-neutral-400 px-1">
      {assignedInRoute} of {states.length} state{states.length === 1 ? "" : "s"} in this route have an effect.
      Where two states overlap — a marker on a path — the more specific one plays.
    </div>

    {#each grouped as { group, states: groupStates }}
      <section class="rounded-md border border-neutral-200 dark:border-neutral-700 bg-white dark:bg-neutral-800">
        <h4 class="px-3 pt-2 pb-1 text-[11px] font-semibold uppercase tracking-wide text-neutral-500">
          {STATE_GROUP_LABELS[group]}
        </h4>
        <ul>
          {#each groupStates as state (state.group + state.key)}
            {@const kind = assignments[state.key] ?? "none"}
            {@const active = activeKeys.has(`${state.group}|${state.key}`)}
            <li
              class="flex flex-wrap items-center justify-between gap-2 px-3 py-2 border-t border-neutral-100 dark:border-neutral-700/60 {active
                ? 'bg-emerald-50 dark:bg-emerald-950/30'
                : ''}"
            >
              <div class="flex min-w-0 items-center gap-2">
                <span class="size-2 shrink-0 rounded-full {groupDot[group]}"></span>
                <div class="min-w-0">
                  <div class="flex items-center gap-1.5">
                    <span class="truncate text-sm font-medium text-neutral-800 dark:text-neutral-100" title={state.key}>{state.key}</span>
                    {#if active}
                      <span class="px-1 py-0.5 text-[9px] font-bold uppercase rounded bg-emerald-500 text-white" title="The robot is in this state at the current playback position">now</span>
                    {/if}
                  </div>
                  <div class="text-[11px] text-neutral-500 tabular-nums">
                    {state.count}× · {seconds(state.seconds)} · first at {seconds(state.firstStart)}
                  </div>
                </div>
              </div>

              <div class="flex items-center gap-2">
                <div class="inline-flex overflow-hidden rounded-md border border-neutral-300 dark:border-neutral-600" role="radiogroup" aria-label="Effect for {state.key}">
                  {#each EFFECT_OPTIONS as option}
                    <button
                      role="radio"
                      aria-checked={kind === option.id}
                      class="px-2 py-1 text-[11px] font-semibold transition-colors {kind === option.id
                        ? kindTone[option.id]
                        : 'bg-white text-neutral-600 hover:bg-neutral-100 dark:bg-neutral-900 dark:text-neutral-300 dark:hover:bg-neutral-800'}"
                      on:click={() => assign(state.key, option.id)}
                    >
                      {option.label}
                    </button>
                  {/each}
                </div>
                <button
                  class="p-1 rounded text-neutral-500 hover:text-emerald-600 disabled:opacity-40"
                  title="Jump playback to the first time the robot is in this state"
                  disabled={!(totalTime > 0)}
                  on:click={() => preview(state)}
                >
                  <svg viewBox="0 0 24 24" fill="currentColor" class="size-4"><path d="M8 5.14v13.72a1 1 0 0 0 1.52.85l10.29-6.86a1 1 0 0 0 0-1.7L9.52 4.29A1 1 0 0 0 8 5.14Z" /></svg>
                </button>
              </div>
            </li>
          {/each}
        </ul>
      </section>
    {/each}
  {/if}

  {#if orphaned.length}
    <section class="rounded-md border border-neutral-200 dark:border-neutral-700 bg-white dark:bg-neutral-800 p-3">
      <h4 class="text-[11px] font-semibold uppercase tracking-wide text-neutral-500 mb-1">Saved, not in this route</h4>
      <p class="text-[11px] text-neutral-500 mb-2">These play whenever a route has a state with the same name.</p>
      <ul class="flex flex-wrap gap-1.5">
        {#each orphaned as [key, kind]}
          <li class="flex items-center gap-1 rounded-full bg-neutral-100 dark:bg-neutral-700 pl-2 pr-1 py-0.5 text-[11px]">
            <span class="text-neutral-700 dark:text-neutral-200">{key}</span>
            <span class="px-1 rounded {kindTone[kind]}">{kind}</span>
            <button class="px-1 text-neutral-500 hover:text-red-600" title="Forget this assignment" on:click={() => assign(key, "none")}>×</button>
          </li>
        {/each}
      </ul>
    </section>
  {/if}
</div>

<style>
  .vfx-demo {
    position: relative;
    width: 38px;
    height: 30px;
    flex-shrink: 0;
    perspective: 120px;
  }
  .vfx-demo-robot {
    position: absolute;
    left: 6px;
    top: 8px;
    width: 18px;
    height: 14px;
    border-radius: 3px;
    background: linear-gradient(90deg, #475569 0%, #94a3b8 70%, #e2e8f0 100%);
    box-shadow: 0 1px 2px rgba(0, 0, 0, 0.35);
  }
  .vfx-demo-roll .vfx-demo-robot {
    animation: demo-roll 1.2s linear infinite;
  }
  .vfx-demo-flip .vfx-demo-robot {
    animation: demo-flip 1.4s ease-in-out infinite;
  }
  .demo-goal {
    position: absolute;
    right: 0;
    top: 2px;
    width: 9px;
    height: 9px;
    border-radius: 999px;
    border: 2px solid #ef4444;
  }
  .demo-ball {
    position: absolute;
    left: 20px;
    top: 13px;
    width: 5px;
    height: 5px;
    border-radius: 999px;
    background: #ef4444;
    box-shadow: 0 0 0 1px rgba(0, 0, 0, 0.4);
    animation: demo-shot 0.9s ease-in infinite;
  }
  .demo-ball.b2 { animation-delay: 0.45s; }
  @keyframes demo-roll {
    from { transform: rotateX(0deg); }
    to { transform: rotateX(360deg); }
  }
  @keyframes demo-flip {
    0% { transform: scale(1) rotateY(0deg); }
    50% { transform: scale(1.3) rotateY(180deg); }
    100% { transform: scale(1) rotateY(360deg); }
  }
  @keyframes demo-shot {
    0% { transform: translate(0, 0) scale(1); opacity: 1; }
    50% { transform: translate(7px, -9px) scale(1.5); opacity: 1; }
    90% { transform: translate(13px, -8px) scale(1); opacity: 1; }
    100% { transform: translate(13px, -8px) scale(1); opacity: 0; }
  }
  @media (prefers-reduced-motion: reduce) {
    .vfx-demo-roll .vfx-demo-robot,
    .vfx-demo-flip .vfx-demo-robot,
    .demo-ball {
      animation: none;
    }
  }
</style>
