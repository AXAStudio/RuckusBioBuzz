<script lang="ts">
  import type {
    BasePoint,
    Line,
    TimePrediction,
    TimelineEvent,
  } from "../../types";
  import type { ScaleLinear } from "d3";
  import { formatTime } from "../../utils";

  export let percent: number = 0;
  export let timePrediction: TimePrediction | null = null;
  export let lines: Line[] = [];
  export let robotXY: BasePoint;
  export let robotHeading: number;
  export let x: ScaleLinear<number, number, number>;
  export let y: ScaleLinear<number, number, number>;

  type ParallelEventState = {
    name: string;
    lineName: string;
    triggerPercent: number;
    triggerTime: number;
    endTime: number;
    durationMs: number;
  };

  function clamp(value: number, min: number, max: number) {
    if (!Number.isFinite(value)) return min;
    return Math.max(min, Math.min(max, value));
  }

  function formatNumber(value: number, digits = 1) {
    if (!Number.isFinite(value)) return "--";
    return value.toFixed(digits);
  }

  function normalizeDegrees(value: number) {
    const normalized = ((((value + 180) % 360) + 360) % 360) - 180;
    return Math.abs(normalized) < 0.5 ? 0 : normalized;
  }

  function getCurrentEvent(
    currentTime: number,
    timeline: TimelineEvent[],
  ): TimelineEvent | null {
    if (!timeline.length) return null;
    return (
      timeline.find(
        (event) => currentTime >= event.startTime && currentTime <= event.endTime,
      ) || timeline[timeline.length - 1]
    );
  }

  function getLineName(lineIndex: number | undefined) {
    if (lineIndex === undefined || lineIndex < 0) return "None";
    const line = lines[lineIndex];
    return line?.name?.trim() || `Path ${lineIndex + 1}`;
  }

  function getStateLabel(event: TimelineEvent | null) {
    if (!event) return "Idle";
    if (event.type === "travel") return "Following path";
    if (event.name?.trim()) return event.name;

    const startHeading = Number(event.startHeading ?? 0);
    const targetHeading = Number(event.targetHeading ?? startHeading);
    if (Math.abs(normalizeDegrees(targetHeading - startHeading)) > 0.5) {
      return "Turning";
    }

    return "Waiting";
  }

  function getPathProgress(currentTime: number, event: TimelineEvent | null) {
    if (!event || event.type !== "travel" || event.duration <= 0) return 0;
    return clamp((currentTime - event.startTime) / event.duration, 0, 1);
  }

  function getParallelEvents(
    currentTime: number,
    totalTime: number,
    timeline: TimelineEvent[],
  ) {
    const active: ParallelEventState[] = [];
    const upcoming: ParallelEventState[] = [];

    timeline.forEach((event) => {
      if (event.type !== "travel") return;
      const lineIndex = event.lineIndex ?? -1;
      const line = lines[lineIndex];
      if (!line?.eventMarkers?.length) return;

      line.eventMarkers.forEach((marker, markerIndex) => {
        const triggerType =
          marker.triggerType === "temporal" || marker.triggerType === "pose"
            ? marker.triggerType
            : "parametric";
        const rawPosition = Number(marker.position ?? 0.5);
        const position = Number.isFinite(rawPosition)
          ? clamp(rawPosition, 0, 1)
          : 0.5;
        const durationMs = Math.max(0, Math.round(Number(marker.durationMs ?? 0) || 0));
        const triggerTime =
          triggerType === "temporal"
            ? event.startTime + Math.max(0, Number(marker.triggerMs ?? 0) || 0) / 1000
            : event.startTime + event.duration * position;
        const endTime =
          durationMs > 0 ? triggerTime + durationMs / 1000 : Math.max(totalTime, triggerTime);
        const item = {
          name: marker.name?.trim() || `Event ${markerIndex + 1}`,
          lineName: getLineName(lineIndex),
          triggerPercent: position * 100,
          triggerTime,
          endTime,
          durationMs,
        };

        if (currentTime >= triggerTime && currentTime <= endTime) {
          active.push(item);
        } else if (currentTime < triggerTime) {
          upcoming.push(item);
        }
      });
    });

    upcoming.sort((a, b) => a.triggerTime - b.triggerTime);
    active.sort((a, b) => a.triggerTime - b.triggerTime);
    return { active, next: upcoming[0] || null };
  }

  $: totalTime = timePrediction?.totalTime ?? 0;
  $: timeline = timePrediction?.timeline ?? [];
  $: currentTime = totalTime > 0 ? (clamp(percent, 0, 100) / 100) * totalTime : 0;
  $: currentEvent = getCurrentEvent(currentTime, timeline);
  $: currentLineIndex =
    currentEvent?.type === "travel" ? currentEvent.lineIndex ?? -1 : -1;
  $: currentLine = currentLineIndex >= 0 ? lines[currentLineIndex] : null;
  $: progress = getPathProgress(currentTime, currentEvent);
  $: fieldX = x?.invert ? x.invert(robotXY?.x ?? 0) : 0;
  $: fieldY = y?.invert ? y.invert(robotXY?.y ?? 0) : 0;
  $: fieldHeading = normalizeDegrees(-Number(robotHeading ?? 0));
  $: parallelState = getParallelEvents(currentTime, totalTime, timeline);
</script>

<div
  class="w-full rounded-md border border-neutral-200 dark:border-neutral-700 bg-white dark:bg-neutral-800 p-3 text-sm"
>
  <div class="flex items-center justify-between gap-2 mb-2">
    <div class="font-semibold">Telemetry</div>
    <div class="text-xs text-neutral-500 dark:text-neutral-400">
      {formatTime(currentTime)} / {formatTime(totalTime)}
    </div>
  </div>

  <div class="grid grid-cols-2 gap-x-4 gap-y-1">
    <div class="text-neutral-500 dark:text-neutral-400">State</div>
    <div class="font-medium truncate">{getStateLabel(currentEvent)}</div>

    <div class="text-neutral-500 dark:text-neutral-400">Path</div>
    <div class="font-medium truncate">{getLineName(currentLineIndex)}</div>

    <div class="text-neutral-500 dark:text-neutral-400">Progress</div>
    <div>{formatNumber(progress * 100, 0)}%</div>

    <div class="text-neutral-500 dark:text-neutral-400">Path speed</div>
    <div>{formatNumber(Number(currentLine?.speed ?? 1), 2)}</div>

    <div class="text-neutral-500 dark:text-neutral-400">Pose</div>
    <div>
      X {formatNumber(fieldX, 1)}, Y {formatNumber(fieldY, 1)}, H {formatNumber(fieldHeading, 0)} deg
    </div>

    <div class="text-neutral-500 dark:text-neutral-400">Active</div>
    <div class="truncate">
      {#if parallelState.active.length > 0}
        {parallelState.active.map((event) => event.name).join(", ")}
      {:else}
        None
      {/if}
    </div>

    <div class="text-neutral-500 dark:text-neutral-400">Next</div>
    <div class="truncate">
      {#if parallelState.next}
        {parallelState.next.name} on {parallelState.next.lineName} @ {formatNumber(parallelState.next.triggerPercent, 0)}%
      {:else}
        None
      {/if}
    </div>
  </div>
</div>
