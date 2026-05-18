<script lang="ts">
  import type { EventMarker, EventTriggerType, Line, PoseVariable } from "../../types";
  import { snapToGrid, showGrid, gridSize } from "../../stores";
  import ControlPointsSection from "./ControlPointsSection.svelte";
  import HeadingControls from "./HeadingControls.svelte";
  import HeadingCurveEditor from "./HeadingCurveEditor.svelte";

  export let line: Line;
  export let idx: number;
  export let lines: Line[];
  export let collapsed: boolean;
  export let collapsedControlPoints: boolean;
  export let onRemove: () => void;
  export let onInsertAfter: () => void;
  export let onInsertMidpoint: () => void;
  export let onAddWaitAfter: () => void;
  export let onAddEventAfter: () => void;
  export let recordChange: () => void;
  export let onMoveUp: () => void;
  export let onMoveDown: () => void;
  export let canMoveUp: boolean = true;
  export let canMoveDown: boolean = true;
  export let optimizeLine: (lineId: string, targetControlPointIndex?: number) => void;
  export let optimizing: boolean = false;
  export let chainOptions: Array<{ id: string; name: string; color: string }> = [];
  export let selectedChainId: string = "";
  export let onChainChange: (chainId: string) => void;
  export let poseVariables: PoseVariable[] = [];
  export let onPoseVariableChange: (lineId: string, poseVariableId: string) => void = () => {};
  export let onHeadingModeChange: (lineId: string, mode: string) => void = () => {};


  $: snapToGridTitle =
    $snapToGrid && $showGrid ? `Snapping to ${$gridSize} grid` : "No snapping";
  $: isEndPointBoundToPoseVariable = Boolean(line.endPoint?.poseVariableId);
  $: boundPoseVariableHeading = (() => {
    const poseVariableId = line.endPoint?.poseVariableId;
    if (!poseVariableId) return null;
    const variable = poseVariables.find((item) => item.id === poseVariableId);
    if (!variable) return null;
    const heading = Number(variable.heading);
    return Number.isFinite(heading) ? heading : 0;
  })();
  $: pathSpeedValue = clampPathSpeed(line?.speed);
  $: eventMarkers = line.eventMarkers || [];

  function toggleCollapsed() {
    collapsed = !collapsed;
  }

  function clampPathSpeed(value: number | undefined): number {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) return 1;
    return Math.max(0.05, Math.min(1, numeric));
  }

  function handlePathSpeedInput(event: Event) {
    const target = event.currentTarget as HTMLInputElement;
    line.speed = clampPathSpeed(Number(target.value));
    lines = [...lines];
  }

  function commitPathSpeed() {
    line.speed = clampPathSpeed(line.speed);
    lines = [...lines];
    recordChange?.();
  }

  function handleChainSelect(event: Event) {
    const target = event.currentTarget as HTMLSelectElement;
    if (onChainChange) {
      onChainChange(target.value);
    }
  }

  function handlePoseVariableSelect(event: Event) {
    onPoseVariableChange(line.id || "", (event.currentTarget as HTMLSelectElement).value);
  }

  function makeEventMarkerId() {
    return `event-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
  }

  function clampEventPosition(value: number | undefined): number {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) return 0.5;
    return Math.max(0, Math.min(1, numeric));
  }

  function clampEventDuration(value: number | undefined): number {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) return 0;
    return Math.max(0, Math.round(numeric));
  }

  function eventTriggerType(marker: EventMarker): EventTriggerType {
    return marker.triggerType === "temporal" || marker.triggerType === "pose"
      ? marker.triggerType
      : "parametric";
  }

  function eventPoseX(marker: EventMarker): number {
    const numeric = Number(marker.poseX);
    return Number.isFinite(numeric) ? numeric : Number(line.endPoint?.x ?? 0);
  }

  function eventPoseY(marker: EventMarker): number {
    const numeric = Number(marker.poseY);
    return Number.isFinite(numeric) ? numeric : Number(line.endPoint?.y ?? 0);
  }

  function normalizeEventMarker(marker: EventMarker, index: number): EventMarker {
    const triggerMs = Number(marker.triggerMs ?? 0);
    return {
      ...marker,
      id: marker.id || makeEventMarkerId(),
      name: marker.name || `Event ${index + 1}`,
      triggerType: eventTriggerType(marker),
      position: clampEventPosition(marker.position),
      triggerMs: Number.isFinite(triggerMs)
        ? Math.max(0, Math.round(triggerMs))
        : 0,
      poseX: eventPoseX(marker),
      poseY: eventPoseY(marker),
      durationMs: clampEventDuration(marker.durationMs),
    };
  }

  function setEventMarkers(nextMarkers: EventMarker[], commit = false) {
    line.eventMarkers = nextMarkers.map(normalizeEventMarker);
    lines = [...lines];
    if (commit) recordChange?.();
  }

  function addEventMarker() {
    const nextIndex = eventMarkers.length;
    setEventMarkers(
      [
        ...eventMarkers,
        {
          id: makeEventMarkerId(),
          name: `Event ${nextIndex + 1}`,
          triggerType: "parametric",
          position: 0.5,
          triggerMs: 0,
          poseX: Number(line.endPoint?.x ?? 0),
          poseY: Number(line.endPoint?.y ?? 0),
          durationMs: 0,
        },
      ],
      true,
    );
  }

  function updateEventMarker(index: number, patch: Partial<EventMarker>, commit = false) {
    const nextMarkers = eventMarkers.map((marker, markerIndex) =>
      markerIndex === index ? { ...marker, ...patch } : marker,
    );
    setEventMarkers(nextMarkers, commit);
  }

  function handleEventNameInput(index: number, event: Event) {
    updateEventMarker(index, {
      name: (event.currentTarget as HTMLInputElement).value,
    });
  }

  function handleEventPositionInput(index: number, event: Event) {
    updateEventMarker(index, {
      position: Number((event.currentTarget as HTMLInputElement).value),
    });
  }

  function handleEventPercentInput(index: number, event: Event) {
    updateEventMarker(index, {
      position: Number((event.currentTarget as HTMLInputElement).value) / 100,
    });
  }

  function handleEventDurationInput(index: number, event: Event) {
    updateEventMarker(index, {
      durationMs: Number((event.currentTarget as HTMLInputElement).value),
    });
  }

  function handleEventTriggerTypeInput(index: number, event: Event) {
    const triggerType = (event.currentTarget as HTMLSelectElement).value as EventTriggerType;
    updateEventMarker(
      index,
      {
        triggerType,
        poseX: eventPoseX(eventMarkers[index]),
        poseY: eventPoseY(eventMarkers[index]),
      },
      true,
    );
  }

  function handleEventTriggerMsInput(index: number, event: Event) {
    updateEventMarker(index, {
      triggerMs: Number((event.currentTarget as HTMLInputElement).value),
    });
  }

  function handleEventPoseXInput(index: number, event: Event) {
    updateEventMarker(index, {
      poseX: Number((event.currentTarget as HTMLInputElement).value),
    });
  }

  function handleEventPoseYInput(index: number, event: Event) {
    updateEventMarker(index, {
      poseY: Number((event.currentTarget as HTMLInputElement).value),
    });
  }

  function removeEventMarker(index: number) {
    setEventMarkers(
      eventMarkers.filter((_, markerIndex) => markerIndex !== index),
      true,
    );
  }

</script>

<div class="flex flex-col w-full justify-start items-start gap-1 rounded-md p-1">
  <div class="flex flex-row w-full items-center gap-3 flex-wrap">
    <div class="flex flex-row items-center gap-2">
      <button
        on:click={toggleCollapsed}
        class="flex items-center gap-2 font-semibold px-2 py-1 rounded transition-colors duration-250"
        title="{collapsed ? 'Expand' : 'Collapse'} path"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          stroke-width={2}
          stroke="currentColor"
          class="size-4 transition-transform {collapsed
            ? 'rotate-0'
            : 'rotate-90'}"
        >
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="m8.25 4.5 7.5 7.5-7.5 7.5"
          />
        </svg>
        Path {idx + 1}
      </button>

      <input
        bind:value={line.name}
        placeholder="Path {idx + 1}"
        class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none text-sm font-semibold"
        disabled={line.locked}
        on:input={() => {
          // Force parent reactivity so other components (like exporters)
          // pick up the updated name immediately.
          lines = [...lines];
        }}
        on:blur={() => {
          // Commit the change for history/undo
          lines = [...lines];
          if (recordChange) recordChange();
        }}
      />

      <select
        value={selectedChainId}
        on:change={handleChainSelect}
        class="px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-100 dark:bg-neutral-900"
        title="Assign path chain"
      >
        {#each chainOptions as chain}
          <option value={chain.id}>{chain.name}</option>
        {/each}
      </select>

      <div
        class="relative size-5 rounded-full overflow-hidden shadow-sm border border-neutral-300 dark:border-neutral-600 shrink-0"
        style="background-color: {line.color}"
      >
        <div class="absolute inset-0" title="Color comes from assigned path chain" />
      </div>

      <!-- Lock/Unlock Button -->
      <button
        title={line.locked ? "Unlock Path" : "Lock Path"}
        on:click|stopPropagation={() => {
          line.locked = !line.locked;
          lines = [...lines]; // Force reactivity
        }}
        class="p-1 rounded transition-colors duration-250"
      >
        {#if line.locked}
          <svg
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
            stroke-width={2}
            stroke="currentColor"
            class="size-5 stroke-yellow-500"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="M16.5 10.5V6.75a4.5 4.5 0 1 0-9 0v3.75m-.75 11.25h10.5a2.25 2.25 0 0 0 2.25-2.25v-6.75a2.25 2.25 0 0 0-2.25-2.25H6.75a2.25 2.25 0 0 0-2.25 2.25v6.75a2.25 2.25 0 0 0 2.25 2.25Z"
            />
          </svg>
        {:else}
          <svg
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
            stroke-width={2}
            stroke="currentColor"
            class="size-5 stroke-gray-400"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="M13.5 10.5V6.75a4.5 4.5 0 1 1 9 0v3.75M3.75 21.75h10.5a2.25 2.25 0 0 0 2.25-2.25v-6.75a2.25 2.25 0 0 0-2.25-2.25H3.75a2.25 2.25 0 0 0-2.25 2.25v6.75a2.25 2.25 0 0 0 2.25 2.25Z"
            />
          </svg>
        {/if}
      </button>

      <div class="flex flex-row gap-0.5 ml-1">
        <button
          title={line.locked ? "Path locked" : "Move up"}
          on:click|stopPropagation={() => {
            if (!line.locked && onMoveUp) onMoveUp();
          }}
          class="p-1 rounded-full text-neutral-500 dark:text-neutral-400 hover:text-neutral-700 dark:hover:text-neutral-200 bg-neutral-100/70 dark:bg-neutral-900/70 border border-neutral-200/70 dark:border-neutral-700/70 disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={!canMoveUp || line.locked}
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            class="size-4"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="m5 15 7-7 7 7"
            />
          </svg>
        </button>
        <button
          title={line.locked ? "Path locked" : "Move down"}
          on:click|stopPropagation={() => {
            if (!line.locked && onMoveDown) onMoveDown();
          }}
          class="p-1 rounded-full text-neutral-500 dark:text-neutral-400 hover:text-neutral-700 dark:hover:text-neutral-200 bg-neutral-100/70 dark:bg-neutral-900/70 border border-neutral-200/70 dark:border-neutral-700/70 disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={!canMoveDown || line.locked}
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            class="size-4"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="m19 9-7 7-7-7"
            />
          </svg>
        </button>
      </div>
    </div>

    <div class="flex flex-row items-center gap-1">
      <button
        class="px-2 py-1 text-xs font-semibold text-neutral-700 dark:text-neutral-200 bg-neutral-200/80 dark:bg-neutral-800/80 border border-neutral-300 dark:border-neutral-700 rounded disabled:opacity-40 disabled:cursor-not-allowed"
        title={line.locked ? "Path locked" : "Optimize this path"}
        on:click={() => line.id && optimizeLine && optimizeLine(line.id)}
        disabled={!line.id || line.locked || optimizing}
      >
        {optimizing ? "Optimizing…" : "Optimize"}
      </button>
    </div>

    <div class="flex flex-row items-center gap-1 ml-auto">
      <button
        title="Add control point after this line"
        on:click={onInsertAfter}
        class="text-blue-500 hover:text-blue-600 disabled:opacity-40 disabled:cursor-not-allowed"
        disabled={line.locked}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          stroke-width={2}
          class="size-5 stroke-green-500"
        >
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 4.5v15m7.5-7.5h-15"
          />
        </svg>
      </button>

      <!-- Insert Midpoint Between This and Next Path (dark-blue plus icon) -->
      <button
        title="Insert point between this path and the next"
        on:click={() => onInsertMidpoint && onInsertMidpoint()}
        class="text-blue-700 hover:text-blue-500"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          stroke-width={2}
          stroke="currentColor"
          class="size-5"
        >
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M5 8h4m6 0h4m-9 0 1.75-2.5M12 6l1.25 2.5"
          />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M5 16h4m6 0h4m-9 0 1.75 2.5M12 18l1.25-2.5"
          />
          <circle cx="12" cy="12" r="2.1" />
        </svg>
      </button>

      <!-- Add Wait After Button -->
      <button
        title="Add Wait After"
        on:click={onAddWaitAfter}
        class="text-[#E1461B] hover:text-orange-600"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          stroke-width="2"
          class="size-5"
        >
          <circle cx="12" cy="12" r="9" />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 7v5l3 2"
          />
        </svg>
      </button>

      <button
        title="Add shoot event after"
        on:click={onAddEventAfter}
        class="text-purple-500 hover:text-purple-600"
        disabled={line.locked}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          stroke-width="2"
          class="size-5"
        >
          <circle cx="12" cy="12" r="8" />
          <circle cx="12" cy="12" r="2" />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 2v4M12 18v4M2 12h4M18 12h4"
          />
        </svg>
      </button>

      {#if lines.length > 1}
        <button title="Remove Line" on:click={onRemove}>
          <svg
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
            stroke-width={2}
            class="size-5 stroke-red-500"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="M15 12H9m12 0a9 9 0 1 1-18 0 9 9 0 0 1 18 0Z"
            />
          </svg>
        </button>
      {/if}
    </div>
  </div>

  {#if !collapsed}
    <div class={`h-[0.75px] w-full`} style={`background: ${line.color}`} />

    <div class="flex flex-col justify-start items-start w-full">
      <div class="font-light">Path Speed:</div>
      <div class="flex flex-row justify-start items-center gap-2 mb-2 w-full max-w-md">
        <input
          type="range"
          min="0.05"
          max="1"
          step="0.05"
          value={pathSpeedValue}
          on:input={handlePathSpeedInput}
          on:change={commitPathSpeed}
          disabled={line.locked}
          class="flex-1"
        />
        <input
          type="number"
          min="0.05"
          max="1"
          step="0.05"
          value={pathSpeedValue}
          on:input={handlePathSpeedInput}
          on:blur={commitPathSpeed}
          disabled={line.locked}
          class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-20"
        />
      </div>

      <div class="font-light">Point Position:</div>
      <div class="flex flex-row justify-start items-center gap-2 flex-wrap">
        <div class="font-extralight">Pose:</div>
        <select
          value={line.endPoint.poseVariableId || ""}
          on:change={handlePoseVariableSelect}
          class="px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-100 dark:bg-neutral-900"
          disabled={line.locked}
        >
          <option value="">Custom</option>
          {#each poseVariables as variable (variable.id)}
            <option value={variable.id}>{variable.name || "Unnamed Pose"}</option>
          {/each}
        </select>

        <div class="font-extralight">X:</div>
        <input
          class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-28"
          step={$snapToGrid && $showGrid ? $gridSize : 0.1}
          type="number"
          min="0"
          max="141.5"
          bind:value={line.endPoint.x}
          disabled={line.locked || isEndPointBoundToPoseVariable}
          title={snapToGridTitle}
        />
        <div class="font-extralight">Y:</div>
        <input
          class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-28"
          step={$snapToGrid && $showGrid ? $gridSize : 0.1}
          min="0"
          max="141.5"
          type="number"
          bind:value={line.endPoint.y}
          disabled={line.locked || isEndPointBoundToPoseVariable}
          title={snapToGridTitle}
        />

        <HeadingControls
          endPoint={line.endPoint}
          locked={line.locked}
          boundEndHeading={boundPoseVariableHeading}
          onHeadingModeChange={(mode) => onHeadingModeChange(line.id || "", mode)}
          on:change={() => {
            // Force reactivity so timeline recalculates immediately
            lines = [...lines];
          }}
          on:commit={() => {
            // Commit change to history
            lines = [...lines];
            recordChange();
          }}
        />
      </div>

      {#if line.endPoint.heading === "linear"}
        <div class="mt-2 w-full">
          <HeadingCurveEditor
            endPoint={line.endPoint}
            locked={line.locked}
            on:change={() => {
              lines = [...lines];
            }}
            on:commit={() => {
              lines = [...lines];
              recordChange();
            }}
          />
        </div>
      {/if}

      <div class="mt-3 w-full border-t border-neutral-200 dark:border-neutral-800 pt-2">
        <div class="flex flex-row items-center gap-2 mb-2">
          <div class="font-light">Parallel Events:</div>
          <button
            title={line.locked ? "Path locked" : "Add parallel event"}
            on:click={addEventMarker}
            disabled={line.locked}
            class="p-1 rounded text-purple-600 hover:text-purple-700 disabled:opacity-40 disabled:cursor-not-allowed"
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              stroke-width="2"
              class="size-5"
            >
              <path stroke-linecap="round" stroke-linejoin="round" d="M12 5v14M5 12h14" />
            </svg>
          </button>
        </div>

        {#if eventMarkers.length > 0}
          <div class="flex flex-col gap-2 w-full">
            {#each eventMarkers as marker, eventIdx (marker.id || `${line.id}-event-${eventIdx}`)}
              <div
                class="flex flex-row items-center gap-2 flex-wrap w-full rounded border border-neutral-200 dark:border-neutral-800 bg-neutral-50 dark:bg-neutral-950 p-2"
              >
                <div class="font-extralight">Name:</div>
                <input
                  class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-900 dark:border-neutral-700 border-[0.5px] focus:outline-none w-36"
                  value={marker.name}
                  disabled={line.locked}
                  on:input={(event) => handleEventNameInput(eventIdx, event)}
                  on:blur={() => updateEventMarker(eventIdx, {}, true)}
                />

                <div class="font-extralight">Type:</div>
                <select
                  value={eventTriggerType(marker)}
                  disabled={line.locked}
                  on:change={(event) => handleEventTriggerTypeInput(eventIdx, event)}
                  class="px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-100 dark:bg-neutral-900"
                >
                  <option value="parametric">Path %</option>
                  <option value="temporal">Time</option>
                  <option value="pose">Pose</option>
                </select>

                {#if eventTriggerType(marker) === "temporal"}
                  <div class="font-extralight">After ms:</div>
                  <input
                    type="number"
                    min="0"
                    step="50"
                    value={clampEventDuration(marker.triggerMs)}
                    disabled={line.locked}
                    on:input={(event) => handleEventTriggerMsInput(eventIdx, event)}
                    on:blur={() => updateEventMarker(eventIdx, {}, true)}
                    class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-900 dark:border-neutral-700 border-[0.5px] focus:outline-none w-24"
                  />
                {:else}
                  <div class="font-extralight">
                    {eventTriggerType(marker) === "pose" ? "Guess:" : "Trigger:"}
                  </div>
                  <input
                    type="range"
                    min="0"
                    max="1"
                    step="0.01"
                    value={clampEventPosition(marker.position)}
                    disabled={line.locked}
                    on:input={(event) => handleEventPositionInput(eventIdx, event)}
                    on:change={() => updateEventMarker(eventIdx, {}, true)}
                    class="w-32"
                  />
                  <input
                    type="number"
                    min="0"
                    max="100"
                    step="1"
                    value={Math.round(clampEventPosition(marker.position) * 100)}
                    disabled={line.locked}
                    on:input={(event) => handleEventPercentInput(eventIdx, event)}
                    on:blur={() => updateEventMarker(eventIdx, {}, true)}
                    class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-900 dark:border-neutral-700 border-[0.5px] focus:outline-none w-16"
                  />
                  <div class="font-extralight">%</div>

                  {#if eventTriggerType(marker) === "pose"}
                    <div class="font-extralight">X:</div>
                    <input
                      type="number"
                      step={$snapToGrid && $showGrid ? $gridSize : 0.1}
                      value={eventPoseX(marker)}
                      disabled={line.locked}
                      on:input={(event) => handleEventPoseXInput(eventIdx, event)}
                      on:blur={() => updateEventMarker(eventIdx, {}, true)}
                      class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-900 dark:border-neutral-700 border-[0.5px] focus:outline-none w-20"
                    />
                    <div class="font-extralight">Y:</div>
                    <input
                      type="number"
                      step={$snapToGrid && $showGrid ? $gridSize : 0.1}
                      value={eventPoseY(marker)}
                      disabled={line.locked}
                      on:input={(event) => handleEventPoseYInput(eventIdx, event)}
                      on:blur={() => updateEventMarker(eventIdx, {}, true)}
                      class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-900 dark:border-neutral-700 border-[0.5px] focus:outline-none w-20"
                    />
                  {/if}
                {/if}

                <div class="font-extralight">Duration ms:</div>
                <input
                  type="number"
                  min="0"
                  step="50"
                  value={clampEventDuration(marker.durationMs)}
                  disabled={line.locked}
                  title="0 keeps the event active until auto end"
                  on:input={(event) => handleEventDurationInput(eventIdx, event)}
                  on:blur={() => updateEventMarker(eventIdx, {}, true)}
                  class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-900 dark:border-neutral-700 border-[0.5px] focus:outline-none w-24"
                />

                <button
                  title={line.locked ? "Path locked" : "Remove parallel event"}
                  on:click={() => removeEventMarker(eventIdx)}
                  disabled={line.locked}
                  class="ml-auto p-1 rounded text-red-500 hover:text-red-600 disabled:opacity-40 disabled:cursor-not-allowed"
                >
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    stroke-width="2"
                    class="size-5"
                  >
                    <path stroke-linecap="round" stroke-linejoin="round" d="M6 12h12" />
                  </svg>
                </button>
              </div>
            {/each}
          </div>
        {/if}
      </div>
    </div>

    <ControlPointsSection
      bind:line
      lineIdx={idx}
      bind:collapsed={collapsedControlPoints}
      onAddControlPoint={onInsertAfter}
      {recordChange}
    />
  {/if}
</div>

<style>
  @keyframes rainbow-glow {
    0% {
      background-position: 0% 50%;
    }
    50% {
      background-position: 100% 50%;
    }
    100% {
      background-position: 0% 50%;
    }
  }

</style>
