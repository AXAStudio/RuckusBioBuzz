<script lang="ts">
  import type { NumberVariable, Point, PoseVariable } from "../../types";
  import {
    pointCoordinateFieldDisplayValue,
    resolvePointExpressions,
  } from "../../utils";

  export let startPoint: Point;
  export let poseVariables: PoseVariable[] = [];
  export let numberVariables: NumberVariable[] = [];
  export let onPoseVariableChange: (poseVariableId: string) => void = () => {};
  export let addPathAtStart: () => void;
  export let addWaitAtStart: () => void;
  export let addEventAtStart: () => void;

  $: isBoundToPoseVariable = Boolean(startPoint.poseVariableId);

  function handlePoseVariableSelect(event: Event) {
    onPoseVariableChange((event.currentTarget as HTMLSelectElement).value);
  }

  function updateCoordinate(field: "x" | "y", event: Event) {
    const value = (event.currentTarget as HTMLInputElement).value;
    const expressionField = `${field}Expression` as "xExpression" | "yExpression";
    const numeric = Number(value);
    startPoint = resolvePointExpressions(
      {
        ...startPoint,
        [field]: Number.isFinite(numeric) ? numeric : startPoint[field],
        [expressionField]: value.trim() ? value : undefined,
      },
      numberVariables,
    );
  }
</script>

<div class="flex flex-col w-full justify-start items-start gap-0.5">
  <div class="flex items-center justify-between w-full">
    <div class="font-semibold flex items-center gap-2">
      Starting Point
      <button
        title={startPoint.locked
          ? "Unlock Starting Point"
          : "Lock Starting Point"}
        on:click|stopPropagation={() => {
          startPoint.locked = !startPoint.locked;
          startPoint = { ...startPoint }; // Force reactivity
        }}
        class="p-1 rounded transition-colors duration-250"
      >
        {#if startPoint.locked}
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
    </div>
  </div>
  <div class="flex flex-row justify-start items-center gap-2">
    <span class="font-extralight">Pose:</span>
    <select
      value={startPoint.poseVariableId || ""}
      on:change={handlePoseVariableSelect}
      class="px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-100 dark:bg-neutral-900"
      disabled={startPoint.locked}
    >
      <option value="">Custom</option>
      {#each poseVariables as variable (variable.id)}
        <option value={variable.id}>{variable.name || "Unnamed Pose"}</option>
      {/each}
    </select>
  </div>
  <div class="flex flex-row justify-start items-center gap-2">
    <span class="font-extralight">X:</span>
    <input
      value={pointCoordinateFieldDisplayValue(startPoint, "x")}
      type="text"
      class="pl-1.5 rounded-md bg-neutral-100 border-[0.5px] focus:outline-none w-28 dark:bg-neutral-950 dark:border-neutral-700"
      on:input={(event) => updateCoordinate("x", event)}
      disabled={startPoint.locked || isBoundToPoseVariable}
    />
    <span class="font-extralight">Y:</span>
    <input
      value={pointCoordinateFieldDisplayValue(startPoint, "y")}
      type="text"
      class="pl-1.5 rounded-md bg-neutral-100 border-[0.5px] focus:outline-none w-28 dark:bg-neutral-950 dark:border-neutral-700"
      on:input={(event) => updateCoordinate("y", event)}
      disabled={startPoint.locked || isBoundToPoseVariable}
    />
    <div class="flex items-center gap-2 ml-2">
      <button
        title="Add path at start"
        on:click={addPathAtStart}
        class="flex items-center gap-1 px-2 py-1 text-xs font-semibold rounded bg-green-100 text-green-700 dark:bg-green-900 dark:text-green-200"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          stroke-width={2}
          stroke="currentColor"
          class="size-4"
        >
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 4.5v15m7.5-7.5h-15"
          />
        </svg>
        Add Path
      </button>
      <button
        title="Add wait at start"
        on:click={addWaitAtStart}
        class="flex items-center gap-1 px-2 py-1 text-xs font-semibold rounded bg-amber-100 text-amber-700 dark:bg-amber-900 dark:text-amber-200"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          stroke-width="2"
          class="size-4"
        >
          <circle cx="12" cy="12" r="9" />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 7v5l3 2"
          />
        </svg>
        Add Wait
      </button>
      <button
        title="Add shoot event at start"
        on:click={addEventAtStart}
        class="flex items-center gap-1 px-2 py-1 text-xs font-semibold rounded bg-purple-100 text-purple-700 dark:bg-purple-900 dark:text-purple-200"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          stroke-width="2"
          class="size-4"
        >
          <circle cx="12" cy="12" r="8" />
          <circle cx="12" cy="12" r="2" />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 2v4M12 18v4M2 12h4M18 12h4"
          />
        </svg>
        Add Event
      </button>
    </div>
  </div>
</div>
