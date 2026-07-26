<script lang="ts">
  import type { Point, PoseVariable, Variable } from "../../types";
  import {
    expressionDisplayValue,
    pointCoordinateFieldDisplayValue,
    resolvePointExpressions,
  } from "../../utils";
  import ExpressionInput from "./ExpressionInput.svelte";
  import { scrubbable } from "../../utils/scrub";

  export let startPoint: Point;
  export let poseVariables: PoseVariable[] = [];
  export let variables: Variable[] = [];
  export let recordChange: () => void = () => {};
  export let onPoseVariableChange: (poseVariableId: string) => void = () => {};
  export let addPathAtStart: () => void;
  export let addWaitAtStart: () => void;
  export let addEventAtStart: () => void;
  /**
   * The heading a tangential start point works out to, i.e. the direction of the
   * first path. Only used to show a truthful number before the user pins one
   * down; `null` when the start point already stores its own heading.
   */
  export let tangentialHeading: number | null = null;

  /**
   * How close the robot is to something solid where it is staged. Null when the
   * start pose is clear.
   */
  export let clearance: { hit: boolean; clearance: number; target: string } | null = null;
  export let onFixClearance: () => void = () => {};
  export let fixingClearance = false;
  export let clearanceFixNote = "";

  $: isBoundToPoseVariable = Boolean(startPoint.poseVariableId);

  // Same rule as the path endpoints: a locked point was pinned on purpose, and
  // one driven by a pose variable takes its position from the variable.
  $: canFixStartClearance = !startPoint.locked && !isBoundToPoseVariable;

  $: startClearanceLabel = !clearance
    ? ""
    : clearance.hit
      ? `Starts in ${clearance.target}`
      : `${clearance.clearance.toFixed(1)}in clearance`;

  $: startClearanceTitle = !clearance
    ? ""
    : clearance.hit
      ? `The robot is staged overlapping ${clearance.target} by ${Math.abs(clearance.clearance).toFixed(2)}in.`
      : `The robot is staged within ${clearance.clearance.toFixed(2)}in of ${clearance.target}.`;

  /**
   * The start point is a pose, not an interpolation: nothing travels "over" it,
   * so it has one heading regardless of which mode the point happens to store.
   * Read it from whichever field that mode uses.
   */
  $: startHeadingValue =
    startPoint.heading === "constant"
      ? (Number(startPoint.degrees) || 0)
      : startPoint.heading === "linear"
        ? (Number(startPoint.startDeg) || 0)
        : (tangentialHeading ?? 0);

  $: startHeadingExpression =
    startPoint.heading === "constant"
      ? startPoint.degreesExpression
      : startPoint.heading === "linear"
        ? startPoint.startDegExpression
        : undefined;

  $: startHeadingTitle = isBoundToPoseVariable
    ? "The starting heading comes from the selected pose"
    : startPoint.heading === "tangential"
      ? "Starting heading in degrees. It currently follows the first path; entering a value pins it down."
      : "The heading the robot is placed at in degrees (literal or expression)";

  function handlePoseVariableSelect(event: Event) {
    onPoseVariableChange((event.currentTarget as HTMLSelectElement).value);
  }

  function updateCoordinate(field: "x" | "y", value: string) {
    const expressionField = `${field}Expression` as "xExpression" | "yExpression";
    const numeric = Number(value);
    startPoint = resolvePointExpressions(
      {
        ...startPoint,
        [field]: Number.isFinite(numeric) ? numeric : startPoint[field],
        [expressionField]: value.trim() ? value : undefined,
      },
      variables,
    );
  }

  function updateHeading(value: string) {
    const numeric = Number(value);
    const expression = value.trim() ? value : undefined;
    const degrees = Number.isFinite(numeric) ? numeric : startHeadingValue;

    if (startPoint.heading === "linear") {
      startPoint = resolvePointExpressions(
        {
          ...startPoint,
          startDeg: degrees,
          startDegExpression: expression,
        },
        variables,
      );
      return;
    }

    if (startPoint.heading === "constant") {
      startPoint = resolvePointExpressions(
        {
          ...startPoint,
          degrees,
          degreesExpression: expression,
        },
        variables,
      );
      return;
    }

    // A tangential start point stores no heading of its own, so entering one
    // turns it into a definite heading. `reverse` has to go with it: the point
    // type only allows it on the tangential variant.
    const { reverse, reverseExpression, ...rest } = startPoint;
    startPoint = resolvePointExpressions(
      {
        ...rest,
        heading: "constant",
        degrees,
        degreesExpression: expression,
      },
      variables,
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

    <!--
      The robot can be staged inside an obstacle, which is a problem no path can
      fix: it is already there before it drives anywhere.
    -->
    <div class="flex items-center gap-1">
      {#if clearance}
        {#if canFixStartClearance}
          <button
            on:click|stopPropagation={onFixClearance}
            disabled={fixingClearance}
            class="rounded-full px-2 py-0.5 text-[10px] font-semibold disabled:opacity-60 {clearance.hit
              ? 'bg-red-100 dark:bg-red-900/60 text-red-700 dark:text-red-200 hover:bg-red-200 dark:hover:bg-red-900'
              : 'bg-amber-100 dark:bg-amber-900/60 text-amber-700 dark:text-amber-200 hover:bg-amber-200 dark:hover:bg-amber-900'}"
            title="{startClearanceTitle} Click to move the starting point on X and Y until the robot clears."
          >
            {fixingClearance ? "Fixing…" : `${startClearanceLabel} — fix collision issues`}
          </button>
        {:else}
          <span
            class="rounded-full px-2 py-0.5 text-[10px] font-semibold {clearance.hit
              ? 'bg-red-100 dark:bg-red-900/60 text-red-700 dark:text-red-200'
              : 'bg-amber-100 dark:bg-amber-900/60 text-amber-700 dark:text-amber-200'}"
            title={startClearanceTitle}
          >
            {startClearanceLabel}
          </span>
        {/if}
      {/if}

      <!--
        Outside the chip's own condition: a fix that works clears the chip, and
        the note is the only thing left saying what happened.
      -->
      {#if clearanceFixNote}
        <span
          class="rounded-full bg-neutral-200 dark:bg-neutral-800 px-2 py-0.5 text-[10px] font-semibold text-neutral-600 dark:text-neutral-300"
          title={clearanceFixNote}
        >
          {clearanceFixNote}
        </span>
      {/if}
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
    <span
      class="font-extralight"
      use:scrubbable={{
        value: pointCoordinateFieldDisplayValue(startPoint, "x"),
        disabled: startPoint.locked || isBoundToPoseVariable,
        onInput: (raw) => updateCoordinate("x", raw),
        onCommit: () => recordChange?.(),
      }}
    >X:</span>
    <div class="w-28">
      <ExpressionInput
        compact
        {variables}
        value={pointCoordinateFieldDisplayValue(startPoint, "x")}
        onInput={(raw) => updateCoordinate("x", raw)}
        onCommit={() => recordChange?.()}
        disabled={startPoint.locked || isBoundToPoseVariable}
      />
    </div>
    <span
      class="font-extralight"
      use:scrubbable={{
        value: pointCoordinateFieldDisplayValue(startPoint, "y"),
        disabled: startPoint.locked || isBoundToPoseVariable,
        onInput: (raw) => updateCoordinate("y", raw),
        onCommit: () => recordChange?.(),
      }}
    >Y:</span>
    <div class="w-28">
      <ExpressionInput
        compact
        {variables}
        value={pointCoordinateFieldDisplayValue(startPoint, "y")}
        onInput={(raw) => updateCoordinate("y", raw)}
        onCommit={() => recordChange?.()}
        disabled={startPoint.locked || isBoundToPoseVariable}
      />
    </div>
    <span
      class="font-extralight"
      title={startHeadingTitle}
      use:scrubbable={{
        value: expressionDisplayValue(startHeadingExpression, startHeadingValue),
        step: 0.25,
        disabled: startPoint.locked || isBoundToPoseVariable,
        onInput: updateHeading,
        onCommit: () => recordChange?.(),
      }}
    >Heading:</span>
    <div class="w-28">
      <ExpressionInput
        scrubStep={0.25}
        compact
        {variables}
        title={startHeadingTitle}
        placeholder="deg"
        value={expressionDisplayValue(startHeadingExpression, startHeadingValue)}
        onInput={updateHeading}
        onCommit={() => recordChange?.()}
        disabled={startPoint.locked || isBoundToPoseVariable}
      />
    </div>
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
