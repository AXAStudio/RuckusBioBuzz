<script lang="ts">
  import { createEventDispatcher } from "svelte";
  import type { Variable } from "../../types";
  import {
    pointHeadingFieldDisplayValue,
    resolveBooleanExpression,
    resolvePointExpressions,
  } from "../../utils";
  import ExpressionInput from "./ExpressionInput.svelte";
  import { scrubbable } from "../../utils/scrub";
  export let endPoint: any;
  export let variables: Variable[] = [];
  export let locked: boolean = false;
  export let boundEndHeading: number | null = null;
  export let onHeadingModeChange: (mode: string) => void = () => {};
  const dispatch = createEventDispatcher();

  $: poseTargetHeading =
    boundEndHeading !== null && Number.isFinite(Number(boundEndHeading))
      ? Number(boundEndHeading)
      : null;

  // `endPoint` is bound by the parent, so every write has to be an assignment:
  // mutating a field would update the object but never tell the parent (or the
  // canvas) that anything changed.
  $: if (
    poseTargetHeading !== null &&
    endPoint.heading === "constant" &&
    endPoint.degrees !== poseTargetHeading
  ) {
    endPoint = { ...endPoint, degrees: poseTargetHeading };
  }

  $: if (
    poseTargetHeading !== null &&
    endPoint.heading === "linear" &&
    endPoint.endDeg !== poseTargetHeading
  ) {
    endPoint = { ...endPoint, endDeg: poseTargetHeading };
  }

  function handleConstantInput(value: string) {
    const numeric = Number(value);
    endPoint = resolvePointExpressions(
      {
        ...endPoint,
        degrees: Number.isFinite(numeric) ? numeric : endPoint.degrees ?? 0,
        degreesExpression: value.trim() ? value : undefined,
      },
      variables,
    );
    dispatch("change");
  }

  function handleLinearInput(field: "startDeg" | "endDeg", value: string) {
    const expressionField = `${field}Expression` as
      | "startDegExpression"
      | "endDegExpression";
    const numeric = Number(value);
    endPoint = resolvePointExpressions(
      {
        ...endPoint,
        [field]: Number.isFinite(numeric) ? numeric : endPoint[field] ?? 0,
        [expressionField]: value.trim() ? value : undefined,
      },
      variables,
    );
    dispatch("change");
  }

  /** Reverse can be driven by a boolean expression instead of the checkbox. */
  function handleReverseExpressionInput(raw: string) {
    const expression = raw.trim() ? raw : undefined;
    endPoint = {
      ...endPoint,
      reverseExpression: expression,
      reverse: expression
        ? resolveBooleanExpression(expression, endPoint.reverse ?? false, variables)
            .value
        : endPoint.reverse,
    };
    dispatch("change");
  }

  /** Assigning (not mutating) keeps the parent's `bind:endPoint` in the loop. */
  function handleReverseToggle(event: Event) {
    endPoint = {
      ...endPoint,
      reverse: (event.currentTarget as HTMLInputElement).checked,
    };
    dispatch("change");
  }

  function handleExpressionBlur() {
    dispatch("commit");
  }

  function handleHeadingModeChange(event: Event) {
    onHeadingModeChange((event.currentTarget as HTMLSelectElement).value);
  }
</script>

<select
  value={endPoint.heading}
  on:change={handleHeadingModeChange}
  class=" rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-28 text-sm"
  title="The heading style of the robot. 
With constant heading, the robot maintains the same heading throughout the line. 
With linear heading, heading changes linearly between given start and end angles. 
With tangential heading, the heading follows the direction of the line."
  disabled={locked}
>
  <option value="constant">Constant</option>
  <option value="linear">Linear</option>
  <option value="tangential">Tangential</option>
</select>

{#if endPoint.heading === "linear"}
  <div class="flex items-center gap-1">
    <span
      class="text-xs text-neutral-600 dark:text-neutral-400"
      use:scrubbable={{
        value: pointHeadingFieldDisplayValue(endPoint, "startDeg"),
        step: 0.25,
        disabled: locked,
        onInput: (raw) => handleLinearInput("startDeg", raw),
        onCommit: handleExpressionBlur,
      }}
    >Start:</span>
    <div class="w-24">
      <ExpressionInput
        scrubStep={0.25}
        compact
        {variables}
        value={pointHeadingFieldDisplayValue(endPoint, "startDeg")}
        onInput={(raw) => handleLinearInput("startDeg", raw)}
        onCommit={handleExpressionBlur}
        title="The heading the robot starts this line at (in degrees)"
        disabled={locked}
      />
    </div>
    <span
      class="text-xs text-neutral-600 dark:text-neutral-400 ml-1"
      use:scrubbable={{
        value: pointHeadingFieldDisplayValue(endPoint, "endDeg"),
        step: 0.25,
        disabled: locked || poseTargetHeading !== null,
        onInput: (raw) => handleLinearInput("endDeg", raw),
        onCommit: handleExpressionBlur,
      }}
    >End:</span>
    <div class="w-24">
      <ExpressionInput
        scrubStep={0.25}
        compact
        {variables}
        value={pointHeadingFieldDisplayValue(endPoint, "endDeg")}
        onInput={(raw) => handleLinearInput("endDeg", raw)}
        onCommit={handleExpressionBlur}
        title={poseTargetHeading !== null
          ? "The ending heading comes from the selected pose"
          : "The heading the robot ends this line at (in degrees)"}
        disabled={locked || poseTargetHeading !== null}
      />
    </div>
  </div>
{:else if endPoint.heading === "constant"}
  <div class="flex items-center gap-1">
    <span
      class="text-xs text-neutral-600 dark:text-neutral-400"
      use:scrubbable={{
        value: pointHeadingFieldDisplayValue(endPoint, "degrees"),
        step: 0.25,
        disabled: locked || poseTargetHeading !== null,
        onInput: handleConstantInput,
        onCommit: handleExpressionBlur,
      }}
    >Deg:</span>
    <div class="w-24">
      <ExpressionInput
        scrubStep={0.25}
        compact
        {variables}
        value={pointHeadingFieldDisplayValue(endPoint, "degrees")}
        onInput={handleConstantInput}
        onCommit={handleExpressionBlur}
        title={poseTargetHeading !== null
          ? "The heading comes from the selected pose"
          : "The constant heading the robot maintains throughout this line (in degrees)"}
        disabled={locked || poseTargetHeading !== null}
      />
    </div>
  </div>
{:else if endPoint.heading === "tangential"}
  <p class="text-sm font-extralight">Reverse:</p>
  <input
    type="checkbox"
    checked={Boolean(endPoint.reverse)}
    on:change={handleReverseToggle}
    on:blur={() => dispatch("commit")}
    title="Reverse the direction the robot faces along the tangential path"
    disabled={locked || Boolean(endPoint.reverseExpression)}
  />
  <div class="w-28">
    <ExpressionInput
      compact
      kind="boolean"
      {variables}
      placeholder="or expression"
      title="Boolean expression driving reverse, e.g. isRedAlliance"
      value={endPoint.reverseExpression || ""}
      onInput={handleReverseExpressionInput}
      onCommit={handleExpressionBlur}
      disabled={locked}
    />
  </div>
{/if}
