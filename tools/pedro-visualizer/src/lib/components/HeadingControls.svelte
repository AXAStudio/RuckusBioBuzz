<script lang="ts">
  import { createEventDispatcher } from "svelte";
  export let endPoint: any;
  export let locked: boolean = false;
  export let boundEndHeading: number | null = null;
  export let onHeadingModeChange: (mode: string) => void = () => {};
  const dispatch = createEventDispatcher();

  $: poseTargetHeading =
    boundEndHeading !== null && Number.isFinite(Number(boundEndHeading))
      ? Number(boundEndHeading)
      : null;

  $: if (poseTargetHeading !== null && endPoint.heading === "constant") {
    endPoint.degrees = poseTargetHeading;
  }

  $: if (poseTargetHeading !== null && endPoint.heading === "linear") {
    endPoint.endDeg = poseTargetHeading;
  }

  function handleConstantInput(event: Event) {
    const target = event.currentTarget as HTMLInputElement;
    const value = parseFloat(target.value);
    if (!isNaN(value)) {
      endPoint.degrees = value;
    } else {
      endPoint.degrees = 0;
      target.value = "0";
    }
    dispatch("change");
  }

  function handleConstantBlur(event: Event) {
    const target = event.currentTarget as HTMLInputElement;
    if (target.value === "" || isNaN(parseFloat(target.value))) {
      endPoint.degrees = 0;
      target.value = "0";
    }
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
    <span class="text-xs text-neutral-600 dark:text-neutral-400">Start:</span>
    <input
      class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-14"
      step="1"
      type="number"
      min="-180"
      max="180"
      bind:value={endPoint.startDeg}
      on:input={() => dispatch("change")}
      on:blur={() => dispatch("commit")}
      title="The heading the robot starts this line at (in degrees)"
      disabled={locked}
    />
    <span class="text-xs text-neutral-600 dark:text-neutral-400 ml-1">End:</span
    >
    <input
      class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-14"
      step="1"
      type="number"
      min="-180"
      max="180"
      bind:value={endPoint.endDeg}
      on:input={() => dispatch("change")}
      on:blur={() => dispatch("commit")}
      title={poseTargetHeading !== null
        ? "The ending heading comes from the selected pose"
        : "The heading the robot ends this line at (in degrees)"}
      disabled={locked || poseTargetHeading !== null}
    />
  </div>
{:else if endPoint.heading === "constant"}
  <div class="flex items-center gap-1">
    <span class="text-xs text-neutral-600 dark:text-neutral-400">Deg:</span>
    <input
      class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-14"
      step="1"
      type="number"
      min="-180"
      max="180"
      value={endPoint.degrees || 0}
      on:input={handleConstantInput}
      on:blur={handleConstantBlur}
      title={poseTargetHeading !== null
        ? "The heading comes from the selected pose"
        : "The constant heading the robot maintains throughout this line (in degrees)"}
      disabled={locked || poseTargetHeading !== null}
    />
  </div>
{:else if endPoint.heading === "tangential"}
  <p class="text-sm font-extralight">Reverse:</p>
  <input
    type="checkbox"
    bind:checked={endPoint.reverse}
    on:change={() => dispatch("change")}
    on:blur={() => dispatch("commit")}
    title="Reverse the direction the robot faces along the tangential path"
    disabled={locked}
  />
{/if}
