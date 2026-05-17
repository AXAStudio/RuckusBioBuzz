<script lang="ts">
  import type { PoseVariable } from "../../types";

  export let poseVariables: PoseVariable[] = [];
  export let onAdd: () => void = () => {};
  export let onRemove: (id: string) => void = () => {};
  export let onChange: (variable: PoseVariable) => void = () => {};
  export let onCommit: () => void = () => {};

  function numericValue(value: string): number {
    const next = Number(value);
    return Number.isFinite(next) ? next : 0;
  }

  function updateVariable(
    index: number,
    patch: Partial<Omit<PoseVariable, "id">>,
  ) {
    const current = poseVariables[index];
    if (!current) return;

    const next: PoseVariable = {
      ...current,
      ...patch,
    };

    poseVariables = poseVariables.map((variable, variableIndex) =>
      variableIndex === index ? next : variable,
    );
    onChange(next);
  }

  function updateName(index: number, event: Event) {
    updateVariable(index, {
      name: (event.currentTarget as HTMLInputElement).value,
    });
  }

  function updateNumber(
    index: number,
    field: "x" | "y" | "heading",
    event: Event,
  ) {
    updateVariable(index, {
      [field]: numericValue((event.currentTarget as HTMLInputElement).value),
    });
  }

  function handleNameBlur(index: number) {
    const current = poseVariables[index];
    if (!current) return;

    const nextName = current.name.trim() || `Pose ${index + 1}`;
    if (nextName !== current.name) {
      updateVariable(index, { name: nextName });
    }
    onCommit();
  }
</script>

<div class="w-full rounded-md border border-neutral-200 dark:border-neutral-700 p-3 bg-white dark:bg-neutral-800">
  <div class="flex items-center justify-between gap-2 mb-2">
    <p class="text-xs font-semibold uppercase tracking-wide text-neutral-500 dark:text-neutral-300">
      Pose Variables
    </p>
    <button
      on:click={onAdd}
      class="px-2 py-1 text-xs rounded bg-sky-100 text-sky-700 dark:bg-sky-900 dark:text-sky-200"
    >
      Add Pose
    </button>
  </div>

  {#if poseVariables.length === 0}
    <p class="text-xs text-neutral-500 dark:text-neutral-400">No variables yet.</p>
  {:else}
    <div class="flex flex-col gap-2">
      {#each poseVariables as variable, index (variable.id)}
        <div class="grid grid-cols-[minmax(8rem,1.2fr)_repeat(3,minmax(4.5rem,0.7fr))_auto] gap-2 items-center">
          <input
            type="text"
            value={variable.name}
            on:input={(event) => updateName(index, event)}
            on:blur={() => handleNameBlur(index)}
            class="px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900"
            placeholder="Pose name"
          />

          <label class="flex items-center gap-1 text-xs text-neutral-500 dark:text-neutral-300">
            X
            <input
              type="number"
              value={variable.x}
              min="0"
              max="141.5"
              step="0.1"
              on:input={(event) => updateNumber(index, "x", event)}
              on:blur={onCommit}
              class="w-full px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900 text-neutral-900 dark:text-neutral-100"
            />
          </label>

          <label class="flex items-center gap-1 text-xs text-neutral-500 dark:text-neutral-300">
            Y
            <input
              type="number"
              value={variable.y}
              min="0"
              max="141.5"
              step="0.1"
              on:input={(event) => updateNumber(index, "y", event)}
              on:blur={onCommit}
              class="w-full px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900 text-neutral-900 dark:text-neutral-100"
            />
          </label>

          <label class="flex items-center gap-1 text-xs text-neutral-500 dark:text-neutral-300">
            H
            <input
              type="number"
              value={variable.heading}
              step="0.1"
              on:input={(event) => updateNumber(index, "heading", event)}
              on:blur={onCommit}
              class="w-full px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900 text-neutral-900 dark:text-neutral-100"
            />
          </label>

          <button
            on:click={() => onRemove(variable.id)}
            class="px-2 py-1 text-xs rounded bg-rose-100 text-rose-700 dark:bg-rose-900 dark:text-rose-200"
          >
            Remove
          </button>
        </div>
      {/each}
    </div>
  {/if}
</div>
