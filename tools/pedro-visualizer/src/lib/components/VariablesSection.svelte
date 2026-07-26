<script lang="ts">
  import type { Variable, VariableType } from "../../types";
  import {
    VARIABLE_TYPES,
    VARIABLE_TYPE_LABELS,
    booleanExpressionDisplayValue,
    convertVariableType,
    expressionDisplayValue,
  } from "../../utils";
  import ExpressionInput from "./ExpressionInput.svelte";

  /**
   * One section for every variable kind. The per-row type dropdown replaces
   * the old separate Pose / Number / Path variable panels.
   */

  export let variables: Variable[] = [];
  export let canStoreChain = false;
  export let onAdd: (type: VariableType) => void = () => {};
  export let onRemove: (id: string) => void = () => {};
  export let onDuplicate: (id: string) => void = () => {};
  export let onChange: (variable: Variable) => void = () => {};
  export let onCommit: () => void = () => {};
  export let onStoreChain: (id?: string) => void = () => {};
  export let onInsertPath: (id: string) => void = () => {};

  let newType: VariableType = "number";
  let filter = "";

  const inputValue = (event: Event) =>
    (event.currentTarget as HTMLInputElement).value;
  const inputChecked = (event: Event) =>
    (event.currentTarget as HTMLInputElement).checked;
  const selectValue = (event: Event) =>
    (event.currentTarget as HTMLSelectElement).value as VariableType;

  $: filteredVariables = filter.trim()
    ? variables.filter((variable) =>
        variable.name.toLowerCase().includes(filter.trim().toLowerCase()),
      )
    : variables;

  function update(id: string, patch: Record<string, unknown>) {
    const current = variables.find((variable) => variable.id === id);
    if (!current) return;

    const next = { ...current, ...patch } as Variable;
    variables = variables.map((variable) =>
      variable.id === id ? next : variable,
    );
    onChange(next);
  }

  function changeType(variable: Variable, type: VariableType) {
    if (variable.type === type) return;
    const next = convertVariableType(variable, type);
    variables = variables.map((item) => (item.id === variable.id ? next : item));
    onChange(next);
    onCommit();
  }

  /**
   * Numeric fields keep both the literal and the expression: the literal is the
   * fallback used when the expression cannot be resolved.
   */
  function updateNumericField(
    id: string,
    field: string,
    raw: string,
  ) {
    const trimmed = raw.trim();
    const numeric = Number(trimmed);
    const patch: Record<string, unknown> = {
      [`${field}Expression`]: trimmed ? raw : undefined,
    };
    if (Number.isFinite(numeric) && trimmed) patch[field] = numeric;
    update(id, patch);
  }

  function updateBooleanExpression(id: string, raw: string) {
    const trimmed = raw.trim().toLowerCase();
    const patch: Record<string, unknown> = {
      valueExpression: raw.trim() ? raw : undefined,
    };
    if (trimmed === "true" || trimmed === "false") {
      patch.value = trimmed === "true";
      patch.valueExpression = undefined;
    }
    update(id, patch);
  }

  function handleNameBlur(variable: Variable, index: number) {
    const nextName =
      variable.name.trim() || `${VARIABLE_TYPE_LABELS[variable.type]} ${index + 1}`;
    if (nextName !== variable.name) update(variable.id, { name: nextName });
    onCommit();
  }
</script>

<div
  class="w-full rounded-md border border-neutral-200 dark:border-neutral-700 p-3 bg-white dark:bg-neutral-800"
>
  <div class="flex flex-wrap items-center gap-2 mb-2">
    <p
      class="text-xs font-semibold uppercase tracking-wide text-neutral-500 dark:text-neutral-300"
    >
      Variables
    </p>

    <div class="flex items-center gap-1 ml-auto">
      <select
        bind:value={newType}
        class="px-1.5 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900"
        aria-label="New variable type"
      >
        {#each VARIABLE_TYPES as type}
          <option value={type}>{VARIABLE_TYPE_LABELS[type]}</option>
        {/each}
      </select>

      <button
        on:click={() =>
          newType === "path" ? onStoreChain() : onAdd(newType)}
        disabled={newType === "path" && !canStoreChain}
        title={newType === "path"
          ? "Store the current route as a reusable path variable"
          : "Add a variable"}
        class="px-2 py-1 text-xs rounded bg-emerald-100 text-emerald-700 dark:bg-emerald-900 dark:text-emerald-200 disabled:opacity-40"
      >
        {newType === "path" ? "Store Route" : "New"}
      </button>
    </div>
  </div>

  {#if variables.length > 4}
    <input
      type="text"
      bind:value={filter}
      placeholder="Filter variables…"
      class="w-full mb-2 px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900"
    />
  {/if}

  {#if variables.length === 0}
    <p class="text-xs text-neutral-500 dark:text-neutral-400">
      Reusable values you can reference by name from any numeric or boolean
      field — for example <code>depth - 10</code> or
      <code>isRed ? 90 : -90</code>.
    </p>
  {:else}
    <div class="flex flex-col gap-2">
      {#each filteredVariables as variable, index (variable.id)}
        <div
          class="rounded border border-neutral-200 dark:border-neutral-700 bg-neutral-50 dark:bg-neutral-900 p-2"
        >
          <div class="flex flex-wrap items-center gap-2">
            <input
              type="text"
              value={variable.name}
              on:input={(event) =>
                update(variable.id, { name: inputValue(event) })}
              on:blur={() => handleNameBlur(variable, index)}
              placeholder="Name"
              title="Reference this name inside expressions"
              class="min-w-0 flex-1 px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-white dark:bg-neutral-950"
            />

            <select
              value={variable.type}
              on:change={(event) => changeType(variable, selectValue(event))}
              class="px-1.5 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-white dark:bg-neutral-950"
              aria-label="Variable type"
            >
              {#each VARIABLE_TYPES as type}
                <option value={type}>{VARIABLE_TYPE_LABELS[type]}</option>
              {/each}
            </select>
          </div>

          <div class="mt-2">
            {#if variable.type === "number"}
              <ExpressionInput
                compact
                {variables}
                label="="
                placeholder="0 or depth - 10"
                value={expressionDisplayValue(
                  variable.valueExpression,
                  variable.value,
                )}
                onInput={(raw) => updateNumericField(variable.id, "value", raw)}
                onCommit={onCommit}
              />
            {:else if variable.type === "boolean"}
              <div class="flex items-center gap-2">
                <label
                  class="flex items-center gap-1 text-xs text-neutral-500 dark:text-neutral-300"
                >
                  <input
                    type="checkbox"
                    checked={variable.value}
                    disabled={Boolean(variable.valueExpression?.trim())}
                    on:change={(event) =>
                      update(variable.id, { value: inputChecked(event) })}
                    on:blur={onCommit}
                  />
                  {variable.value ? "true" : "false"}
                </label>

                <div class="flex-1 min-w-0">
                  <ExpressionInput
                    compact
                    kind="boolean"
                    {variables}
                    placeholder="true, or depth > 10 && isRed"
                    value={booleanExpressionDisplayValue(
                      variable.valueExpression,
                      variable.value,
                    )}
                    onInput={(raw) => updateBooleanExpression(variable.id, raw)}
                    onCommit={onCommit}
                  />
                </div>
              </div>
            {:else if variable.type === "pose"}
              <div class="grid grid-cols-3 gap-2">
                <ExpressionInput
                  compact
                  {variables}
                  label="X"
                  placeholder="x"
                  value={expressionDisplayValue(
                    variable.xExpression,
                    variable.x,
                  )}
                  onInput={(raw) => updateNumericField(variable.id, "x", raw)}
                  onCommit={onCommit}
                />
                <ExpressionInput
                  compact
                  {variables}
                  label="Y"
                  placeholder="y"
                  value={expressionDisplayValue(
                    variable.yExpression,
                    variable.y,
                  )}
                  onInput={(raw) => updateNumericField(variable.id, "y", raw)}
                  onCommit={onCommit}
                />
                <ExpressionInput
                  compact
                  scrubStep={0.25}
                  {variables}
                  label="H"
                  placeholder="deg"
                  value={expressionDisplayValue(
                    variable.headingExpression,
                    variable.heading,
                  )}
                  onInput={(raw) =>
                    updateNumericField(variable.id, "heading", raw)}
                  onCommit={onCommit}
                />
              </div>
            {:else}
              <p class="text-xs text-neutral-500 dark:text-neutral-400">
                {variable.lines.length} path{variable.lines.length === 1
                  ? ""
                  : "s"} stored
              </p>
            {/if}
          </div>

          <div class="mt-2 flex flex-wrap items-center gap-2">
            {#if variable.type === "path"}
              <button
                on:click={() => onInsertPath(variable.id)}
                class="px-2 py-1 text-xs rounded bg-emerald-100 text-emerald-700 dark:bg-emerald-900 dark:text-emerald-200"
              >
                Insert Copy
              </button>
              <button
                on:click={() => onStoreChain(variable.id)}
                disabled={!canStoreChain}
                title="Replace with the current route"
                class="px-2 py-1 text-xs rounded bg-cyan-100 text-cyan-700 dark:bg-cyan-900 dark:text-cyan-200 disabled:opacity-40"
              >
                Update
              </button>
            {/if}
            <button
              on:click={() => onDuplicate(variable.id)}
              class="px-2 py-1 text-xs rounded bg-indigo-100 text-indigo-700 dark:bg-indigo-900 dark:text-indigo-200"
            >
              Duplicate
            </button>
            <button
              on:click={() => onRemove(variable.id)}
              class="px-2 py-1 text-xs rounded bg-rose-100 text-rose-700 dark:bg-rose-900 dark:text-rose-200"
            >
              Remove
            </button>
          </div>
        </div>
      {/each}
    </div>
  {/if}
</div>
