<script lang="ts">
  import type { Variable } from "../../types";
  import {
    buildExpressionScope,
    evaluateExpression,
    variableInsertOptions,
  } from "../../utils";
  import { DEFAULT_SCRUB_STEP, scrubbable } from "../../utils/scrub";

  /**
   * Text field that accepts either a literal or an expression referencing
   * variables. Shows an "insert variable" menu, the evaluated result, and
   * flags expressions that cannot be resolved.
   */

  export let value = "";
  export let variables: Variable[] = [];
  export let placeholder = "";
  export let disabled = false;
  export let label = "";
  export let title = "";
  /** "number" shows the numeric result, "boolean" shows true/false. */
  export let kind: "number" | "boolean" = "number";
  export let compact = false;
  export let onInput: (value: string) => void = () => {};
  export let onCommit: (value: string) => void = () => {};
  /**
   * How much one pixel of sideways drag is worth. Coordinates want inches,
   * headings want degrees, so callers that are neither can set their own.
   */
  export let scrubStep = DEFAULT_SCRUB_STEP;
  /** Turns off drag-to-scrub for fields where a number makes no sense. */
  export let allowScrub = true;

  let inputElement: HTMLInputElement;
  let menuOpen = false;
  let filter = "";

  $: scope = buildExpressionScope(variables);
  $: options = variableInsertOptions(variables);
  $: filteredOptions = filter.trim()
    ? options.filter((option) =>
        option.label.toLowerCase().includes(filter.trim().toLowerCase()),
      )
    : options;

  // A bare literal needs no evaluation feedback; only expressions do.
  $: isLiteral = /^\s*-?\d*\.?\d*\s*$/.test(value || "");
  $: evaluated =
    !value?.trim() || isLiteral
      ? null
      : evaluateExpression(value, variables, scope);
  $: invalid = Boolean(value?.trim()) && !isLiteral && evaluated === null;
  $: preview =
    evaluated === null
      ? ""
      : kind === "boolean"
        ? String(Boolean(evaluated))
        : typeof evaluated === "boolean"
          ? String(evaluated)
          : String(Number(evaluated.toFixed(4)));

  function handleInput(event: Event) {
    const next = (event.currentTarget as HTMLInputElement).value;
    value = next;
    onInput(next);
  }

  function handleBlur() {
    onCommit(value);
  }

  function insertToken(token: string) {
    const element = inputElement;
    const start = element?.selectionStart ?? value.length;
    const end = element?.selectionEnd ?? value.length;
    const next = `${value.slice(0, start)}${token}${value.slice(end)}`;

    value = next;
    menuOpen = false;
    filter = "";
    onInput(next);
    onCommit(next);

    // Restore focus with the caret after the inserted token.
    queueMicrotask(() => {
      element?.focus();
      const caret = start + token.length;
      element?.setSelectionRange(caret, caret);
    });
  }

  function closeMenu() {
    menuOpen = false;
    filter = "";
  }

  // The drag lives on the label, not on the box: an input has to tell a click
  // for the caret apart from the start of a drag, and a stray few pixels while
  // clicking would nudge the value. A label has no other job.
  $: scrubOptions = {
    value,
    step: scrubStep,
    disabled: disabled || !allowScrub || kind !== "number",
    onInput: (next: string) => {
      value = next;
      onInput(next);
    },
    onCommit: (next: string) => onCommit(next),
  };
</script>

<svelte:window on:click={closeMenu} />

<div class="flex flex-col gap-0.5 min-w-0 w-full">
  <div class="flex items-center gap-1 min-w-0">
    {#if label}
      <span
        use:scrubbable={scrubOptions}
        class="text-xs text-neutral-500 dark:text-neutral-300 shrink-0"
      >
        {label}
      </span>
    {/if}

    <div class="relative flex-1 min-w-0">
      <input
        bind:this={inputElement}
        type="text"
        {placeholder}
        {disabled}
        {title}
        {value}
        on:input={handleInput}
        on:blur={handleBlur}
        class="w-full {compact ? 'px-1.5 py-0.5' : 'px-2 py-1'} pr-6 text-xs rounded border bg-neutral-50 dark:bg-neutral-900 text-neutral-900 dark:text-neutral-100 disabled:opacity-50 {invalid
          ? 'border-rose-400 dark:border-rose-500'
          : 'border-neutral-300 dark:border-neutral-600'}"
      />

      <button
        type="button"
        {disabled}
        title="Insert variable"
        aria-label="Insert variable"
        on:click|stopPropagation={() => (menuOpen = !menuOpen)}
        class="absolute right-0.5 top-1/2 -translate-y-1/2 px-1 text-xs text-neutral-400 hover:text-sky-500 disabled:opacity-40"
      >
        ƒ
      </button>

      {#if menuOpen}
        <div
          role="menu"
          tabindex="-1"
          on:click|stopPropagation
          on:keydown|stopPropagation
          class="absolute right-0 top-full z-30 mt-1 w-56 max-h-64 overflow-y-auto rounded border border-neutral-300 dark:border-neutral-600 bg-white dark:bg-neutral-900 shadow-lg p-1"
        >
          <input
            type="text"
            bind:value={filter}
            placeholder="Filter variables…"
            class="w-full mb-1 px-2 py-1 text-xs rounded border border-neutral-200 dark:border-neutral-700 bg-neutral-50 dark:bg-neutral-950"
          />

          {#if filteredOptions.length === 0}
            <p class="px-2 py-1 text-xs text-neutral-500 dark:text-neutral-400">
              No variables available.
            </p>
          {:else}
            {#each filteredOptions as option (option.token)}
              <button
                type="button"
                on:click={() => insertToken(option.token)}
                class="w-full flex items-center justify-between gap-2 px-2 py-1 text-xs text-left rounded hover:bg-sky-50 dark:hover:bg-sky-900/40"
              >
                <span class="truncate text-neutral-800 dark:text-neutral-100">
                  {option.label}
                </span>
                <span class="shrink-0 text-[10px] uppercase text-neutral-400">
                  {option.type}
                </span>
              </button>
            {/each}
          {/if}
        </div>
      {/if}
    </div>
  </div>

  {#if invalid}
    <span class="text-[10px] text-rose-500 dark:text-rose-400">
      Unresolved expression
    </span>
  {:else if preview}
    <span class="text-[10px] text-neutral-400 dark:text-neutral-500">
      = {preview}
    </span>
  {/if}
</div>
