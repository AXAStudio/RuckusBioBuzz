<script lang="ts">
  import type { Variable } from "../../types";
  import { expressionDisplayValue } from "../../utils";
  import ExpressionInput from "./ExpressionInput.svelte";

  export let name: string;
  export let durationMs: number;
  export let durationExpression: string | undefined = undefined;
  export let enabledExpression: string | undefined = undefined;
  export let variables: Variable[] = [];
  export let locked: boolean = false;
  export let onToggleLock: () => void;
  export let onChange: (newName: string, newDuration: number) => void;
  export let onDurationExpressionChange: (raw: string) => void = () => {};
  export let onEnabledExpressionChange: (raw: string) => void = () => {};
  export let onCommit: () => void = () => {};
  export let onRemove: () => void;
  export let onInsertAfter: () => void;
  export let onAddPathAfter: () => void;
  export let onAddEventAfter: () => void;
  export let onMoveUp: () => void;
  export let onMoveDown: () => void;
  export let canMoveUp: boolean = true;
  export let canMoveDown: boolean = true;
  export let label: "Wait" | "Event" = "Wait";
  /** Drag handle wiring: lets this step be dropped into a loop or if block. */
  export let onPointerDown: (event: PointerEvent) => void = () => {};
  export let dragging = false;

  function handleNameChange(e: Event) {
    const target = e.currentTarget as HTMLInputElement;
    if (!locked) onChange(target?.value ?? "", durationMs);
  }

  function handleDurationInput(raw: string) {
    if (locked) return;

    // A bare number stays a literal; anything else becomes an expression.
    const trimmed = raw.trim();
    const numeric = Number(trimmed);
    if (trimmed && Number.isFinite(numeric) && !/[a-zA-Z]/.test(trimmed)) {
      onDurationExpressionChange("");
      onChange(name, Math.max(0, numeric));
      return;
    }

    onDurationExpressionChange(raw);
  }
</script>

<div
  class="flex w-full items-center justify-between gap-2 px-2 py-1 rounded border border-neutral-200 dark:border-neutral-700 bg-neutral-100 dark:bg-neutral-900"
>
  <div class="flex items-center gap-2">
    <!-- Drag handle: drop this step into any repeat loop or if block -->
    <span
      role="button"
      tabindex={locked ? -1 : 0}
      draggable="false"
      on:pointerdown={onPointerDown}
      on:dragstart|preventDefault
      title={locked
        ? `Locked ${label.toLowerCase()}s cannot be dragged`
        : "Drag into a repeat loop or if block"}
      aria-label="Drag {label.toLowerCase()}"
      class="shrink-0 select-none touch-none rounded px-1 py-0.5 text-neutral-400 hover:text-neutral-700 dark:hover:text-neutral-200 hover:bg-neutral-200/70 dark:hover:bg-neutral-800 {locked
        ? 'cursor-not-allowed opacity-40'
        : dragging
          ? 'cursor-grabbing'
          : 'cursor-grab'}"
    >
      <svg
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        fill="currentColor"
        class="size-4 pointer-events-none"
      >
        <circle cx="9" cy="6" r="1.6" />
        <circle cx="15" cy="6" r="1.6" />
        <circle cx="9" cy="12" r="1.6" />
        <circle cx="15" cy="12" r="1.6" />
        <circle cx="9" cy="18" r="1.6" />
        <circle cx="15" cy="18" r="1.6" />
      </svg>
    </span>

    <span
      class={label === "Event"
        ? "px-1.5 py-0.5 text-xs rounded bg-purple-200 text-purple-800 dark:bg-purple-900 dark:text-purple-200"
        : "px-1.5 py-0.5 text-xs rounded bg-amber-200 text-amber-800 dark:bg-amber-900 dark:text-amber-200"}
      >{label}</span
    >
    <input
      class="pl-1.5 rounded-md bg-neutral-50 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none w-40"
      type="text"
      placeholder="Name"
      list={label === "Event" ? "event-presets" : undefined}
      bind:value={name}
      on:change={handleNameChange}
      disabled={locked}
    />
    {#if label === "Event"}
      <datalist id="event-presets">
        <option value="Shoot" />
        <option value="Intake" />
        <option value="Intake On" />
        <option value="Intake Off" />
        <option value="Arm Up" />
        <option value="Arm Down" />
      </datalist>
    {/if}
    <div class="w-32">
      <ExpressionInput
        compact
        {variables}
        disabled={locked}
        placeholder="0"
        title="{label} duration in ms (literal or expression)"
        value={expressionDisplayValue(durationExpression, durationMs)}
        onInput={handleDurationInput}
        onCommit={onCommit}
      />
    </div>
    <span>ms</span>
    <div class="w-32" title="Optional boolean expression; when false this step is skipped">
      <ExpressionInput
        compact
        kind="boolean"
        {variables}
        disabled={locked}
        placeholder="if…"
        value={enabledExpression || ""}
        onInput={onEnabledExpressionChange}
        onCommit={onCommit}
      />
    </div>
  </div>

  <div class="flex items-center gap-2">
    <!-- Lock/Unlock Button -->
    <button
      title={locked ? "Unlock Wait" : "Lock Wait"}
      on:click|stopPropagation={() => {
        if (onToggleLock) onToggleLock();
      }}
      class="p-1 rounded transition-colors duration-250"
    >
      {#if locked}
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

    <div class="flex flex-row gap-0.5 mr-1">
      <button
        title="Move up"
        on:click={() => {
          if (!locked && onMoveUp) onMoveUp();
        }}
        class="p-1 rounded-full text-neutral-500 dark:text-neutral-400 hover:text-neutral-700 dark:hover:text-neutral-200 bg-neutral-100/70 dark:bg-neutral-900/70 border border-neutral-200/70 dark:border-neutral-700/70 disabled:opacity-40 disabled:cursor-not-allowed"
        disabled={!canMoveUp || locked}
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
        title="Move down"
        on:click={() => {
          if (!locked && onMoveDown) onMoveDown();
        }}
        class="p-1 rounded-full text-neutral-500 dark:text-neutral-400 hover:text-neutral-700 dark:hover:text-neutral-200 bg-neutral-100/70 dark:bg-neutral-900/70 border border-neutral-200/70 dark:border-neutral-700/70 disabled:opacity-40 disabled:cursor-not-allowed"
        disabled={!canMoveDown || locked}
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

    <button
      title="Add path after"
      on:click={() => {
        if (!locked && onAddPathAfter) onAddPathAfter();
      }}
      class="text-green-500 hover:text-green-600"
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
          d="M12 4.5v15m7.5-7.5h-15"
        />
      </svg>
    </button>

    <button
      title="Add wait after"
      on:click={() => {
        if (!locked && onInsertAfter) onInsertAfter();
      }}
      class="text-amber-500 hover:text-amber-600"
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
        <path stroke-linecap="round" stroke-linejoin="round" d="M12 7v5l3 2" />
      </svg>
    </button>

    <button
      title="Add shoot event after"
      on:click={() => {
        if (!locked && onAddEventAfter) onAddEventAfter();
      }}
      class="text-purple-500 hover:text-purple-600"
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
        <path stroke-linecap="round" stroke-linejoin="round" d="M12 2v4M12 18v4M2 12h4M18 12h4" />
      </svg>
    </button>

    <button
      title="Remove"
      on:click={() => {
        if (!locked && onRemove) onRemove();
      }}
      class="text-red-500 hover:text-red-600"
    >
      <svg
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        class="size-5"
      >
        <path
          stroke-linecap="round"
          stroke-linejoin="round"
          d="M15 12H9m12 0a9 9 0 1 1-18 0 9 9 0 0 1 18 0Z"
        />
      </svg>
    </button>
  </div>
</div>
