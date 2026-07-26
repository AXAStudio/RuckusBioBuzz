<script lang="ts">
  import type { Shape, Variable } from "../../types";
  import {
    createTriangle,
    pointCoordinateFieldDisplayValue,
    resolveBasePointExpressions,
  } from "../../utils";
  import { snapToGrid, showGrid, gridSize } from "../../stores";
  import ExpressionInput from "./ExpressionInput.svelte";
  import { scrubbable } from "../../utils/scrub";

  const colorChoices = [
    { label: "Red", color: "#dc2626", fill: "#ff6b6b" },
    { label: "Blue", color: "#2563eb", fill: "#60a5fa" },
  ];

  function setPresetColor(shape: Shape, color: string) {
    const choice = colorChoices.find((c) => c.color === color);
    if (!choice) return;

    // Reassign so the field repaints: mutating the shape in place left the
    // canvas showing the old colour until some other change came along.
    shapes = shapes.map((item) =>
      item === shape
        ? { ...item, color: choice.color, fillColor: choice.fill }
        : item,
    );
    recordChange?.();
  }

  export let shapes: Shape[];
  export let collapsedObstacles: boolean[];
  export let variables: Variable[] = [];
  export let recordChange: () => void = () => {};

  function updateVertex(
    shapeIdx: number,
    vertexIdx: number,
    field: "x" | "y",
    value: string,
  ) {
    const expressionField = `${field}Expression` as "xExpression" | "yExpression";
    const numeric = Number(value);

    shapes = shapes.map((shape, index) => {
      if (index !== shapeIdx) return shape;
      return {
        ...shape,
        vertices: shape.vertices.map((vertex, idx) =>
          idx === vertexIdx
            ? resolveBasePointExpressions(
                {
                  ...vertex,
                  [field]: Number.isFinite(numeric) ? numeric : vertex[field],
                  [expressionField]: value.trim() ? value : undefined,
                },
                variables,
              )
            : vertex,
        ),
      };
    });
  }

  $: snapToGridTitle =
    $snapToGrid && $showGrid ? `Snapping to ${$gridSize} grid` : "No snapping";

  function toggleObstacle(index: number) {
    collapsedObstacles[index] = !collapsedObstacles[index];
    collapsedObstacles = [...collapsedObstacles]; // Force reactivity
  }

  function toggleAllObstacles() {
    const allCollapsed = collapsedObstacles.every((c) => c);
    collapsedObstacles = collapsedObstacles.map(() => !allCollapsed);
  }
</script>

<div class="flex flex-col w-full justify-start items-start gap-0.5 text-sm">
  <div class="flex items-center gap-2 w-full">
    <button
      on:click={toggleAllObstacles}
      class="flex items-center gap-2 font-semibold px-2 py-1 rounded transition-colors duration-250"
      title="{collapsedObstacles.every((c) => c)
        ? 'Expand all'
        : 'Collapse all'} obstacles"
    >
      <svg
        xmlns="http://www.w3.org/2000/svg"
        fill="none"
        viewBox="0 0 24 24"
        stroke-width={2}
        stroke="currentColor"
        class="size-4 transition-transform {collapsedObstacles.every((c) => c)
          ? 'rotate-0'
          : 'rotate-90'}"
      >
        <path
          stroke-linecap="round"
          stroke-linejoin="round"
          d="m8.25 4.5 7.5 7.5-7.5 7.5"
        />
      </svg>
      Obstacles ({shapes.length})
    </button>
  </div>

  {#each shapes as shape, shapeIdx}
    <div
      class="flex flex-col w-full justify-start items-start gap-1 p-2 border rounded-md border-neutral-300 dark:border-neutral-600 mt-2"
    >
      <div class="flex flex-row w-full justify-between items-center">
        <div class="flex flex-row items-center gap-2">
          <button
            on:click={() => toggleObstacle(shapeIdx)}
            class="flex items-center gap-2 font-medium text-sm px-2 py-1 rounded transition-colors duration-250"
            title="{collapsedObstacles[shapeIdx]
              ? 'Expand'
              : 'Collapse'} obstacle"
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              fill="none"
              viewBox="0 0 24 24"
              stroke-width={2}
              stroke="currentColor"
              class="size-4 transition-transform {collapsedObstacles[shapeIdx]
                ? 'rotate-0'
                : 'rotate-90'}"
            >
              <path
                stroke-linecap="round"
                stroke-linejoin="round"
                d="m8.25 4.5 7.5 7.5-7.5 7.5"
              />
            </svg>
            Obstacle {shapeIdx + 1}
          </button>

          <input
            bind:value={shape.name}
            placeholder="Obstacle {shapeIdx + 1}"
            class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none text-sm font-medium"
          />
          <select
            class="rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] px-2 py-1 text-sm font-medium"
            bind:value={shape.color}
            on:change={(e) => setPresetColor(shape, e.currentTarget.value)}
          >
            {#each colorChoices as c}
              <option value={c.color}>{c.label}</option>
            {/each}
          </select>
        </div>

        <div class="flex flex-row gap-1">
          <button
            title="Add Vertex"
            on:click={() => {
              shape.vertices = [...shape.vertices, { x: 50, y: 50 }];
            }}
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              fill="none"
              viewBox="0 0 24 24"
              stroke-width={2}
              class="size-4 stroke-green-500"
            >
              <path
                stroke-linecap="round"
                stroke-linejoin="round"
                d="M12 4.5v15m7.5-7.5h-15"
              />
            </svg>
          </button>
          {#if shapes.length > 0}
            <button
              title="Remove Shape"
              on:click={() => {
                shapes.splice(shapeIdx, 1);
                shapes = shapes;
                // Also remove the collapsed state for this obstacle
                collapsedObstacles.splice(shapeIdx, 1);
                collapsedObstacles = [...collapsedObstacles];
              }}
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                fill="none"
                viewBox="0 0 24 24"
                stroke-width={2}
                class="size-4 stroke-red-500"
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

      {#if !collapsedObstacles[shapeIdx]}
        {#each shape.vertices as vertex, vertexIdx}
          <div class="flex flex-row justify-start items-center gap-2">
            <div class="font-bold text-sm">{vertexIdx + 1}:</div>
            <div
              class="font-extralight text-sm"
              use:scrubbable={{
                value: pointCoordinateFieldDisplayValue(vertex, "x"),
                onInput: (raw) => updateVertex(shapeIdx, vertexIdx, "x", raw),
                onCommit: () => recordChange?.(),
              }}
            >X:</div>
            <div class="w-24">
              <ExpressionInput
                compact
                {variables}
                title={snapToGridTitle}
                value={pointCoordinateFieldDisplayValue(vertex, "x")}
                onInput={(raw) => updateVertex(shapeIdx, vertexIdx, "x", raw)}
                onCommit={() => recordChange?.()}
              />
            </div>
            <div
              class="font-extralight text-sm"
              use:scrubbable={{
                value: pointCoordinateFieldDisplayValue(vertex, "y"),
                onInput: (raw) => updateVertex(shapeIdx, vertexIdx, "y", raw),
                onCommit: () => recordChange?.(),
              }}
            >Y:</div>
            <div class="w-24">
              <ExpressionInput
                compact
                {variables}
                title={snapToGridTitle}
                value={pointCoordinateFieldDisplayValue(vertex, "y")}
                onInput={(raw) => updateVertex(shapeIdx, vertexIdx, "y", raw)}
                onCommit={() => recordChange?.()}
              />
            </div>
            {#if $snapToGrid && $showGrid}
              <span class="text-xs text-green-500" title="Snapping enabled"
                >✓</span
              >
            {/if}
            {#if shape.vertices.length > 3}
              <button
                title="Remove Vertex"
                on:click={() => {
                  shape.vertices.splice(vertexIdx, 1);
                  shape.vertices = shape.vertices;
                }}
              >
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  fill="none"
                  viewBox="0 0 24 24"
                  stroke-width={2}
                  class="size-4 stroke-red-500"
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
        {/each}
      {/if}
    </div>
  {/each}

  <button
    on:click={() => {
      shapes = [...shapes, createTriangle(shapes.length)];
      // Add a new collapsed state for the new obstacle (default to collapsed)
      collapsedObstacles = [...collapsedObstacles, true];
    }}
    class="font-semibold text-red-500 text-sm flex flex-row justify-start items-center gap-1 mt-2"
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
    <p>Add Obstacle</p>
  </button>
</div>
