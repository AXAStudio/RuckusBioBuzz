<script lang="ts">
  import type {
    Point,
    Line,
    BasePoint,
    Settings,
    Shape,
    SequenceItem,
    PathChain,
    PoseVariable,
    PathVariable,
    NumberVariable,
  } from "../types";
  import _ from "lodash";
  import {
    calculatePathTime,
    buildEventTimingWindows,
    getRandomColor,
    mirrorPathData,
    resolveBasePointExpressions,
    resolvePointExpressions,
    resolvePoseVariableExpressions,
    type MirrorAxis,
  } from "../utils";
  import ObstaclesSection from "./components/ObstaclesSection.svelte";
  import TelemetryPanel from "./components/TelemetryPanel.svelte";
  import StartingPointSection from "./components/StartingPointSection.svelte";
  import PathLineSection from "./components/PathLineSection.svelte";
  import PoseVariablesSection from "./components/PoseVariablesSection.svelte";
  import PlaybackControls from "./components/PlaybackControls.svelte";
  import WaitRow from "./components/WaitRow.svelte";

  export let percent: number;
  export let playing: boolean;
  export let play: () => any;
  export let pause: () => any;
  export let startPoint: Point;
  export let lines: Line[];
  export let poseVariables: PoseVariable[] = [];
  export let pathVariables: PathVariable[] = [];
  export let numberVariables: NumberVariable[] = [];
  export let sequence: SequenceItem[];
  export let pathChains: PathChain[] = [];
  export let robotWidth: number = 16;
  export let robotHeight: number = 16;
  export let robotXY: BasePoint;
  export let robotHeading: number;
  export let x: d3.ScaleLinear<number, number, number>;
  export let y: d3.ScaleLinear<number, number, number>;
  export let settings: Settings;
  export let handleSeek: (percent: number) => void;
  export let loopAnimation: boolean;
  export let optimizeLine: (lineId: string, targetControlPointIndex?: number) => void;
  export let optimizingLineIds: Record<string, boolean> = {};

  export let shapes: Shape[];
  export let recordChange: () => void;

  const makeChainId = () =>
    `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
  const defaultChainName = "Main Chain";

  let selectedChainId = "";
  let chainNameDraft = "";
  let chainColorDraft = "#22c55e";
  let selectedChain: PathChain | null = null;
  let previousSelectedChainId = "";
  let chainOptions: Array<{ id: string; name: string; color: string }> = [];
  let draggedLineId = "";
  let dragOverRepeatId = "";

  const getChainById = (chainId: string): PathChain | null =>
    pathChains.find((chain) => chain.id === chainId) || null;

  function getLinePrimaryChainId(lineId: string): string {
    for (const chain of pathChains) {
      if ((chain.lineIds || []).includes(lineId)) return chain.id;
    }
    return pathChains[0]?.id || "";
  }

  function syncLineColorsToChains() {
    const chainColorById = new Map(pathChains.map((chain) => [chain.id, chain.color || "#22c55e"]));
    let changed = false;
    const nextLines = lines.map((line) => {
      const ownerId = getLinePrimaryChainId(line.id || "");
      const targetColor = chainColorById.get(ownerId) || line.color;
      if (line.color !== targetColor) {
        changed = true;
        return { ...line, color: targetColor };
      }
      return line;
    });
    if (changed) {
      lines = nextLines;
    }
  }

  function ensureDefaultChain() {
    if (pathChains.length === 0) {
      pathChains = [
        {
          id: makeChainId(),
          name: defaultChainName,
          color: getRandomColor(),
          lineIds: lines.map((ln) => ln.id!).filter(Boolean),
        },
      ];
      selectedChainId = pathChains[0]?.id || "";
      return;
    }

    if (!selectedChainId || !pathChains.some((c) => c.id === selectedChainId)) {
      selectedChainId = pathChains[0]?.id || "";
    }
  }

  $: {
    const normalized = pathChains.map((chain) => ({
      ...chain,
      color: chain.color || getRandomColor(),
      lineIds: chain.lineIds || [],
    }));
    if (JSON.stringify(normalized) !== JSON.stringify(pathChains)) {
      pathChains = normalized;
    }
  }

  $: ensureDefaultChain();

  $: selectedChain =
    pathChains.find((chain) => chain.id === selectedChainId) || pathChains[0] || null;

  $: if (selectedChainId !== previousSelectedChainId) {
    chainNameDraft = selectedChain?.name || "";
    chainColorDraft = selectedChain?.color || "#22c55e";
    previousSelectedChainId = selectedChainId;
  }

  function ensureLineInDefaultChain(lineId: string) {
    if (!lineId || !pathChains.length) return;
    assignLineToChain(lineId, pathChains[0].id);
  }

  function removeLineFromChains(lineId: string) {
    if (!lineId) return;
    const updated = pathChains.map((chain) => ({
        ...chain,
        lineIds: chain.lineIds.filter((id) => id !== lineId),
      }));
    pathChains = updated;
    ensureDefaultChain();
    syncLineColorsToChains();
  }

  function assignLineToChain(lineId: string, chainId: string) {
    if (!lineId || !chainId) return;
    pathChains = pathChains.map((chain) => ({
      ...chain,
      lineIds: chain.lineIds.filter((id) => id !== lineId),
    }));

    const target = getChainById(chainId);
    if (!target) return;

    pathChains = pathChains.map((chain) => {
      if (chain.id !== chainId) return chain;
      return {
        ...chain,
        lineIds: Array.from(new Set([...(chain.lineIds || []), lineId])),
      };
    });

    syncLineColorsToChains();
    recordChange?.();
  }

  function addPathChain() {
    const newChain: PathChain = {
      id: makeChainId(),
      name: `Chain ${pathChains.length + 1}`,
      color: getRandomColor(),
      lineIds: [],
    };
    pathChains = [...pathChains, newChain];
    selectedChainId = newChain.id;
    recordChange?.();
  }

  function duplicateSelectedPathChain() {
    if (!selectedChain) return;

    const sourceLineIds = selectedChain.lineIds || [];
    const selectedLineSet = new Set(sourceLineIds);
    const lineLookup = new Map(lines.map((line) => [line.id, line]));
    const idMap = new Map<string, string>();
    const clonedLines: Line[] = [];

    // Keep duplication order aligned with timeline, then append any non-sequenced lines.
    const orderedSourceIds: string[] = [];
    sequence.forEach((item) => {
      if (item.kind === "path" && selectedLineSet.has(item.lineId)) {
        orderedSourceIds.push(item.lineId);
      }
    });
    sourceLineIds.forEach((lineId) => {
      if (!orderedSourceIds.includes(lineId)) {
        orderedSourceIds.push(lineId);
      }
    });

    orderedSourceIds.forEach((sourceId, index) => {
      const sourceLine = lineLookup.get(sourceId);
      if (!sourceLine) return;
      const clone = JSON.parse(JSON.stringify(sourceLine)) as Line;
      const newLineId = makeId();
      clone.id = newLineId;
      clone.name = `${sourceLine.name || `Path ${lines.length + index + 1}`} Copy`;
      idMap.set(sourceId, newLineId);
      clonedLines.push(clone);
    });

    const newSequence: SequenceItem[] = [];
    sequence.forEach((item) => {
      newSequence.push(item);
      if (item.kind === "path") {
        const clonedId = idMap.get(item.lineId);
        if (clonedId) {
          newSequence.push({ kind: "path", lineId: clonedId });
        }
      }
    });

    // If chain contains lines currently not present in the timeline, append their clones.
    orderedSourceIds.forEach((sourceId) => {
      const inSequence = sequence.some((item) => item.kind === "path" && item.lineId === sourceId);
      const clonedId = idMap.get(sourceId);
      if (!inSequence && clonedId) {
        newSequence.push({ kind: "path", lineId: clonedId });
      }
    });

    lines = [...lines, ...clonedLines];
    sequence = newSequence;
    syncLinesToSequence(newSequence);

    const duplicateChain: PathChain = {
      id: makeChainId(),
      name: `${selectedChain.name} Copy`,
      color: getRandomColor(),
      lineIds: orderedSourceIds.map((sourceId) => idMap.get(sourceId)).filter(Boolean) as string[],
    };

    const selectedIndex = pathChains.findIndex((chain) => chain.id === selectedChain.id);
    if (selectedIndex >= 0) {
      pathChains = [
        ...pathChains.slice(0, selectedIndex + 1),
        duplicateChain,
        ...pathChains.slice(selectedIndex + 1),
      ];
    } else {
      pathChains = [...pathChains, duplicateChain];
    }

    selectedChainId = duplicateChain.id;
    syncLineColorsToChains();
    recordChange?.();
  }

  function removeSelectedPathChain() {
    if (!selectedChain || pathChains.length <= 1) return;
    const fallbackChainId = pathChains.find((chain) => chain.id !== selectedChain.id)?.id;
    const orphanedLines = [...(selectedChain.lineIds || [])];
    pathChains = pathChains.filter((chain) => chain.id !== selectedChain.id);

    if (fallbackChainId) {
      orphanedLines.forEach((lineId) => assignLineToChain(lineId, fallbackChainId));
    }

    selectedChainId = pathChains[0]?.id || "";
    syncLineColorsToChains();
    recordChange?.();
  }

  function updateSelectedChainName() {
    if (!selectedChain) return;
    const nextName = chainNameDraft.trim();
    if (!nextName) return;
    pathChains = pathChains.map((chain) =>
      chain.id === selectedChain.id ? { ...chain, name: nextName } : chain,
    );
    recordChange?.();
  }

  function updateSelectedChainColor() {
    if (!selectedChain) return;
    pathChains = pathChains.map((chain) =>
      chain.id === selectedChain.id ? { ...chain, color: chainColorDraft } : chain,
    );
    syncLineColorsToChains();
    recordChange?.();
  }

  $: chainOptions = pathChains.map((chain) => ({
    id: chain.id,
    name: chain.name,
    color: chain.color || "#22c55e",
  }));

  $: syncLineColorsToChains();

  // Reference exported but unused props to silence Svelte unused-export warnings

  $: robotWidth;
  $: robotHeight;

  function timelineEventColor(eventName: string, index: number) {
    const colors = ["#f97316", "#06b6d4", "#22c55e", "#e11d48", "#8b5cf6", "#f59e0b"];
    let hash = index;
    for (const char of eventName) {
      hash = (hash * 31 + char.charCodeAt(0)) % colors.length;
    }
    return colors[Math.abs(hash) % colors.length];
  }

  // Compute timeline markers for the UI (start of each travel segment)
  $: timePrediction = calculatePathTime(startPoint, lines, settings, sequence);
  $: markers = (() => {
    const _markers: {
      percent?: number;
      startPercent?: number;
      endPercent?: number;
      color: string;
      name: string;
      label?: string;
      type?: "marker" | "duration";
    }[] = [];
    if (
      !timePrediction ||
      !timePrediction.timeline ||
      timePrediction.totalTime <= 0
    )
      return _markers;

    const eventWindows = buildEventTimingWindows(
      startPoint,
      lines,
      timePrediction,
      settings,
      sequence,
    );

    timePrediction.timeline.forEach((ev) => {
      if ((ev as any).type === "travel") {
        const start = (ev as any).startTime as number;
        const end = (ev as any).endTime as number;
        const pct = (end / timePrediction.totalTime) * 100;
        const lineIndex = (ev as any).lineIndex as number;
        const line = lines[lineIndex];
        const color = line?.color || "#ffffff";
        const name = line?.name || `Path ${lineIndex + 1}`;
        _markers.push({ percent: pct, color, name });
      }
    });

    eventWindows.forEach((eventWindow, index) => {
      const eventColor = timelineEventColor(eventWindow.name, index + eventWindow.lineIndex * 7);
      _markers.push({
        percent: eventWindow.startPercent,
        color: eventColor,
        name: `${eventWindow.name} on ${eventWindow.lineName}`,
        type: "marker",
      });
      _markers.push({
        startPercent: eventWindow.startPercent,
        endPercent: eventWindow.endPercent,
        color: eventColor,
        name: `${eventWindow.name} on ${eventWindow.lineName}`,
        label:
          eventWindow.durationMs > 0
            ? `${eventWindow.name} ${(eventWindow.durationMs / 1000).toFixed(1)}s`
            : `${eventWindow.name} hold`,
        type: "duration",
      });
    });

    return _markers;
  })();


  // State for collapsed sections
  let collapsedSections = {
    obstacles: shapes.map(() => true),
    lines: lines.map(() => false),
    controlPoints: lines.map(() => true), // Start with control points collapsed
  };

  // Collapsed state for obstacles (default collapsed)
  let collapsedObstacles = shapes.map(() => true);

  // Reactive statements to update UI state when lines or shapes change from file load
  $: if (lines.length !== collapsedSections.lines.length) {
    collapsedSections = {
      obstacles: collapsedSections.obstacles ?? shapes.map(() => true),
      lines: lines.map(() => false),
      controlPoints: lines.map(() => true),
    };
  }

  // Keep obstacle collapse state aligned with shapes list
  $: if (shapes.length !== collapsedObstacles.length) {
    collapsedObstacles = shapes.map(() => true);
  }

  $: if (!collapsedSections.obstacles || shapes.length !== collapsedSections.obstacles.length) {
    collapsedSections = {
      ...collapsedSections,
      obstacles: shapes.map(() => true),
    };
  }

  const makeId = () =>
    `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;

  function cloneJson<T>(value: T): T {
    return JSON.parse(JSON.stringify(value));
  }

  function inputValue(event: Event): string {
    return (event.currentTarget as HTMLInputElement).value;
  }

  function inputNumber(event: Event): number {
    return Number(inputValue(event));
  }

  function makeNumberVariableName(): string {
    const usedNames = new Set(numberVariables.map((variable) => variable.name.trim()));
    let index = numberVariables.length + 1;
    let name = `Number ${index}`;

    while (usedNames.has(name)) {
      index++;
      name = `Number ${index}`;
    }

    return name;
  }

  function numberVariableValue(variableId: string | undefined, fallback: number): number {
    if (!variableId) return fallback;
    const variable = numberVariables.find((item) => item.id === variableId);
    const value = Number(variable?.value);
    return Number.isFinite(value) ? value : fallback;
  }

  function positionNumberValue(variableId: string | undefined, fallback: number): number {
    const value = numberVariableValue(variableId, fallback);
    return Math.max(0, Math.min(1, value > 1 ? value / 100 : value));
  }

  function addNumberVariable() {
    numberVariables = [
      ...numberVariables,
      {
        id: `number-${makeId()}`,
        name: makeNumberVariableName(),
        value: 1,
      },
    ];
    recordChange?.();
  }

  function updateNumberVariable(
    variableId: string,
    patch: Partial<NumberVariable>,
    commit = false,
  ) {
    let updatedVariable: NumberVariable | null = null;
    numberVariables = numberVariables.map((variable) => {
      if (variable.id !== variableId) return variable;
      const next = {
        ...variable,
        ...patch,
        value:
          patch.value === undefined
            ? variable.value
            : Number.isFinite(Number(patch.value))
              ? Number(patch.value)
              : variable.value,
      };
      updatedVariable = next;
      return next;
    });

    if (updatedVariable) {
      syncNumberVariableUsage(updatedVariable);
    }

    if (commit) recordChange?.();
  }

  function duplicateNumberVariable(variableId: string) {
    const variable = numberVariables.find((item) => item.id === variableId);
    if (!variable) return;

    numberVariables = [
      ...numberVariables,
      {
        ...cloneJson(variable),
        id: `number-${makeId()}`,
        name: `${variable.name || "Number"} Copy`,
      },
    ];
    recordChange?.();
  }

  function removeNumberVariable(variableId: string) {
    numberVariables = numberVariables.filter((variable) => variable.id !== variableId);
    lines = lines.map((line) => ({
      ...line,
      speedVariableId:
        line.speedVariableId === variableId ? undefined : line.speedVariableId,
      eventMarkers: (line.eventMarkers || []).map((marker) => ({
        ...marker,
        positionVariableId:
          marker.positionVariableId === variableId ? undefined : marker.positionVariableId,
        triggerMsVariableId:
          marker.triggerMsVariableId === variableId ? undefined : marker.triggerMsVariableId,
        poseXVariableId:
          marker.poseXVariableId === variableId ? undefined : marker.poseXVariableId,
        poseYVariableId:
          marker.poseYVariableId === variableId ? undefined : marker.poseYVariableId,
        durationVariableId:
          marker.durationVariableId === variableId ? undefined : marker.durationVariableId,
      })),
    }));
    sequence = sequence.map((item) => {
      if (item.kind === "repeat") {
        return {
          ...item,
          countVariableId:
            item.countVariableId === variableId ? undefined : item.countVariableId,
        };
      }
      if (item.kind === "wait" || item.kind === "event") {
        return {
          ...item,
          durationVariableId:
            item.durationVariableId === variableId ? undefined : item.durationVariableId,
        };
      }
      return item;
    });
    recordChange?.();
  }

  function syncNumberVariableUsage(variable: NumberVariable) {
    const value = Number(variable.value);
    if (!Number.isFinite(value)) return;

    poseVariables = resolvePoseVariableExpressions(poseVariables, numberVariables);
    poseVariables.forEach((poseVariable) => {
      syncPoseVariableUsage(poseVariable.id, poseVariable);
    });

    startPoint = resolvePointExpressions(startPoint, numberVariables);

    lines = lines.map((line) => ({
      ...line,
      endPoint:
        line.endPoint.poseVariableId
          ? line.endPoint
          : resolvePointExpressions(line.endPoint, numberVariables),
      controlPoints: (line.controlPoints || []).map((point) =>
        resolveBasePointExpressions(point, numberVariables),
      ),
      speed:
        line.speedVariableId === variable.id
          ? Math.max(0.05, Math.min(1, value))
          : line.speed,
      eventMarkers: (line.eventMarkers || []).map((marker) => ({
        ...marker,
        position:
          marker.positionVariableId === variable.id
            ? positionNumberValue(variable.id, marker.position)
            : marker.position,
        triggerMs:
          marker.triggerMsVariableId === variable.id
            ? Math.max(0, Math.round(value))
            : marker.triggerMs,
        poseX: marker.poseXVariableId === variable.id ? value : marker.poseX,
        poseY: marker.poseYVariableId === variable.id ? value : marker.poseY,
        durationMs:
          marker.durationVariableId === variable.id
            ? Math.max(0, Math.round(value))
            : marker.durationMs,
      })),
    }));

    sequence = sequence.map((item) => {
      if (item.kind === "repeat" && item.countVariableId === variable.id) {
        return {
          ...item,
          count: Math.max(1, Math.min(20, Math.round(value))),
        };
      }

      if (
        (item.kind === "wait" || item.kind === "event") &&
        item.durationVariableId === variable.id
      ) {
        return {
          ...item,
          durationMs: Math.max(0, Math.round(value)),
        };
      }

      return item;
    });
  }

  function setLineSpeedVariable(lineId: string, variableId: string) {
    lines = lines.map((line) =>
      line.id === lineId
        ? {
            ...line,
            speedVariableId: variableId || undefined,
            speed: variableId
              ? Math.max(0.05, Math.min(1, numberVariableValue(variableId, line.speed ?? 1)))
              : line.speed,
          }
        : line,
    );
    recordChange?.();
  }

  function setSequenceDurationVariable(seqIndex: number, variableId: string) {
    const item = sequence[seqIndex];
    if (!item || (item.kind !== "wait" && item.kind !== "event")) return;

    const newSeq = [...sequence];
    newSeq[seqIndex] = {
      ...item,
      durationVariableId: variableId || undefined,
      durationMs: variableId
        ? Math.max(0, Math.round(numberVariableValue(variableId, item.durationMs)))
        : item.durationMs,
    } as SequenceItem;
    sequence = newSeq;
    recordChange?.();
  }

  function setRepeatCountVariable(seqIndex: number, variableId: string) {
    const item = sequence[seqIndex];
    if (!item || item.kind !== "repeat") return;

    const newSeq = [...sequence];
    newSeq[seqIndex] = {
      ...item,
      countVariableId: variableId || undefined,
      count: variableId
        ? Math.max(1, Math.min(20, Math.round(numberVariableValue(variableId, item.count))))
        : item.count,
    };
    sequence = newSeq;
    recordChange?.();
  }

  function sequencePathLineIds(sourceSequence: SequenceItem[] = sequence): string[] {
    const ids: string[] = [];
    sourceSequence.forEach((item) => {
      if (item.kind === "path") {
        ids.push(item.lineId);
      } else if (item.kind === "repeat") {
        ids.push(...(item.lineIds || []));
      }
    });
    return ids;
  }

  function orderedLineIdsForSelection(sourceLineIds: string[]): string[] {
    const sourceSet = new Set(sourceLineIds);
    const ordered = sequencePathLineIds().filter((lineId) => sourceSet.has(lineId));
    sourceLineIds.forEach((lineId) => {
      if (!ordered.includes(lineId)) ordered.push(lineId);
    });
    return ordered;
  }

  function lineIsLocked(lineId: string): boolean {
    return lines.find((line) => line.id === lineId)?.locked ?? false;
  }

  function handlePathDragStart(
    event: DragEvent,
    lineId: string,
    sourceRepeatId = "",
  ) {
    if (!lineId || lineIsLocked(lineId)) {
      event.preventDefault();
      return;
    }

    draggedLineId = lineId;
    event.dataTransfer?.setData("application/x-pedro-line-id", lineId);
    event.dataTransfer?.setData("application/x-pedro-repeat-id", sourceRepeatId);
    event.dataTransfer?.setData("text/plain", lineId);
    if (event.dataTransfer) {
      event.dataTransfer.effectAllowed = "move";
    }
  }

  function handlePathDragEnd() {
    draggedLineId = "";
    dragOverRepeatId = "";
  }

  function handleRepeatDragOver(event: DragEvent, repeatId: string) {
    if (!draggedLineId || !repeatId) return;
    const repeatItem = sequence.find(
      (item) => item.kind === "repeat" && item.id === repeatId,
    ) as Extract<SequenceItem, { kind: "repeat" }> | undefined;
    if (!repeatItem || repeatItem.locked) return;

    event.preventDefault();
    if (event.dataTransfer) {
      event.dataTransfer.dropEffect = "move";
    }
    dragOverRepeatId = repeatId;
  }

  function handleRepeatDragLeave(event: DragEvent, repeatId: string) {
    const nextTarget = event.relatedTarget as Node | null;
    if (
      dragOverRepeatId === repeatId &&
      (!nextTarget || !(event.currentTarget as HTMLElement).contains(nextTarget))
    ) {
      dragOverRepeatId = "";
    }
  }

  function moveLineIntoRepeat(seqIndex: number, lineId: string) {
    const targetRepeat = sequence[seqIndex];
    if (
      !lineId ||
      lineIsLocked(lineId) ||
      !targetRepeat ||
      targetRepeat.kind !== "repeat" ||
      targetRepeat.locked
    ) {
      return;
    }

    const nextSequence: SequenceItem[] = [];
    let targetIndex = -1;

    sequence.forEach((item) => {
      if (item.kind === "path") {
        if (item.lineId !== lineId) {
          nextSequence.push(item);
        }
        return;
      }

      if (item.kind === "repeat") {
        const nextLineIds = (item.lineIds || []).filter((id) => id !== lineId);
        const nextRepeat = { ...item, lineIds: nextLineIds };
        if (item.id === targetRepeat.id) {
          targetIndex = nextSequence.length;
          nextSequence.push(nextRepeat);
        } else {
          nextSequence.push(nextRepeat);
        }
        return;
      }

      nextSequence.push(item);
    });

    if (targetIndex < 0) return;

    const updatedTarget = nextSequence[targetIndex];
    if (updatedTarget.kind !== "repeat") return;
    nextSequence[targetIndex] = {
      ...updatedTarget,
      lineIds: [...updatedTarget.lineIds, lineId],
    };

    sequence = nextSequence;
    syncLinesToSequence(nextSequence);
    recordChange?.();
  }

  function handleRepeatDrop(event: DragEvent, seqIndex: number) {
    event.preventDefault();
    const lineId =
      event.dataTransfer?.getData("application/x-pedro-line-id") ||
      event.dataTransfer?.getData("text/plain") ||
      draggedLineId;
    moveLineIntoRepeat(seqIndex, lineId);
    handlePathDragEnd();
  }

  function offsetPoint<T extends BasePoint>(point: T, dx: number, dy: number): T {
    if (point.poseVariableId) return { ...point };
    return {
      ...point,
      x: Number(point.x || 0) + dx,
      y: Number(point.y || 0) + dy,
    };
  }

  function cloneLineForRoute(line: Line, index: number, offset = 0): Line {
    const clone = cloneJson(line);
    clone.id = makeId();
    clone.name = `${line.name || `Path ${lines.length + index + 1}`} Copy`;
    clone.endPoint = offsetPoint(clone.endPoint, offset, offset);
    clone.controlPoints = (clone.controlPoints || []).map((point) =>
      offsetPoint(point, offset, offset),
    );
    return clone;
  }

  function getLineStartPoint(lineId: string): Point {
    const lineIndex = lines.findIndex((line) => line.id === lineId);
    if (lineIndex <= 0) return cloneJson(startPoint);
    return cloneJson(lines[lineIndex - 1].endPoint);
  }

  function makePathVariableName(): string {
    const usedNames = new Set(pathVariables.map((variable) => variable.name.trim()));
    let index = pathVariables.length + 1;
    let name = `Path Variable ${index}`;

    while (usedNames.has(name)) {
      index++;
      name = `Path Variable ${index}`;
    }

    return name;
  }

  function createPathVariableFromSelectedChain() {
    if (!selectedChain) return;
    const orderedIds = orderedLineIdsForSelection(selectedChain.lineIds || []);
    const selectedLines = orderedIds
      .map((lineId) => lines.find((line) => line.id === lineId))
      .filter(Boolean) as Line[];

    if (!selectedLines.length) return;

    const variable: PathVariable = {
      id: `path-variable-${makeId()}`,
      name: selectedChain.name
        ? `${selectedChain.name} Variable`
        : makePathVariableName(),
      startPoint: getLineStartPoint(selectedLines[0].id || ""),
      lines: selectedLines.map((line, index) => {
        const clone = cloneJson(line);
        clone.id = `path-variable-line-${makeId()}-${index}`;
        return clone;
      }),
    };

    pathVariables = [...pathVariables, variable];
    recordChange?.();
  }

  function updatePathVariableName(variableId: string, name: string) {
    pathVariables = pathVariables.map((variable) =>
      variable.id === variableId
        ? { ...variable, name }
        : variable,
    );
  }

  function duplicatePathVariable(variableId: string) {
    const variable = pathVariables.find((item) => item.id === variableId);
    if (!variable) return;

    const clone = cloneJson(variable);
    clone.id = `path-variable-${makeId()}`;
    clone.name = `${variable.name || "Path Variable"} Copy`;
    clone.lines = clone.lines.map((line, index) => ({
      ...line,
      id: `path-variable-line-${makeId()}-${index}`,
    }));
    pathVariables = [...pathVariables, clone];
    recordChange?.();
  }

  function removePathVariable(variableId: string) {
    pathVariables = pathVariables.filter((variable) => variable.id !== variableId);
    recordChange?.();
  }

  function insertPathVariable(variableId: string) {
    const variable = pathVariables.find((item) => item.id === variableId);
    if (!variable || !variable.lines.length) return;

    const clonedLines = variable.lines.map((line, index) =>
      cloneLineForRoute(
        {
          ...line,
          name: `${variable.name || "Path Variable"} ${index + 1}`,
        },
        index,
        0,
      ),
    );

    lines = [...lines, ...clonedLines];
    sequence = [
      ...sequence,
      ...clonedLines.map((line) => ({ kind: "path", lineId: line.id! }) as SequenceItem),
    ];
    collapsedSections.lines.push(...clonedLines.map(() => false));
    collapsedSections.controlPoints.push(...clonedLines.map(() => true));
    collapsedSections = { ...collapsedSections };

    const insertedChain: PathChain = {
      id: makeChainId(),
      name: variable.name || `Inserted Path ${pathChains.length + 1}`,
      color: getRandomColor(),
      lineIds: clonedLines.map((line) => line.id!).filter(Boolean),
    };
    pathChains = [...pathChains, insertedChain];
    selectedChainId = insertedChain.id;
    syncLineColorsToChains();
    recordChange?.();
  }

  function pointHeadingDegrees(point: Point): number {
    const currentPoint = point as any;
    if (currentPoint.heading === "constant") return Number(currentPoint.degrees) || 0;
    if (currentPoint.heading === "linear") return Number(currentPoint.endDeg ?? currentPoint.startDeg) || 0;
    return 0;
  }

  function firstFinite(...values: unknown[]): number {
    for (const value of values) {
      const numeric = Number(value);
      if (Number.isFinite(numeric)) return numeric;
    }
    return 0;
  }

  function makePoseVariableName(): string {
    const usedNames = new Set(poseVariables.map((variable) => variable.name.trim()));
    let index = poseVariables.length + 1;
    let name = `Pose ${index}`;

    while (usedNames.has(name)) {
      index++;
      name = `Pose ${index}`;
    }

    return name;
  }

  function bindPointToPoseVariable(
    point: Point,
    variable: PoseVariable,
    forcePoseHeading = false,
    preferLinear = false,
  ): Point {
    const targetHeading = Number.isFinite(Number(variable.heading))
      ? Number(variable.heading)
      : 0;
    const basePoint = {
      x: Number(variable.x) || 0,
      y: Number(variable.y) || 0,
      locked: point.locked,
      poseVariableId: variable.id,
    };

    if (preferLinear || point.heading === "linear") {
      return {
        ...basePoint,
        heading: "linear",
        startDeg: firstFinite(point.startDeg, point.degrees, point.endDeg, targetHeading),
        endDeg: targetHeading,
        headingCurve: point.heading === "linear" ? point.headingCurve ?? 1 : 1,
      };
    }

    if (!forcePoseHeading && point.heading === "tangential") {
      return {
        ...basePoint,
        heading: "tangential",
        reverse: point.reverse ?? false,
      };
    }

    return {
      ...basePoint,
      heading: "constant",
      degrees: targetHeading,
    };
  }

  function clearPointPoseVariable(point: Point): Point {
    const { poseVariableId, ...nextPoint } = point;
    return nextPoint as Point;
  }

  function syncPoseVariableUsage(
    poseVariableId: string,
    sourceVariable?: PoseVariable,
  ) {
    const variable =
      sourceVariable || poseVariables.find((item) => item.id === poseVariableId);
    if (!variable) return;

    if (startPoint.poseVariableId === poseVariableId) {
      startPoint = bindPointToPoseVariable(startPoint, variable);
    }

    const nextLines = lines.map((line) =>
      line.endPoint?.poseVariableId === poseVariableId
        ? { ...line, endPoint: bindPointToPoseVariable(line.endPoint, variable) }
        : line,
    );

    if (JSON.stringify(nextLines) !== JSON.stringify(lines)) {
      lines = nextLines;
    }
  }

  function addPoseVariable() {
    const variable: PoseVariable = {
      id: `pose-${makeId()}`,
      name: makePoseVariableName(),
      x: Number(startPoint.x) || 0,
      y: Number(startPoint.y) || 0,
      heading: pointHeadingDegrees(startPoint),
    };

    poseVariables = [...poseVariables, variable];
    recordChange?.();
  }

  function removePoseVariable(poseVariableId: string) {
    poseVariables = poseVariables.filter((variable) => variable.id !== poseVariableId);

    if (startPoint.poseVariableId === poseVariableId) {
      startPoint = clearPointPoseVariable(startPoint);
    }

    lines = lines.map((line) =>
      line.endPoint?.poseVariableId === poseVariableId
        ? { ...line, endPoint: clearPointPoseVariable(line.endPoint) }
        : line,
    );
    recordChange?.();
  }

  function handlePoseVariableChange(variable: PoseVariable) {
    const resolvedVariables = resolvePoseVariableExpressions(
      poseVariables.map((item) => (item.id === variable.id ? variable : item)),
      numberVariables,
    );
    poseVariables = resolvedVariables;
    const resolvedVariable = resolvedVariables.find((item) => item.id === variable.id);
    if (resolvedVariable) {
      syncPoseVariableUsage(variable.id, resolvedVariable);
    }
  }

  function handleStartPoseVariableChange(poseVariableId: string) {
    if (!poseVariableId) {
      startPoint = clearPointPoseVariable(startPoint);
      recordChange?.();
      return;
    }

    const variable = poseVariables.find((item) => item.id === poseVariableId);
    if (!variable) return;

    startPoint = bindPointToPoseVariable(startPoint, variable, true);
    recordChange?.();
  }

  function handleLinePoseVariableChange(lineId: string, poseVariableId: string) {
    if (!lineId) return;

    const variable = poseVariables.find((item) => item.id === poseVariableId);
    lines = lines.map((line) => {
      if (line.id !== lineId) return line;
      return {
        ...line,
        endPoint: variable
          ? bindPointToPoseVariable(line.endPoint, variable, false, true)
          : clearPointPoseVariable(line.endPoint),
      };
    });
    recordChange?.();
  }

  function buildPointWithHeadingMode(point: Point, mode: string): Point {
    const variable = point.poseVariableId
      ? poseVariables.find((item) => item.id === point.poseVariableId)
      : null;
    const targetHeading = variable ? Number(variable.heading) || 0 : null;
    const previousHeading = firstFinite(
      point.startDeg,
      point.degrees,
      point.endDeg,
      targetHeading,
    );
    const basePoint = {
      x: Number(point.x) || 0,
      y: Number(point.y) || 0,
      locked: point.locked,
      poseVariableId: point.poseVariableId,
    };

    if (mode === "linear") {
      return {
        ...basePoint,
        heading: "linear",
        startDeg: firstFinite(point.startDeg, previousHeading),
        endDeg: targetHeading ?? firstFinite(point.endDeg, previousHeading),
        headingCurve: point.heading === "linear" ? point.headingCurve ?? 1 : 1,
      };
    }

    if (mode === "tangential") {
      return {
        ...basePoint,
        heading: "tangential",
        reverse: point.reverse ?? false,
      };
    }

    return {
      ...basePoint,
      heading: "constant",
      degrees: targetHeading ?? previousHeading,
    };
  }

  function handleLineHeadingModeChange(lineId: string, mode: string) {
    if (!lineId) return;

    lines = lines.map((line) =>
      line.id === lineId
        ? { ...line, endPoint: buildPointWithHeadingMode(line.endPoint, mode) }
        : line,
    );
    recordChange?.();
  }

  function mirrorCurrentPath(axis: MirrorAxis) {
    const mirrored = mirrorPathData(
      {
        startPoint,
        lines,
        poseVariables,
        pathVariables,
        numberVariables,
      },
      axis,
    );

    startPoint = mirrored.startPoint || startPoint;
    lines = mirrored.lines || [];
    poseVariables = mirrored.poseVariables || [];
    pathVariables = mirrored.pathVariables || [];
    numberVariables = mirrored.numberVariables || [];
    percent = 0;
    recordChange?.();
  }

  function getWait(i: any) {
    return i as any;
  }
  function makeWaitItem(name = "Wait", durationMs = 0): SequenceItem {
    return {
      kind: "wait",
      id: makeId(),
      name,
      durationMs,
      locked: false,
    } as SequenceItem;
  }
  function makeShootEventItem(): SequenceItem {
    return {
      kind: "event",
      id: makeId(),
      name: "Shoot",
      durationMs: 500,
      locked: false,
    } as SequenceItem;
  }

  function insertLineAfter(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;
    const lineIndex = lines.findIndex((l) => l.id === seqItem.lineId);
    const currentLine = lines[lineIndex];

    // Find the next path item in the sequence after seqIndex
    let nextPathSeqIndex = -1;
    for (let i = seqIndex + 1; i < sequence.length; i++) {
      if (sequence[i].kind === "path") {
        nextPathSeqIndex = i;
        break;
      }
    }

    // If there is no next path in sequence, fall back to addLine behavior (append new randomized point)
    let newPoint: Point | null = null;
    if (nextPathSeqIndex !== -1) {
      const nextLineId = (sequence[nextPathSeqIndex] as any).lineId;
      const nextLine = lines.find((l) => l.id === nextLineId);
      if (
        nextLine &&
        nextLine.endPoint &&
        currentLine &&
        currentLine.endPoint
      ) {
        const a = currentLine.endPoint;
        const b = nextLine.endPoint;
        const midX = (Number(a.x) + Number(b.x)) / 2;
        const midY = (Number(a.y) + Number(b.y)) / 2;
        newPoint = {
          x: midX,
          y: midY,
          heading: "tangential",
          reverse: false,
        };
      }
    }

    if (!newPoint) {
      // fallback: random nearby point from current end
      if (currentLine && currentLine.endPoint) {
        newPoint = {
          x: (currentLine.endPoint.x ?? 72) + _.random(-12, 12),
          y: (currentLine.endPoint.y ?? 72) + _.random(-12, 12),
          heading: "tangential",
          reverse: false,
        };
      } else {
        newPoint = {
          x: _.random(0, 141.5),
          y: _.random(0, 141.5),
          heading: "tangential",
          reverse: false,
        };
      }
    }

    const newLine = {
      id: makeId(),
      endPoint: newPoint,
      controlPoints: [],
      color: getRandomColor(),
      speed: currentLine?.speed ?? 1,
      name: `Path ${lines.length + 1}`,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };

    // Insert the new line after the current one and a sequence item after current seq index
    const newLines = [...lines];
    newLines.splice(lineIndex + 1, 0, newLine);
    lines = newLines;

    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, { kind: "path", lineId: newLine.id! });
    sequence = newSeq;
    ensureLineInDefaultChain(newLine.id!);

    collapsedSections.lines.splice(lineIndex + 1, 0, false);
    collapsedSections.controlPoints.splice(lineIndex + 1, 0, true);

    // Force reactivity
    collapsedSections = { ...collapsedSections };
  }

  function duplicatePathAfter(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;

    const lineIndex = lines.findIndex((line) => line.id === seqItem.lineId);
    const sourceLine = lines[lineIndex];
    if (!sourceLine) return;

    const clone = cloneLineForRoute(sourceLine, 0, 4);
    const newLines = [...lines];
    newLines.splice(lineIndex + 1, 0, clone);
    lines = newLines;

    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, { kind: "path", lineId: clone.id! });
    sequence = newSeq;

    const ownerChainId = getLinePrimaryChainId(sourceLine.id || "");
    if (ownerChainId) {
      pathChains = pathChains.map((chain) => {
        if (chain.id !== ownerChainId) return chain;
        const sourceIndex = chain.lineIds.indexOf(sourceLine.id || "");
        const nextLineIds = [...chain.lineIds];
        nextLineIds.splice(sourceIndex >= 0 ? sourceIndex + 1 : nextLineIds.length, 0, clone.id!);
        return { ...chain, lineIds: nextLineIds };
      });
    } else {
      ensureLineInDefaultChain(clone.id!);
    }

    collapsedSections.lines.splice(lineIndex + 1, 0, false);
    collapsedSections.controlPoints.splice(lineIndex + 1, 0, true);
    collapsedSections = { ...collapsedSections };
    syncLineColorsToChains();
    recordChange?.();
  }

  function wrapPathInRepeat(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;

    const line = lines.find((item) => item.id === seqItem.lineId);
    const repeatItem: SequenceItem = {
      kind: "repeat",
      id: makeId(),
      name: "Repeat Loop",
      count: 2,
      lineIds: [seqItem.lineId],
      locked: false,
    };

    const newSeq = [...sequence];
    newSeq.splice(seqIndex, 1, repeatItem);
    sequence = newSeq;
    syncLinesToSequence(newSeq);
    if (line) selectedChainId = getLinePrimaryChainId(line.id || "") || selectedChainId;
    recordChange?.();
  }

  function wrapSelectedChainInRepeat() {
    if (!selectedChain) return;
    const orderedIds = orderedLineIdsForSelection(selectedChain.lineIds || []);
    if (!orderedIds.length) return;

    const selectedLineSet = new Set(orderedIds);
    const firstIndex = sequence.findIndex(
      (item) => item.kind === "path" && selectedLineSet.has(item.lineId),
    );
    const insertIndex = firstIndex >= 0 ? firstIndex : sequence.length;
    const newSeq = sequence.filter(
      (item) => !(item.kind === "path" && selectedLineSet.has(item.lineId)),
    );
    newSeq.splice(insertIndex, 0, {
      kind: "repeat",
      id: makeId(),
      name: `${selectedChain.name || "Chain"} Repeat`,
      count: 2,
      lineIds: orderedIds,
      locked: false,
    });
    sequence = newSeq;
    syncLinesToSequence(newSeq);
    recordChange?.();
  }

  function unwrapRepeat(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "repeat") return;

    const replacement = (seqItem.lineIds || []).map(
      (lineId) => ({ kind: "path", lineId }) as SequenceItem,
    );
    const newSeq = [...sequence];
    newSeq.splice(seqIndex, 1, ...replacement);
    sequence = newSeq;
    syncLinesToSequence(newSeq);
    recordChange?.();
  }

  function duplicateRepeatAfter(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "repeat") return;

    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, {
      ...cloneJson(seqItem),
      id: makeId(),
      name: `${seqItem.name || "Repeat Loop"} Copy`,
    });
    sequence = newSeq;
    syncLinesToSequence(newSeq);
    recordChange?.();
  }

  function duplicateLineInsideRepeat(seqIndex: number, lineId: string) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "repeat") return;

    const lineIndex = lines.findIndex((line) => line.id === lineId);
    const sourceLine = lines[lineIndex];
    if (!sourceLine) return;

    const clone = cloneLineForRoute(sourceLine, 0, 4);
    const newLines = [...lines];
    newLines.splice(lineIndex + 1, 0, clone);
    lines = newLines;

    const insertIndex = seqItem.lineIds.indexOf(lineId);
    const nextLineIds = [...seqItem.lineIds];
    nextLineIds.splice(insertIndex >= 0 ? insertIndex + 1 : nextLineIds.length, 0, clone.id!);

    const newSeq = [...sequence];
    newSeq[seqIndex] = { ...seqItem, lineIds: nextLineIds };
    sequence = newSeq;

    const ownerChainId = getLinePrimaryChainId(sourceLine.id || "");
    if (ownerChainId) {
      pathChains = pathChains.map((chain) => {
        if (chain.id !== ownerChainId) return chain;
        const sourceIndex = chain.lineIds.indexOf(sourceLine.id || "");
        const nextIds = [...chain.lineIds];
        nextIds.splice(sourceIndex >= 0 ? sourceIndex + 1 : nextIds.length, 0, clone.id!);
        return { ...chain, lineIds: nextIds };
      });
    }

    collapsedSections.lines.splice(lineIndex + 1, 0, false);
    collapsedSections.controlPoints.splice(lineIndex + 1, 0, true);
    collapsedSections = { ...collapsedSections };
    syncLineColorsToChains();
    recordChange?.();
  }

  function updateRepeatItem(
    seqIndex: number,
    patch: Partial<{ name: string; count: number; locked: boolean }>,
    commit = false,
  ) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "repeat") return;

    const newSeq = [...sequence];
    newSeq[seqIndex] = {
      ...seqItem,
      ...patch,
      count: Math.max(1, Math.min(20, Math.round(Number(patch.count ?? seqItem.count) || 1))),
    };
    sequence = newSeq;
    if (commit) recordChange?.();
  }

  // Insert a midpoint between this path and the next path in sequence
  function insertMidpointAfter(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;
    const lineIndex = lines.findIndex((l) => l.id === seqItem.lineId);
    const currentLine = lines[lineIndex];

    // Find the next path in sequence
    let nextPathSeqIndex = -1;
    for (let i = seqIndex + 1; i < sequence.length; i++) {
      if (sequence[i].kind === "path") {
        nextPathSeqIndex = i;
        break;
      }
    }

    if (nextPathSeqIndex === -1) {
      // no next path -> do nothing or fallback
      return;
    }

    const nextLineId = (sequence[nextPathSeqIndex] as any).lineId;
    const nextLine = lines.find((l) => l.id === nextLineId);
    if (!currentLine || !nextLine) return;

    const a = currentLine.endPoint;
    const b = nextLine.endPoint;
    const midX = (Number(a.x) + Number(b.x)) / 2;
    const midY = (Number(a.y) + Number(b.y)) / 2;

    const newLine: Line = {
      id: makeId(),
      endPoint: {
        x: midX,
        y: midY,
        heading: "tangential",
        reverse: false,
      },
      controlPoints: [],
      color: getRandomColor(),
      speed: currentLine?.speed ?? 1,
      name: `Path ${lines.length + 1}`,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };

    // Insert into lines right after current line index
    const newLines = [...lines];
    newLines.splice(lineIndex + 1, 0, newLine);
    lines = newLines;

    // Insert into sequence right after seqIndex
    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, { kind: "path", lineId: newLine.id! });
    sequence = newSeq;
    ensureLineInDefaultChain(newLine.id!);

    collapsedSections.lines.splice(lineIndex + 1, 0, false);
    collapsedSections.controlPoints.splice(lineIndex + 1, 0, true);

    collapsedSections = { ...collapsedSections };
    recordChange();
  }

  function removeLine(idx: number) {
    const removedId = lines[idx]?.id;
    let _lns = lines;
    lines.splice(idx, 1);
    lines = _lns;
    if (removedId) {
      sequence = sequence
        .map((item) =>
          item.kind === "repeat"
            ? {
                ...item,
                lineIds: item.lineIds.filter((lineId) => lineId !== removedId),
              }
            : item,
        )
        .filter((s) =>
          s.kind === "path"
            ? s.lineId !== removedId
            : s.kind === "repeat"
              ? s.lineIds.length > 0
              : true,
        );
      removeLineFromChains(removedId);
    }
    collapsedSections.lines.splice(idx, 1);
    collapsedSections.controlPoints.splice(idx, 1);
    recordChange();
  }

  function addLine() {
    const newLine: Line = {
      id: makeId(),
      name: `Path ${lines.length + 1}`,
      endPoint: {
        x: _.random(0, 141.5),
        y: _.random(0, 141.5),
        heading: "tangential",
        reverse: false,
      },
      controlPoints: [],
      color: getRandomColor(),
      speed: 1,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };
    lines = [...lines, newLine];
    sequence = [...sequence, { kind: "path", lineId: newLine.id! }];
    ensureLineInDefaultChain(newLine.id!);
    collapsedSections.lines.push(false);
    collapsedSections.controlPoints.push(true);
    recordChange();
  }

  function addControlPointToLineById(lineId: string) {
    const lineIndex = lines.findIndex((line) => line.id === lineId);
    if (lineIndex === -1) return;
    const line = lines[lineIndex];
    line.controlPoints = line.controlPoints || [];
    const prevPt = lineIndex === 0 ? startPoint : lines[lineIndex - 1].endPoint;
    const endPt = line.endPoint || { x: 72, y: 72 };
    const mx = ((prevPt?.x ?? 72) + (endPt?.x ?? 72)) / 2;
    const my = ((prevPt?.y ?? 72) + (endPt?.y ?? 72)) / 2;
    line.controlPoints.push({
      x: mx + _.random(-4, 4),
      y: my + _.random(-4, 4),
    });
    collapsedSections.controlPoints[lineIndex] = false;
    lines = [...lines];
    collapsedSections = { ...collapsedSections };
    recordChange?.();
  }

  // Add a control point to the line represented by `seqIndex` in the sequence
  function addControlPointToLine(seqIndex: number) {
    const seqItem = sequence[seqIndex];
    if (!seqItem || seqItem.kind !== "path") return;
    addControlPointToLineById(seqItem.lineId);
  }

  // Add a control point to the last path in `lines` (fallback: create a new line)
  function addControlPointToLastLine() {
    if (!lines || lines.length === 0) {
      // No lines exist: create a new line instead
      addLine();
      return;
    }

    // Prefer adding to the first line whose control points are expanded (user is focusing it)
    let targetIdx = collapsedSections.controlPoints.findIndex(
      (v) => v === false,
    );
    if (targetIdx === -1) targetIdx = lines.length - 1;

    const line = lines[targetIdx];
    line.controlPoints = line.controlPoints || [];
    // Insert a control point near the line midpoint for convenience
    const prevPt = targetIdx === 0 ? startPoint : lines[targetIdx - 1].endPoint;
    const endPt = line.endPoint || { x: 72, y: 72 };
    const mx = ((prevPt?.x ?? 72) + (endPt?.x ?? 72)) / 2;
    const my = ((prevPt?.y ?? 72) + (endPt?.y ?? 72)) / 2;
    line.controlPoints.push({
      x: mx + _.random(-4, 4),
      y: my + _.random(-4, 4),
    });
    // Ensure control points UI is expanded for this line
    collapsedSections.controlPoints[targetIdx] = false;
    lines = [...lines];
    collapsedSections = { ...collapsedSections };
    recordChange?.();
  }

  function addWait() {
    sequence = [...sequence, makeWaitItem()];
  }

  function addEvent() {
    sequence = [...sequence, makeShootEventItem()];
  }

  function addRepeatLoop() {
    sequence = [
      ...sequence,
      {
        kind: "repeat",
        id: makeId(),
        name: "Repeat Loop",
        count: 2,
        lineIds: [],
        locked: false,
      } as SequenceItem,
    ];
    recordChange?.();
  }

  function addWaitAtStart() {
    sequence = [makeWaitItem(), ...sequence];
  }

  function addEventAtStart() {
    sequence = [makeShootEventItem(), ...sequence];
  }

  function addPathAtStart() {
    const newLine: Line = {
      id: makeId(),
      name: `Path ${lines.length + 1}`,
      endPoint: {
        x: _.random(0, 141.5),
        y: _.random(0, 141.5),
        heading: "tangential",
        reverse: false,
      },
      controlPoints: [],
      color: getRandomColor(),
      speed: 1,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };
    lines = [newLine, ...lines];
    sequence = [{ kind: "path", lineId: newLine.id! }, ...sequence];
    ensureLineInDefaultChain(newLine.id!);
    collapsedSections.lines = [false, ...collapsedSections.lines];
    collapsedSections.controlPoints = [
      true,
      ...collapsedSections.controlPoints,
    ];
    recordChange();
  }

  function insertWaitAfter(seqIndex: number) {
    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, makeWaitItem());
    sequence = newSeq;
  }

  function insertEventAfter(seqIndex: number) {
    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, makeShootEventItem());
    sequence = newSeq;
  }

  function insertPathAfter(seqIndex: number) {
    // Create a new line with default settings
    const newLine: Line = {
      id: makeId(),
      name: `Path ${lines.length + 1}`,
      endPoint: {
        x: _.random(36, 108),
        y: _.random(36, 108),
        heading: "tangential",
        reverse: false,
      },
      controlPoints: [],
      color: getRandomColor(),
      speed: 1,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    };

    // Add the new line to the lines array
    lines = [...lines, newLine];

    // Insert the new path in the sequence after the wait
    const newSeq = [...sequence];
    newSeq.splice(seqIndex + 1, 0, { kind: "path", lineId: newLine.id! });
    sequence = newSeq;
    ensureLineInDefaultChain(newLine.id!);

    // Add UI state for the new line
    collapsedSections.lines.push(false);
    collapsedSections.controlPoints.push(true);

    // Force reactivity
    collapsedSections = { ...collapsedSections };
    recordChange();
  }

  function syncLinesToSequence(newSeq: SequenceItem[]) {
    const pathOrder = sequencePathLineIds(newSeq);

    const indexedLines = lines.map((line, idx) => ({
      line,
      collapsed: collapsedSections.lines[idx],
      control: collapsedSections.controlPoints[idx],
    }));

    const byId = new Map(indexedLines.map((entry) => [entry.line.id, entry]));
    const reordered: typeof indexedLines = [];

    pathOrder.forEach((id) => {
      const entry = byId.get(id);
      if (entry) {
        reordered.push(entry);
        byId.delete(id);
      }
    });

    // Append any lines that are not currently in the sequence to preserve data
    reordered.push(...byId.values());

    lines = reordered.map((entry) => entry.line);
    collapsedSections = {
      ...collapsedSections,
      lines: reordered.map((entry) => entry.collapsed ?? false),
      controlPoints: reordered.map((entry) => entry.control ?? true),
    };
    // No collapsedEventMarkers to update
  }

  function moveSequenceItem(seqIndex: number, delta: number) {
    const targetIndex = seqIndex + delta;
    if (targetIndex < 0 || targetIndex >= sequence.length) return;

    // Prevent moving if either the source or target is a locked path or a locked wait
    const isLockedSequenceItem = (index: number) => {
      const it = sequence[index];
      if (!it) return false;
      if (it.kind === "path") {
        const ln = lines.find((l) => l.id === it.lineId);
        return ln?.locked ?? false;
      }
      if (it.kind === "wait" || it.kind === "event") {
        return (it as any).locked ?? false;
      }
      if (it.kind === "repeat") {
        const loopLocked = (it as any).locked ?? false;
        return loopLocked || it.lineIds.some((lineId) =>
          lines.find((line) => line.id === lineId)?.locked,
        );
      }
      return false;
    };

    if (isLockedSequenceItem(seqIndex) || isLockedSequenceItem(targetIndex))
      return;

    const newSeq = [...sequence];
    const [item] = newSeq.splice(seqIndex, 1);
    newSeq.splice(targetIndex, 0, item);
    sequence = newSeq;

    syncLinesToSequence(newSeq);
    recordChange?.();
  }
</script>

<div class="flex-1 flex flex-col justify-start items-center gap-2 h-full">
  <div
    class="flex flex-col justify-start items-start w-full rounded-lg bg-neutral-50 dark:bg-neutral-900 shadow-md p-4 overflow-y-scroll overflow-x-hidden h-full gap-6"
  >
    <ObstaclesSection bind:shapes bind:collapsedObstacles />

    <TelemetryPanel
      {percent}
      {timePrediction}
      {startPoint}
      {sequence}
      {settings}
      {lines}
      {robotXY}
      {robotHeading}
      {x}
      {y}
    />

    <PoseVariablesSection
      bind:poseVariables
      onAdd={addPoseVariable}
      onRemove={removePoseVariable}
      onChange={handlePoseVariableChange}
      onCommit={recordChange}
    />

    <div class="w-full rounded-md border border-neutral-200 dark:border-neutral-700 p-3 bg-white dark:bg-neutral-800">
      <div class="flex items-center gap-2 mb-2">
        <p class="text-xs font-semibold uppercase tracking-wide text-neutral-500 dark:text-neutral-300">
          Number Variables
        </p>
        <button
          on:click={addNumberVariable}
          class="px-2 py-1 text-xs rounded bg-emerald-100 text-emerald-700 dark:bg-emerald-900 dark:text-emerald-200"
        >
          New
        </button>
      </div>

      {#if numberVariables.length === 0}
        <p class="text-xs text-neutral-500 dark:text-neutral-400">
          Store reusable values for repeat counts, speeds, event timing, and other numeric auto constants.
        </p>
      {:else}
        <div class="flex flex-col gap-2">
          {#each numberVariables as variable (variable.id)}
            <div class="rounded border border-neutral-200 dark:border-neutral-700 bg-neutral-50 dark:bg-neutral-900 p-2">
              <div class="flex flex-wrap items-center gap-2">
                <input
                  type="text"
                  value={variable.name}
                  on:input={(event) =>
                    updateNumberVariable(variable.id, {
                      name: inputValue(event),
                    })}
                  on:blur={() => recordChange?.()}
                  class="min-w-0 flex-1 px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-white dark:bg-neutral-950"
                />
                <input
                  type="number"
                  step="0.001"
                  value={variable.value}
                  on:input={(event) =>
                    updateNumberVariable(variable.id, {
                      value: inputNumber(event),
                    })}
                  on:blur={() => updateNumberVariable(variable.id, {}, true)}
                  class="w-28 px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-white dark:bg-neutral-950"
                />
              </div>
              <div class="mt-2 flex flex-wrap items-center gap-2">
                <button
                  on:click={() => duplicateNumberVariable(variable.id)}
                  class="px-2 py-1 text-xs rounded bg-indigo-100 text-indigo-700 dark:bg-indigo-900 dark:text-indigo-200"
                >
                  Duplicate
                </button>
                <button
                  on:click={() => removeNumberVariable(variable.id)}
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

    <div class="w-full rounded-md border border-neutral-200 dark:border-neutral-700 p-3 bg-white dark:bg-neutral-800">
      <div class="flex items-center gap-2 mb-2">
        <p class="text-xs font-semibold uppercase tracking-wide text-neutral-500 dark:text-neutral-300">
          Path Variables
        </p>
        <button
          on:click={createPathVariableFromSelectedChain}
          disabled={!selectedChain || !(selectedChain.lineIds || []).length}
          class="px-2 py-1 text-xs rounded bg-cyan-100 text-cyan-700 dark:bg-cyan-900 dark:text-cyan-200 disabled:opacity-40"
        >
          Store Chain
        </button>
      </div>

      {#if pathVariables.length === 0}
        <p class="text-xs text-neutral-500 dark:text-neutral-400">
          Store the selected path chain as a reusable path template.
        </p>
      {:else}
        <div class="flex flex-col gap-2">
          {#each pathVariables as variable (variable.id)}
            <div class="rounded border border-neutral-200 dark:border-neutral-700 bg-neutral-50 dark:bg-neutral-900 p-2">
              <div class="flex items-center gap-2">
                <input
                  type="text"
                  value={variable.name}
                  on:input={(event) =>
                    updatePathVariableName(
                      variable.id,
                      inputValue(event),
                    )}
                  on:blur={recordChange}
                  class="min-w-0 flex-1 px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-white dark:bg-neutral-950"
                />
                <span class="text-xs text-neutral-500 dark:text-neutral-400">
                  {variable.lines.length} path{variable.lines.length === 1 ? "" : "s"}
                </span>
              </div>
              <div class="mt-2 flex flex-wrap items-center gap-2">
                <button
                  on:click={() => insertPathVariable(variable.id)}
                  class="px-2 py-1 text-xs rounded bg-emerald-100 text-emerald-700 dark:bg-emerald-900 dark:text-emerald-200"
                >
                  Insert Copy
                </button>
                <button
                  on:click={() => duplicatePathVariable(variable.id)}
                  class="px-2 py-1 text-xs rounded bg-indigo-100 text-indigo-700 dark:bg-indigo-900 dark:text-indigo-200"
                >
                  Duplicate
                </button>
                <button
                  on:click={() => removePathVariable(variable.id)}
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

    <StartingPointSection
      bind:startPoint
      {poseVariables}
      {numberVariables}
      onPoseVariableChange={handleStartPoseVariableChange}
      {addPathAtStart}
      {addWaitAtStart}
      {addEventAtStart}
    />

    <div class="w-full rounded-md border border-neutral-200 dark:border-neutral-700 p-3 bg-white dark:bg-neutral-800">
      <div class="flex items-center gap-2">
        <p class="text-xs font-semibold uppercase tracking-wide text-neutral-500 dark:text-neutral-300">Mirror</p>
        <button
          on:click={() => mirrorCurrentPath("x")}
          class="px-2.5 py-1.5 text-xs font-semibold rounded bg-sky-100 text-sky-700 dark:bg-sky-900 dark:text-sky-200"
          title="Mirror X coordinates across the field centerline"
        >
          X
        </button>
        <button
          on:click={() => mirrorCurrentPath("y")}
          class="px-2.5 py-1.5 text-xs font-semibold rounded bg-teal-100 text-teal-700 dark:bg-teal-900 dark:text-teal-200"
          title="Mirror Y coordinates across the field centerline"
        >
          Y
        </button>
      </div>
    </div>

    <div class="w-full rounded-md border border-neutral-200 dark:border-neutral-700 p-3 bg-white dark:bg-neutral-800">
      <div class="flex items-center gap-2 mb-2">
        <p class="text-xs font-semibold uppercase tracking-wide text-neutral-500 dark:text-neutral-300">Path Chains</p>
        <select
          bind:value={selectedChainId}
          class="flex-1 px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900"
        >
          {#each pathChains as chain (chain.id)}
            <option value={chain.id}>{chain.name} ({(chain.lineIds || []).length})</option>
          {/each}
        </select>
        <button on:click={addPathChain} class="px-2 py-1 text-xs rounded bg-emerald-100 text-emerald-700 dark:bg-emerald-900 dark:text-emerald-200">New</button>
        <button on:click={duplicateSelectedPathChain} class="px-2 py-1 text-xs rounded bg-indigo-100 text-indigo-700 dark:bg-indigo-900 dark:text-indigo-200">Duplicate</button>
        <button
          on:click={wrapSelectedChainInRepeat}
          disabled={!selectedChain || !(selectedChain.lineIds || []).length}
          class="px-2 py-1 text-xs rounded bg-cyan-100 text-cyan-700 dark:bg-cyan-900 dark:text-cyan-200 disabled:opacity-40"
        >
          Loop
        </button>
        <button
          on:click={removeSelectedPathChain}
          disabled={pathChains.length <= 1}
          class="px-2 py-1 text-xs rounded bg-rose-100 text-rose-700 dark:bg-rose-900 dark:text-rose-200 disabled:opacity-40"
        >
          Remove
        </button>
      </div>

      {#if selectedChain}
        <div class="grid grid-cols-1 md:grid-cols-2 gap-2 mb-2">
          <div class="flex items-center gap-2">
            <input
              type="text"
              bind:value={chainNameDraft}
              on:input={updateSelectedChainName}
              class="flex-1 px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900"
              placeholder="Chain name"
            />
          </div>

          <div class="flex items-center gap-2">
            <input
              type="color"
              bind:value={chainColorDraft}
              on:input={updateSelectedChainColor}
              class="w-10 h-8 rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-50 dark:bg-neutral-900"
              title="Path chain color"
            />
            <span class="text-xs text-neutral-500 dark:text-neutral-400">Path color</span>
          </div>
        </div>
      {/if}
    </div>

    <!-- Unified sequence render: paths and waits -->
    {#each sequence as item, sIdx}
      <div class="w-full">
        {#if item.kind === "path"}
          {#each lines.filter((l) => l.id === item.lineId) as ln (ln.id)}
            <div
              class="rounded-lg border border-transparent transition {draggedLineId === (ln.id || "") ? 'opacity-60' : ''}"
              role="listitem"
            >
              <div class="mb-1 flex justify-end">
                <button
                  type="button"
                  class="cursor-grab rounded border border-cyan-200 bg-cyan-50 px-2 py-0.5 text-[11px] font-semibold text-cyan-700 active:cursor-grabbing disabled:cursor-not-allowed disabled:opacity-40 dark:border-cyan-900 dark:bg-cyan-950/40 dark:text-cyan-200"
                  draggable={!ln.locked}
                  title={ln.locked ? "Locked paths cannot be dragged" : "Drag this path into a repeat loop"}
                  on:dragstart={(event) => handlePathDragStart(event, ln.id || "")}
                  on:dragend={handlePathDragEnd}
                  disabled={ln.locked}
                >
                  Drag To Loop
                </button>
              </div>
              <PathLineSection
                bind:line={ln}
                idx={lines.findIndex((l) => l.id === ln.id)}
                bind:lines
                bind:collapsed={
                  collapsedSections.lines[lines.findIndex((l) => l.id === ln.id)]
                }
                bind:collapsedControlPoints={
                  collapsedSections.controlPoints[
                    lines.findIndex((l) => l.id === ln.id)
                  ]
                }
                onRemove={() =>
                  removeLine(lines.findIndex((l) => l.id === ln.id))}
                onInsertAfter={() => addControlPointToLineById(ln.id || "")}
                onInsertMidpoint={() => insertMidpointAfter(sIdx)}
                onDuplicate={() => duplicatePathAfter(sIdx)}
                onWrapRepeat={() => wrapPathInRepeat(sIdx)}
                onAddWaitAfter={() => insertWaitAfter(sIdx)}
                onAddEventAfter={() => insertEventAfter(sIdx)}
                onMoveUp={() => moveSequenceItem(sIdx, -1)}
                onMoveDown={() => moveSequenceItem(sIdx, 1)}
                canMoveUp={sIdx !== 0}
                canMoveDown={sIdx !== sequence.length - 1}
                optimizeLine={optimizeLine}
                optimizing={optimizingLineIds?.[ln.id ?? ""] ?? false}
                chainOptions={chainOptions}
                selectedChainId={getLinePrimaryChainId(ln.id || "")}
                onChainChange={(chainId) => assignLineToChain(ln.id || "", chainId)}
                {poseVariables}
                {numberVariables}
                onPoseVariableChange={handleLinePoseVariableChange}
                onHeadingModeChange={handleLineHeadingModeChange}
                onSpeedVariableChange={setLineSpeedVariable}
                {recordChange}
              />
            </div>
          {/each}
        {:else if item.kind === "repeat"}
          <div
            class="w-full rounded-lg border bg-cyan-50 dark:bg-cyan-950/30 p-3 shadow-sm transition {dragOverRepeatId === item.id ? 'border-cyan-500 ring-2 ring-cyan-400/70 dark:ring-cyan-500/60' : 'border-cyan-300 dark:border-cyan-800'}"
            role="group"
            on:dragover={(event) => handleRepeatDragOver(event, item.id)}
            on:dragleave={(event) => handleRepeatDragLeave(event, item.id)}
            on:drop={(event) => handleRepeatDrop(event, sIdx)}
          >
            <div class="flex flex-wrap items-center gap-2 mb-2">
              <span class="text-xs font-semibold uppercase tracking-wide text-cyan-700 dark:text-cyan-200">
                Repeat Loop
              </span>
              {#if dragOverRepeatId === item.id}
                <span class="rounded-full bg-cyan-600 px-2 py-0.5 text-[11px] font-semibold text-white">
                  Drop path here
                </span>
              {/if}
              <input
                type="text"
                value={item.name}
                on:input={(event) =>
                  updateRepeatItem(sIdx, {
                    name: inputValue(event),
                  })}
                on:blur={() => recordChange?.()}
                class="min-w-0 flex-1 px-2 py-1 text-xs rounded border border-cyan-200 dark:border-cyan-800 bg-white dark:bg-neutral-950"
              />
              <span class="text-xs text-neutral-600 dark:text-neutral-300">x</span>
              <input
                type="number"
                min="1"
                max="20"
                value={item.count}
                disabled={item.locked || Boolean(item.countVariableId)}
                on:input={(event) =>
                  updateRepeatItem(sIdx, {
                    count: inputNumber(event),
                  })}
                on:blur={() => updateRepeatItem(sIdx, { count: item.count }, true)}
                class="w-16 px-2 py-1 text-xs rounded border border-cyan-200 dark:border-cyan-800 bg-white dark:bg-neutral-950"
              />
              <select
                value={item.countVariableId || ""}
                disabled={item.locked || numberVariables.length === 0}
                on:change={(event) =>
                  setRepeatCountVariable(
                    sIdx,
                    inputValue(event),
                  )}
                class="px-2 py-1 text-xs rounded border border-cyan-200 dark:border-cyan-800 bg-white dark:bg-neutral-950 disabled:opacity-40"
                title="Repeat count variable"
              >
                <option value="">Custom count</option>
                {#each numberVariables as variable (variable.id)}
                  <option value={variable.id}>{variable.name}</option>
                {/each}
              </select>
              <button
                on:click={() => updateRepeatItem(sIdx, { locked: !item.locked }, true)}
                class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200"
              >
                {item.locked ? "Unlock" : "Lock"}
              </button>
              <button
                on:click={() => duplicateRepeatAfter(sIdx)}
                class="px-2 py-1 text-xs rounded bg-indigo-100 text-indigo-700 dark:bg-indigo-900 dark:text-indigo-200"
              >
                Duplicate
              </button>
              <button
                on:click={() => unwrapRepeat(sIdx)}
                class="px-2 py-1 text-xs rounded bg-amber-100 text-amber-700 dark:bg-amber-900 dark:text-amber-200"
              >
                Unwrap
              </button>
              <button
                on:click={() => {
                  const newSeq = [...sequence];
                  newSeq.splice(sIdx, 1);
                  sequence = newSeq;
                  recordChange?.();
                }}
                class="px-2 py-1 text-xs rounded bg-rose-100 text-rose-700 dark:bg-rose-900 dark:text-rose-200"
              >
                Remove
              </button>
              <button
                on:click={() => moveSequenceItem(sIdx, -1)}
                disabled={sIdx === 0 || item.locked}
                class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200 disabled:opacity-40"
              >
                Up
              </button>
              <button
                on:click={() => moveSequenceItem(sIdx, 1)}
                disabled={sIdx === sequence.length - 1 || item.locked}
                class="px-2 py-1 text-xs rounded bg-neutral-100 text-neutral-700 dark:bg-neutral-800 dark:text-neutral-200 disabled:opacity-40"
              >
                Down
              </button>
            </div>
            <div class="flex flex-col gap-2">
              {#if item.lineIds.length === 0}
                <div class="rounded border border-dashed border-cyan-300 dark:border-cyan-700 bg-white/70 dark:bg-neutral-950/50 p-4 text-center text-xs font-semibold text-cyan-700 dark:text-cyan-200">
                  Drag paths into this loop
                </div>
              {/if}
              {#each item.lineIds as lineId, loopLineIndex (lineId)}
                {#each lines.filter((l) => l.id === lineId) as ln (ln.id)}
                  <div
                    class="rounded border border-cyan-200 dark:border-cyan-900 bg-white dark:bg-neutral-900 p-2 transition {draggedLineId === (ln.id || '') ? 'opacity-60' : ''}"
                    role="listitem"
                  >
                    <div class="mb-1 text-xs text-neutral-500 dark:text-neutral-400">
                      <span>Loop path {loopLineIndex + 1}</span>
                      <button
                        type="button"
                        class="ml-2 cursor-grab rounded border border-cyan-200 bg-cyan-50 px-2 py-0.5 text-[11px] font-semibold text-cyan-700 active:cursor-grabbing disabled:cursor-not-allowed disabled:opacity-40 dark:border-cyan-800 dark:bg-cyan-950/40 dark:text-cyan-200"
                        draggable={!ln.locked}
                        title={ln.locked ? "Locked paths cannot be dragged" : "Drag this path into another repeat loop"}
                        on:dragstart={(event) =>
                          handlePathDragStart(event, ln.id || "", item.id)}
                        on:dragend={handlePathDragEnd}
                        disabled={ln.locked}
                      >
                        Drag
                      </button>
                    </div>
                    <PathLineSection
                      bind:line={ln}
                      idx={lines.findIndex((l) => l.id === ln.id)}
                      bind:lines
                      bind:collapsed={
                        collapsedSections.lines[lines.findIndex((l) => l.id === ln.id)]
                      }
                      bind:collapsedControlPoints={
                        collapsedSections.controlPoints[
                          lines.findIndex((l) => l.id === ln.id)
                        ]
                      }
                      onRemove={() =>
                        removeLine(lines.findIndex((l) => l.id === ln.id))}
                      onInsertAfter={() => addControlPointToLineById(ln.id || "")}
                      onInsertMidpoint={() => insertMidpointAfter(sIdx)}
                      onDuplicate={() => duplicateLineInsideRepeat(sIdx, ln.id || "")}
                      onWrapRepeat={() => {}}
                      onAddWaitAfter={() => insertWaitAfter(sIdx)}
                      onAddEventAfter={() => insertEventAfter(sIdx)}
                      onMoveUp={() => moveSequenceItem(sIdx, -1)}
                      onMoveDown={() => moveSequenceItem(sIdx, 1)}
                      canMoveUp={sIdx !== 0}
                      canMoveDown={sIdx !== sequence.length - 1}
                      optimizeLine={optimizeLine}
                      optimizing={optimizingLineIds?.[ln.id ?? ""] ?? false}
                      chainOptions={chainOptions}
                      selectedChainId={getLinePrimaryChainId(ln.id || "")}
                      onChainChange={(chainId) => assignLineToChain(ln.id || "", chainId)}
                      {poseVariables}
                      {numberVariables}
                      onPoseVariableChange={handleLinePoseVariableChange}
                      onHeadingModeChange={handleLineHeadingModeChange}
                      onSpeedVariableChange={setLineSpeedVariable}
                      {recordChange}
                    />
                  </div>
                {/each}
              {/each}
            </div>
          </div>
        {:else}
          <WaitRow
            label={item.kind === "event" ? "Event" : "Wait"}
            name={getWait(item).name}
            durationMs={getWait(item).durationMs}
            durationVariableId={getWait(item).durationVariableId || ""}
            {numberVariables}
            locked={getWait(item).locked ?? false}
            onToggleLock={() => {
              const newSeq = [...sequence];
              newSeq[sIdx] = {
                ...getWait(item),
                locked: !(getWait(item).locked ?? false),
              };
              sequence = newSeq;
              recordChange?.();
            }}
            onChange={(newName, newDuration) => {
              const newSeq = [...sequence];
              newSeq[sIdx] = {
                ...getWait(item),
                name: newName,
                durationMs: Math.max(0, Number(newDuration) || 0),
              };
              sequence = newSeq;
            }}
            onDurationVariableChange={(variableId) =>
              setSequenceDurationVariable(sIdx, variableId)}
            onRemove={() => {
              const newSeq = [...sequence];
              newSeq.splice(sIdx, 1);
              sequence = newSeq;
            }}
            onInsertAfter={() => {
              const newSeq = [...sequence];
              newSeq.splice(sIdx + 1, 0, makeWaitItem());
              sequence = newSeq;
            }}
            onAddPathAfter={() => insertPathAfter(sIdx)}
            onAddEventAfter={() => insertEventAfter(sIdx)}
            onMoveUp={() => moveSequenceItem(sIdx, -1)}
            onMoveDown={() => moveSequenceItem(sIdx, 1)}
            canMoveUp={sIdx !== 0}
            canMoveDown={sIdx !== sequence.length - 1}
          />
        {/if}
      </div>
    {/each}

    <!-- Add Line Button -->
    <div class="flex flex-row items-center gap-4">
      <button
        on:click={addLine}
        class="font-semibold text-green-500 text-sm flex flex-row justify-start items-center gap-1"
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
        <p>Add Path</p>
      </button>

      <button
        on:click={addWait}
        class="font-semibold text-[#E1461B] text-sm flex flex-row justify-start items-center gap-1"
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
        <p>Add Wait</p>
      </button>

      <button
        on:click={addEvent}
        class="font-semibold text-purple-500 text-sm flex flex-row justify-start items-center gap-1"
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
        <p>Add Event</p>
      </button>

      <button
        on:click={addRepeatLoop}
        class="font-semibold text-cyan-600 text-sm flex flex-row justify-start items-center gap-1"
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
            d="M17 1l4 4-4 4"
          />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M3 11V9a4 4 0 014-4h14"
          />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M7 23l-4-4 4-4"
          />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M21 13v2a4 4 0 01-4 4H3"
          />
        </svg>
        <p>Add Repeat</p>
      </button>
    </div>
  </div>

  <PlaybackControls
    bind:playing
    {play}
    {pause}
    bind:percent
    {handleSeek}
    bind:loopAnimation
    {markers}
    totalTime={timePrediction?.totalTime ?? 0}
  />
</div>
