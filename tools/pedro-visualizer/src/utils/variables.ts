import type {
  BooleanVariable,
  EventMarker,
  Line,
  NumberVariable,
  PathVariable,
  PoseVariable,
  ScalarVariable,
  SequenceConditionalItem,
  SequenceItem,
  Variable,
  VariableType,
} from "../types";
import {
  buildExpressionScope,
  resolveBasePointExpressions,
  resolveBooleanExpression,
  resolveNumberExpression,
  resolvePointExpressions,
  sanitizeExpressionIdentifier,
  type ExpressionScope,
} from "./numberExpressions";

/**
 * Helpers for the unified `variables` array.
 *
 * Older .pp files stored three separate arrays (`numberVariables`,
 * `poseVariables`, `pathVariables`) and referenced numbers by id through
 * `*VariableId` fields. `migrateVariables` folds those into one list and
 * `migrate*` rewrites the id references as expressions.
 */

export const VARIABLE_TYPES: VariableType[] = [
  "number",
  "boolean",
  "pose",
  "path",
];

export const VARIABLE_TYPE_LABELS: Record<VariableType, string> = {
  number: "Number",
  boolean: "Boolean",
  pose: "Pose",
  path: "Path",
};

export function isNumberVariable(variable: Variable): variable is NumberVariable {
  return variable.type === "number";
}

export function isBooleanVariable(
  variable: Variable,
): variable is BooleanVariable {
  return variable.type === "boolean";
}

export function isPoseVariable(variable: Variable): variable is PoseVariable {
  return variable.type === "pose";
}

export function isPathVariable(variable: Variable): variable is PathVariable {
  return variable.type === "path";
}

export function isScalarVariable(variable: Variable): variable is ScalarVariable {
  return variable.type === "number" || variable.type === "boolean";
}

export function numberVariablesOf(variables: Variable[] = []): NumberVariable[] {
  return variables.filter(isNumberVariable);
}

export function booleanVariablesOf(
  variables: Variable[] = [],
): BooleanVariable[] {
  return variables.filter(isBooleanVariable);
}

export function poseVariablesOf(variables: Variable[] = []): PoseVariable[] {
  return variables.filter(isPoseVariable);
}

export function pathVariablesOf(variables: Variable[] = []): PathVariable[] {
  return variables.filter(isPathVariable);
}

export function scalarVariablesOf(variables: Variable[] = []): ScalarVariable[] {
  return variables.filter(isScalarVariable);
}

export function findVariable(
  variables: Variable[] = [],
  id: string | undefined,
): Variable | undefined {
  if (!id) return undefined;
  return variables.find((variable) => variable.id === id);
}

export function createVariableId(): string {
  return `var-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
}

export function uniqueVariableName(
  variables: Variable[],
  base: string,
): string {
  const used = new Set(
    variables.map((variable) => variable.name.trim().toLowerCase()),
  );
  if (!used.has(base.trim().toLowerCase())) return base;

  let index = 2;
  while (used.has(`${base}${index}`.toLowerCase())) index++;
  return `${base}${index}`;
}

const DEFAULT_NAME_BY_TYPE: Record<VariableType, string> = {
  number: "value",
  boolean: "flag",
  pose: "pose",
  path: "path",
};

export function createVariable(
  type: VariableType,
  variables: Variable[] = [],
  overrides: Partial<Variable> = {},
): Variable {
  const name = uniqueVariableName(variables, DEFAULT_NAME_BY_TYPE[type]);
  const id = createVariableId();

  switch (type) {
    case "boolean":
      return { id, name, type: "boolean", value: false, ...overrides } as BooleanVariable;
    case "pose":
      return {
        id,
        name,
        type: "pose",
        x: 0,
        y: 0,
        heading: 0,
        ...overrides,
      } as PoseVariable;
    case "path":
      return {
        id,
        name,
        type: "path",
        startPoint: { x: 0, y: 0, heading: "tangential", reverse: false },
        lines: [],
        ...overrides,
      } as PathVariable;
    case "number":
    default:
      return { id, name, type: "number", value: 0, ...overrides } as NumberVariable;
  }
}

/**
 * Converts a variable to another type, preserving whatever carries over.
 * Used by the type dropdown in the variables list.
 */
export function convertVariableType(
  variable: Variable,
  type: VariableType,
): Variable {
  if (variable.type === type) return variable;

  const base = { id: variable.id, name: variable.name, description: variable.description };

  switch (type) {
    case "number": {
      const value =
        variable.type === "boolean"
          ? variable.value
            ? 1
            : 0
          : variable.type === "pose"
            ? variable.x
            : 0;
      const valueExpression =
        variable.type === "boolean" ? variable.valueExpression : undefined;
      return { ...base, type: "number", value, valueExpression };
    }

    case "boolean": {
      const value =
        variable.type === "number"
          ? Number(variable.value) !== 0
          : false;
      const valueExpression =
        variable.type === "number" ? variable.valueExpression : undefined;
      return { ...base, type: "boolean", value, valueExpression };
    }

    case "pose": {
      const x = variable.type === "number" ? Number(variable.value) || 0 : 0;
      const xExpression =
        variable.type === "number" ? variable.valueExpression : undefined;
      return { ...base, type: "pose", x, xExpression, y: 0, heading: 0 };
    }

    case "path":
    default:
      return {
        ...base,
        type: "path",
        startPoint: { x: 0, y: 0, heading: "tangential", reverse: false },
        lines: [],
      };
  }
}

function legacyExpressionFor(
  variableId: string | undefined,
  variables: Variable[],
): string | undefined {
  if (!variableId) return undefined;
  const variable = variables.find((item) => item.id === variableId);
  if (!variable) return undefined;
  return sanitizeExpressionIdentifier(variable.name) || undefined;
}

/** Rewrites a legacy `*VariableId` reference as an expression, in place. */
function adoptLegacyId<T extends Record<string, any>>(
  target: T,
  idField: string,
  expressionField: string,
  variables: Variable[],
): T {
  const legacyId = target[idField];
  if (!legacyId) return target;

  const next = { ...target } as Record<string, any>;
  if (!next[expressionField]) {
    const expression = legacyExpressionFor(legacyId, variables);
    if (expression) next[expressionField] = expression;
  }
  delete next[idField];
  return next as T;
}

export function migrateEventMarker(
  marker: EventMarker,
  variables: Variable[],
): EventMarker {
  let next = marker;
  next = adoptLegacyId(next, "positionVariableId", "positionExpression", variables);
  next = adoptLegacyId(next, "triggerMsVariableId", "triggerMsExpression", variables);
  next = adoptLegacyId(next, "poseXVariableId", "poseXExpression", variables);
  next = adoptLegacyId(next, "poseYVariableId", "poseYExpression", variables);
  next = adoptLegacyId(next, "durationVariableId", "durationExpression", variables);
  return next;
}

export function migrateLine(line: Line, variables: Variable[]): Line {
  let next = adoptLegacyId(line, "speedVariableId", "speedExpression", variables);

  if (next.eventMarkers?.length) {
    next = {
      ...next,
      eventMarkers: next.eventMarkers.map((marker) =>
        migrateEventMarker(marker, variables),
      ),
    };
  }

  return next;
}

export function migrateSequenceItem(
  item: SequenceItem,
  variables: Variable[],
): SequenceItem {
  if (item.kind === "wait" || item.kind === "event") {
    return adoptLegacyId(item, "durationVariableId", "durationExpression", variables);
  }
  if (item.kind === "repeat") {
    return adoptLegacyId(item, "countVariableId", "countExpression", variables);
  }
  return item;
}

/* -------------------------------------------------------------------------
 * Expression resolution for path data
 *
 * Every expression-bearing field keeps its literal alongside the expression;
 * the literal is what the canvas, animation, and time estimates read, so these
 * helpers recompute the literals whenever variables change.
 * ---------------------------------------------------------------------- */

const clampSpeed = (value: number) => Math.max(0.05, Math.min(1, value));
const clampMs = (value: number) => Math.max(0, Math.round(value));
const clampRepeatCount = (value: number) =>
  Math.max(1, Math.min(20, Math.round(value)));

/** Values above 1 are read as percentages, matching the old picker behaviour. */
const clampParametricPosition = (value: number) =>
  Math.max(0, Math.min(1, value > 1 ? value / 100 : value));

function resolveField(
  expression: string | undefined,
  fallback: number,
  variables: Variable[],
  scope: ExpressionScope,
  clamp: (value: number) => number = (value) => value,
): number {
  const resolved = resolveNumberExpression(expression, fallback, variables, scope);
  return resolved.valid ? clamp(resolved.value) : fallback;
}

export function resolveEventMarkerExpressions(
  marker: EventMarker,
  variables: Variable[],
  scope: ExpressionScope = buildExpressionScope(variables),
): EventMarker {
  return {
    ...marker,
    position: resolveField(
      marker.positionExpression,
      marker.position,
      variables,
      scope,
      clampParametricPosition,
    ),
    triggerMs: resolveField(
      marker.triggerMsExpression,
      marker.triggerMs ?? 0,
      variables,
      scope,
      clampMs,
    ),
    poseX: resolveField(marker.poseXExpression, marker.poseX ?? 0, variables, scope),
    poseY: resolveField(marker.poseYExpression, marker.poseY ?? 0, variables, scope),
    durationMs: resolveField(
      marker.durationExpression,
      marker.durationMs ?? 0,
      variables,
      scope,
      clampMs,
    ),
  };
}

export function resolveLineExpressions(
  line: Line,
  variables: Variable[],
  scope?: ExpressionScope,
): Line {
  const activeScope = scope || buildExpressionScope(variables);

  const next: Line = {
    ...line,
    // A point bound to a pose variable takes its coordinates from the pose.
    endPoint: line.endPoint?.poseVariableId
      ? line.endPoint
      : resolvePointExpressions(line.endPoint, variables, activeScope),
    controlPoints: (line.controlPoints || []).map((point) =>
      resolveBasePointExpressions(point, variables, activeScope),
    ),
    speed: resolveField(
      line.speedExpression,
      line.speed ?? 1,
      variables,
      activeScope,
      clampSpeed,
    ),
  };

  if (line.waitBeforeExpression) {
    next.waitBeforeMs = resolveField(
      line.waitBeforeExpression,
      line.waitBeforeMs ?? 0,
      variables,
      activeScope,
      clampMs,
    );
  }

  if (line.waitAfterExpression) {
    next.waitAfterMs = resolveField(
      line.waitAfterExpression,
      line.waitAfterMs ?? 0,
      variables,
      activeScope,
      clampMs,
    );
  }

  if (line.waitBefore?.durationExpression) {
    next.waitBefore = {
      ...line.waitBefore,
      durationMs: resolveField(
        line.waitBefore.durationExpression,
        line.waitBefore.durationMs,
        variables,
        activeScope,
        clampMs,
      ),
    };
  }

  if (line.waitAfter?.durationExpression) {
    next.waitAfter = {
      ...line.waitAfter,
      durationMs: resolveField(
        line.waitAfter.durationExpression,
        line.waitAfter.durationMs,
        variables,
        activeScope,
        clampMs,
      ),
    };
  }

  if (line.eventMarkers?.length) {
    next.eventMarkers = line.eventMarkers.map((marker) =>
      resolveEventMarkerExpressions(marker, variables, activeScope),
    );
  }

  return next;
}

export function resolveSequenceItemExpressions(
  item: SequenceItem,
  variables: Variable[],
  scope: ExpressionScope,
): SequenceItem {
  if (item.kind === "wait" || item.kind === "event") {
    return {
      ...item,
      durationMs: resolveField(
        item.durationExpression,
        item.durationMs,
        variables,
        scope,
        clampMs,
      ),
    };
  }

  if (item.kind === "repeat") {
    return {
      ...item,
      count: resolveField(
        item.countExpression,
        item.count,
        variables,
        scope,
        clampRepeatCount,
      ),
    };
  }

  return item;
}

export function resolveSequenceExpressions(
  sequence: SequenceItem[],
  variables: Variable[],
  scope?: ExpressionScope,
): SequenceItem[] {
  const activeScope = scope || buildExpressionScope(variables);
  return sequence.map((item) =>
    resolveSequenceItemExpressions(item, variables, activeScope),
  );
}

/** False only when an `enabledExpression` resolves to false. */
export function isEnabled(
  target: { enabledExpression?: string } | undefined,
  variables: Variable[],
  scope?: ExpressionScope,
): boolean {
  if (!target?.enabledExpression?.trim()) return true;
  return resolveBooleanExpression(
    target.enabledExpression,
    true,
    variables,
    scope,
  ).value;
}

/** True when an `if` block's condition currently holds. */
export function isConditionalActive(
  item: SequenceConditionalItem,
  variables: Variable[] = [],
  scope?: ExpressionScope,
): boolean {
  if (!item.condition?.trim()) return true;
  return resolveBooleanExpression(item.condition, true, variables, scope).value;
}

/**
 * Paths whose own condition is currently false.
 *
 * Membership of an `if` block is deliberately not counted here: the block
 * header already reports whether it is running, so dimming the paths inside
 * it as well just reads as "disabled" and is confusing.
 */
export function skippedLineIds(
  lines: Line[] = [],
  variables: Variable[] = [],
  scope?: ExpressionScope,
): Set<string> {
  const activeScope = scope || buildExpressionScope(variables);
  const skipped = new Set<string>();

  lines.forEach((line) => {
    if (!line.id) return;
    if (!isEnabled(line, variables, activeScope)) skipped.add(line.id);
  });

  return skipped;
}

type LegacyVariableSource = {
  variables?: Variable[];
  numberVariables?: Omit<NumberVariable, "type">[];
  poseVariables?: Omit<PoseVariable, "type">[];
  pathVariables?: Omit<PathVariable, "type">[];
};

/**
 * Produces the unified variable list from either the current `variables` field
 * or the three legacy arrays. Entries missing a `type` are tagged by shape.
 */
export function migrateVariables(source: LegacyVariableSource): Variable[] {
  const merged: Variable[] = [];
  const seenIds = new Set<string>();

  const push = (variable: Variable | undefined) => {
    if (!variable || !variable.id) return;
    if (seenIds.has(variable.id)) return;
    seenIds.add(variable.id);
    merged.push(variable);
  };

  (source.variables || []).forEach((variable) => {
    if (!variable) return;
    push(inferVariableType(variable));
  });

  (source.numberVariables || []).forEach((variable) =>
    push({ ...variable, type: "number", value: Number(variable.value) || 0 }),
  );

  (source.poseVariables || []).forEach((variable) =>
    push({ ...variable, type: "pose" } as PoseVariable),
  );

  (source.pathVariables || []).forEach((variable) =>
    push({ ...variable, type: "path" } as PathVariable),
  );

  return merged;
}

/** Tags a variable that predates the `type` field by inspecting its shape. */
function inferVariableType(input: Variable): Variable {
  const candidate = input as unknown as Record<string, unknown>;
  if (candidate.type) return input;

  if (Array.isArray(candidate.lines))
    return { ...candidate, type: "path" } as unknown as PathVariable;
  if ("heading" in candidate && "x" in candidate)
    return { ...candidate, type: "pose" } as unknown as PoseVariable;
  if (typeof candidate.value === "boolean")
    return { ...candidate, type: "boolean" } as unknown as BooleanVariable;

  return {
    ...candidate,
    type: "number",
    value: Number(candidate.value) || 0,
  } as unknown as NumberVariable;
}
