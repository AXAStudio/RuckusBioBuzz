import type { BasePoint, NumberVariable, Point, PoseVariable } from "../types";

type Operator = "+" | "-" | "*" | "/";

type ExpressionNode =
  | { type: "number"; value: number; source: string }
  | { type: "identifier"; name: string }
  | { type: "unary"; operator: "+" | "-"; value: ExpressionNode }
  | {
      type: "binary";
      operator: Operator;
      left: ExpressionNode;
      right: ExpressionNode;
    };

type ParsedExpression =
  | { ok: true; node: ExpressionNode }
  | { ok: false };

type ExpressionResolution = {
  rawExpression?: string;
  value: number;
  valid: boolean;
};

type PoseExpressionField = "x" | "y" | "heading";
type PointCoordinateField = "x" | "y";
type PointHeadingField = "degrees" | "startDeg" | "endDeg";

function isIdentifierStart(char: string): boolean {
  return /[A-Za-z_]/.test(char);
}

function isIdentifierPart(char: string): boolean {
  return /[A-Za-z0-9_]/.test(char);
}

export function sanitizeExpressionIdentifier(name: string): string {
  const sanitized = name
    .trim()
    .replace(/[^A-Za-z0-9_]+/g, "_")
    .replace(/^_+/, "")
    .replace(/_+/g, "_");

  if (!sanitized) return "";
  return /^[A-Za-z_]/.test(sanitized) ? sanitized : `n_${sanitized}`;
}

function buildExpressionVariableMap(numberVariables: NumberVariable[]) {
  const aliases = new Map<string, number>();

  numberVariables.forEach((variable) => {
    const value = Number(variable.value);
    if (!Number.isFinite(value)) return;

    const trimmedName = variable.name.trim();
    if (/^[A-Za-z_][A-Za-z0-9_]*$/.test(trimmedName)) {
      aliases.set(trimmedName.toLowerCase(), value);
    }

    const sanitized = sanitizeExpressionIdentifier(trimmedName);
    if (sanitized) {
      aliases.set(sanitized.toLowerCase(), value);
    }
  });

  return aliases;
}

function parseExpression(input: string): ParsedExpression {
  const source = input.trim();
  if (!source) return { ok: false };

  let index = 0;

  function skipWhitespace() {
    while (index < source.length && /\s/.test(source[index])) {
      index++;
    }
  }

  function parsePrimary(): ExpressionNode | null {
    skipWhitespace();
    const char = source[index];

    if (char === "(") {
      index++;
      const expression = parseAdditive();
      skipWhitespace();
      if (!expression || source[index] !== ")") return null;
      index++;
      return expression;
    }

    if (char && (/\d/.test(char) || char === ".")) {
      const start = index;
      while (index < source.length && /[\d.]/.test(source[index])) {
        index++;
      }
      const token = source.slice(start, index);
      const value = Number(token);
      if (!Number.isFinite(value)) return null;
      return { type: "number", value, source: token };
    }

    if (char && isIdentifierStart(char)) {
      const start = index;
      index++;
      while (index < source.length && isIdentifierPart(source[index])) {
        index++;
      }
      return { type: "identifier", name: source.slice(start, index) };
    }

    return null;
  }

  function parseUnary(): ExpressionNode | null {
    skipWhitespace();
    const char = source[index];
    if (char === "+" || char === "-") {
      index++;
      const value = parseUnary();
      if (!value) return null;
      return { type: "unary", operator: char, value };
    }
    return parsePrimary();
  }

  function parseMultiplicative(): ExpressionNode | null {
    let left = parseUnary();
    if (!left) return null;

    while (true) {
      skipWhitespace();
      const operator = source[index] as Operator;
      if (operator !== "*" && operator !== "/") break;
      index++;
      const right = parseUnary();
      if (!right) return null;
      left = { type: "binary", operator, left, right };
    }

    return left;
  }

  function parseAdditive(): ExpressionNode | null {
    let left = parseMultiplicative();
    if (!left) return null;

    while (true) {
      skipWhitespace();
      const operator = source[index] as Operator;
      if (operator !== "+" && operator !== "-") break;
      index++;
      const right = parseMultiplicative();
      if (!right) return null;
      left = { type: "binary", operator, left, right };
    }

    return left;
  }

  const node = parseAdditive();
  skipWhitespace();
  if (!node || index !== source.length) return { ok: false };
  return { ok: true, node };
}

function evaluateNode(
  node: ExpressionNode,
  variableValues: Map<string, number>,
): number | null {
  if (node.type === "number") return node.value;
  if (node.type === "identifier") {
    const value = variableValues.get(node.name.toLowerCase());
    return Number.isFinite(value) ? value! : null;
  }
  if (node.type === "unary") {
    const value = evaluateNode(node.value, variableValues);
    if (value === null) return null;
    return node.operator === "-" ? -value : value;
  }

  const left = evaluateNode(node.left, variableValues);
  const right = evaluateNode(node.right, variableValues);
  if (left === null || right === null) return null;

  switch (node.operator) {
    case "+":
      return left + right;
    case "-":
      return left - right;
    case "*":
      return left * right;
    case "/":
      return Math.abs(right) < 1e-9 ? null : left / right;
  }
}

function renderNodeToJava(
  node: ExpressionNode,
  identifierToJava: Map<string, string>,
): string | null {
  if (node.type === "number") return node.source;
  if (node.type === "identifier") {
    return identifierToJava.get(node.name.toLowerCase()) || null;
  }
  if (node.type === "unary") {
    const value = renderNodeToJava(node.value, identifierToJava);
    if (!value) return null;
    return `${node.operator}(${value})`;
  }

  const left = renderNodeToJava(node.left, identifierToJava);
  const right = renderNodeToJava(node.right, identifierToJava);
  if (!left || !right) return null;
  return `(${left} ${node.operator} ${right})`;
}

export function resolveNumberExpression(
  expression: string | undefined,
  fallbackValue: number,
  numberVariables: NumberVariable[],
): ExpressionResolution {
  const rawExpression = expression?.trim();
  if (!rawExpression) {
    return {
      rawExpression: undefined,
      value: Number.isFinite(fallbackValue) ? fallbackValue : 0,
      valid: true,
    };
  }

  const parsed = parseExpression(rawExpression);
  if (!parsed.ok) {
    return {
      rawExpression,
      value: Number.isFinite(fallbackValue) ? fallbackValue : 0,
      valid: false,
    };
  }

  const evaluated = evaluateNode(
    parsed.node,
    buildExpressionVariableMap(numberVariables),
  );
  if (!Number.isFinite(evaluated)) {
    return {
      rawExpression,
      value: Number.isFinite(fallbackValue) ? fallbackValue : 0,
      valid: false,
    };
  }

  return {
    rawExpression,
    value: evaluated!,
    valid: true,
  };
}

export function resolvePoseVariableExpressions(
  poseVariables: PoseVariable[],
  numberVariables: NumberVariable[],
): PoseVariable[] {
  return poseVariables.map((variable) => {
    const resolvedX = resolveNumberExpression(
      variable.xExpression,
      Number(variable.x) || 0,
      numberVariables,
    );
    const resolvedY = resolveNumberExpression(
      variable.yExpression,
      Number(variable.y) || 0,
      numberVariables,
    );
    const resolvedHeading = resolveNumberExpression(
      variable.headingExpression,
      Number(variable.heading) || 0,
      numberVariables,
    );

    return {
      ...variable,
      x: resolvedX.value,
      y: resolvedY.value,
      heading: resolvedHeading.value,
      xExpression: resolvedX.rawExpression,
      yExpression: resolvedY.rawExpression,
      headingExpression: resolvedHeading.rawExpression,
    };
  });
}

export function poseVariableFieldDisplayValue(
  variable: PoseVariable,
  field: PoseExpressionField,
): string {
  const expressionField = `${field}Expression` as const;
  const expression = variable[expressionField];
  if (typeof expression === "string" && expression.trim()) {
    return expression;
  }
  const numeric = Number(variable[field]);
  return Number.isFinite(numeric) ? String(numeric) : "0";
}

export function pointCoordinateFieldDisplayValue(
  point: BasePoint,
  field: PointCoordinateField,
): string {
  const expressionField = `${field}Expression` as const;
  const expression = point[expressionField];
  if (typeof expression === "string" && expression.trim()) {
    return expression;
  }
  const numeric = Number(point[field]);
  return Number.isFinite(numeric) ? String(numeric) : "0";
}

export function pointHeadingFieldDisplayValue(
  point: Point,
  field: PointHeadingField,
): string {
  const expressionField = `${field}Expression` as const;
  const expression = point[expressionField];
  if (typeof expression === "string" && expression.trim()) {
    return expression;
  }
  const numeric = Number(point[field]);
  return Number.isFinite(numeric) ? String(numeric) : "0";
}

export function resolveBasePointExpressions<T extends BasePoint>(
  point: T,
  numberVariables: NumberVariable[],
): T {
  const resolvedX = resolveNumberExpression(
    point.xExpression,
    Number(point.x) || 0,
    numberVariables,
  );
  const resolvedY = resolveNumberExpression(
    point.yExpression,
    Number(point.y) || 0,
    numberVariables,
  );

  return {
    ...point,
    x: resolvedX.value,
    y: resolvedY.value,
    xExpression: resolvedX.rawExpression,
    yExpression: resolvedY.rawExpression,
  };
}

export function resolvePointExpressions(
  point: Point,
  numberVariables: NumberVariable[],
): Point {
  const base = resolveBasePointExpressions(point, numberVariables);

  if (base.heading === "constant") {
    const resolved = resolveNumberExpression(
      base.degreesExpression,
      Number(base.degrees) || 0,
      numberVariables,
    );
    return {
      ...base,
      degrees: resolved.value,
      degreesExpression: resolved.rawExpression,
    };
  }

  if (base.heading === "linear") {
    const resolvedStart = resolveNumberExpression(
      base.startDegExpression,
      Number(base.startDeg) || 0,
      numberVariables,
    );
    const resolvedEnd = resolveNumberExpression(
      base.endDegExpression,
      Number(base.endDeg) || 0,
      numberVariables,
    );
    return {
      ...base,
      startDeg: resolvedStart.value,
      endDeg: resolvedEnd.value,
      startDegExpression: resolvedStart.rawExpression,
      endDegExpression: resolvedEnd.rawExpression,
    };
  }

  return base;
}

export function mirrorPoseExpression(
  expression: string | undefined,
  numericValue: number,
  transform: (inner: string) => string,
): string | undefined {
  const rawExpression = expression?.trim();
  if (rawExpression) {
    return transform(rawExpression);
  }
  if (Number.isFinite(numericValue)) {
    return transform(String(numericValue));
  }
  return undefined;
}

export function buildJavaExpressionFromNumberExpression(
  expression: string | undefined,
  fallbackExpression: string,
  numberVariables: NumberVariable[],
  numberVariableConstantById: Map<string, string>,
): string {
  const rawExpression = expression?.trim();
  if (!rawExpression) return fallbackExpression;

  const parsed = parseExpression(rawExpression);
  if (!parsed.ok) return fallbackExpression;

  const identifierToJava = new Map<string, string>();
  numberVariables.forEach((variable) => {
    const constantName = numberVariableConstantById.get(variable.id);
    if (!constantName) return;
    const trimmedName = variable.name.trim();
    if (/^[A-Za-z_][A-Za-z0-9_]*$/.test(trimmedName)) {
      identifierToJava.set(trimmedName.toLowerCase(), constantName);
    }
    const sanitized = sanitizeExpressionIdentifier(trimmedName);
    if (sanitized) {
      identifierToJava.set(sanitized.toLowerCase(), constantName);
    }
  });

  return renderNodeToJava(parsed.node, identifierToJava) || fallbackExpression;
}
