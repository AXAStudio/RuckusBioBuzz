import type {
  BasePoint,
  Point,
  PoseVariable,
  ScalarVariable,
  Variable,
} from "../types";

/**
 * Expression engine for the visualizer.
 *
 * Any numeric or boolean field in the editor can hold an expression instead of
 * a literal. Expressions may reference variables by name, read pose members
 * (`basket.x`), call math helpers, compare values, and branch with `?:`.
 *
 * Grammar (loosest to tightest binding):
 *   ternary    := logicalOr ("?" ternary ":" ternary)?
 *   logicalOr  := logicalAnd ("||" logicalAnd)*
 *   logicalAnd := equality ("&&" equality)*
 *   equality   := relational (("==" | "!=") relational)*
 *   relational := additive (("<" | "<=" | ">" | ">=") additive)*
 *   additive   := multiplicative (("+" | "-") multiplicative)*
 *   multiplicative := unary (("*" | "/" | "%") unary)*
 *   unary      := ("+" | "-" | "!") unary | primary
 *   primary    := number | boolean | call | identifier | "(" ternary ")"
 */

export type ExpressionValue = number | boolean;

type ArithmeticOperator = "+" | "-" | "*" | "/" | "%";
type ComparisonOperator = "<" | "<=" | ">" | ">=" | "==" | "!=";
type LogicalOperator = "&&" | "||";
type BinaryOperator = ArithmeticOperator | ComparisonOperator | LogicalOperator;

type ExpressionNode =
  | { type: "number"; value: number; source: string }
  | { type: "boolean"; value: boolean }
  | { type: "identifier"; name: string; member?: string }
  | { type: "unary"; operator: "+" | "-" | "!"; value: ExpressionNode }
  | {
      type: "binary";
      operator: BinaryOperator;
      left: ExpressionNode;
      right: ExpressionNode;
    }
  | {
      type: "ternary";
      condition: ExpressionNode;
      whenTrue: ExpressionNode;
      whenFalse: ExpressionNode;
    }
  | { type: "call"; name: string; args: ExpressionNode[] };

type ParsedExpression = { ok: true; node: ExpressionNode } | { ok: false };

type ExpressionResolution = {
  rawExpression?: string;
  value: number;
  valid: boolean;
};

type BooleanExpressionResolution = {
  rawExpression?: string;
  value: boolean;
  valid: boolean;
};

type PoseValue = { x: number; y: number; heading: number };

/** Everything an expression can see while evaluating. */
export type ExpressionScope = {
  scalars: Map<string, ExpressionValue>;
  poses: Map<string, PoseValue>;
};

type PoseExpressionField = "x" | "y" | "heading";
type PointCoordinateField = "x" | "y";
type PointHeadingField = "degrees" | "startDeg" | "endDeg";

const POSE_MEMBERS = new Set(["x", "y", "heading", "h", "deg", "degrees"]);

/** name -> [arity, Java rendering] */
const FUNCTIONS: Record<
  string,
  { arity: number | "variadic"; evaluate: (args: number[]) => number; java: string }
> = {
  abs: { arity: 1, evaluate: ([a]) => Math.abs(a), java: "Math.abs" },
  min: { arity: "variadic", evaluate: (args) => Math.min(...args), java: "Math.min" },
  max: { arity: "variadic", evaluate: (args) => Math.max(...args), java: "Math.max" },
  sqrt: { arity: 1, evaluate: ([a]) => Math.sqrt(a), java: "Math.sqrt" },
  pow: { arity: 2, evaluate: ([a, b]) => Math.pow(a, b), java: "Math.pow" },
  round: { arity: 1, evaluate: ([a]) => Math.round(a), java: "Math.round" },
  floor: { arity: 1, evaluate: ([a]) => Math.floor(a), java: "Math.floor" },
  ceil: { arity: 1, evaluate: ([a]) => Math.ceil(a), java: "Math.ceil" },
  sin: { arity: 1, evaluate: ([a]) => Math.sin(a), java: "Math.sin" },
  cos: { arity: 1, evaluate: ([a]) => Math.cos(a), java: "Math.cos" },
  tan: { arity: 1, evaluate: ([a]) => Math.tan(a), java: "Math.tan" },
  atan2: { arity: 2, evaluate: ([a, b]) => Math.atan2(a, b), java: "Math.atan2" },
  hypot: { arity: 2, evaluate: ([a, b]) => Math.hypot(a, b), java: "Math.hypot" },
  toradians: {
    arity: 1,
    evaluate: ([a]) => (a * Math.PI) / 180,
    java: "Math.toRadians",
  },
  todegrees: {
    arity: 1,
    evaluate: ([a]) => (a * 180) / Math.PI,
    java: "Math.toDegrees",
  },
};

const CONSTANTS: Record<string, { value: number; java: string }> = {
  pi: { value: Math.PI, java: "Math.PI" },
  e: { value: Math.E, java: "Math.E" },
};

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

/** Every alias a variable can be referenced by inside an expression. */
export function expressionAliases(name: string): string[] {
  const aliases: string[] = [];
  const trimmed = (name || "").trim();

  if (/^[A-Za-z_][A-Za-z0-9_]*$/.test(trimmed)) {
    aliases.push(trimmed.toLowerCase());
  }

  const sanitized = sanitizeExpressionIdentifier(trimmed);
  if (sanitized) aliases.push(sanitized.toLowerCase());

  return [...new Set(aliases)];
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

  function match(token: string): boolean {
    skipWhitespace();
    if (source.startsWith(token, index)) {
      index += token.length;
      return true;
    }
    return false;
  }

  function peek(token: string): boolean {
    skipWhitespace();
    return source.startsWith(token, index);
  }

  function parsePrimary(): ExpressionNode | null {
    skipWhitespace();
    const char = source[index];

    if (char === "(") {
      index++;
      const expression = parseTernary();
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
      const name = source.slice(start, index);
      const lowered = name.toLowerCase();

      if (lowered === "true") return { type: "boolean", value: true };
      if (lowered === "false") return { type: "boolean", value: false };

      // Function call
      skipWhitespace();
      if (source[index] === "(") {
        index++;
        const args: ExpressionNode[] = [];
        skipWhitespace();
        if (source[index] === ")") {
          index++;
        } else {
          while (true) {
            const arg = parseTernary();
            if (!arg) return null;
            args.push(arg);
            skipWhitespace();
            if (source[index] === ",") {
              index++;
              continue;
            }
            if (source[index] === ")") {
              index++;
              break;
            }
            return null;
          }
        }
        return { type: "call", name: lowered, args };
      }

      // Member access (pose fields)
      if (source[index] === "." && isIdentifierStart(source[index + 1] || "")) {
        index++;
        const memberStart = index;
        while (index < source.length && isIdentifierPart(source[index])) {
          index++;
        }
        return {
          type: "identifier",
          name,
          member: source.slice(memberStart, index).toLowerCase(),
        };
      }

      return { type: "identifier", name };
    }

    return null;
  }

  function parseUnary(): ExpressionNode | null {
    skipWhitespace();
    const char = source[index];
    // Guard against consuming "!=" as a unary "!"
    if (char === "!" && source[index + 1] !== "=") {
      index++;
      const value = parseUnary();
      if (!value) return null;
      return { type: "unary", operator: "!", value };
    }
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
      const operator = source[index] as ArithmeticOperator;
      if (operator !== "*" && operator !== "/" && operator !== "%") break;
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
      const operator = source[index] as ArithmeticOperator;
      if (operator !== "+" && operator !== "-") break;
      index++;
      const right = parseMultiplicative();
      if (!right) return null;
      left = { type: "binary", operator, left, right };
    }

    return left;
  }

  function parseRelational(): ExpressionNode | null {
    let left = parseAdditive();
    if (!left) return null;

    while (true) {
      skipWhitespace();
      let operator: ComparisonOperator | null = null;
      if (peek("<=")) operator = "<=";
      else if (peek(">=")) operator = ">=";
      else if (peek("<")) operator = "<";
      else if (peek(">")) operator = ">";
      if (!operator) break;
      index += operator.length;
      const right = parseAdditive();
      if (!right) return null;
      left = { type: "binary", operator, left, right };
    }

    return left;
  }

  function parseEquality(): ExpressionNode | null {
    let left = parseRelational();
    if (!left) return null;

    while (true) {
      let operator: ComparisonOperator | null = null;
      if (peek("==")) operator = "==";
      else if (peek("!=")) operator = "!=";
      if (!operator) break;
      index += operator.length;
      const right = parseRelational();
      if (!right) return null;
      left = { type: "binary", operator, left, right };
    }

    return left;
  }

  function parseLogicalAnd(): ExpressionNode | null {
    let left = parseEquality();
    if (!left) return null;

    while (match("&&")) {
      const right = parseEquality();
      if (!right) return null;
      left = { type: "binary", operator: "&&", left, right };
    }

    return left;
  }

  function parseLogicalOr(): ExpressionNode | null {
    let left = parseLogicalAnd();
    if (!left) return null;

    while (match("||")) {
      const right = parseLogicalAnd();
      if (!right) return null;
      left = { type: "binary", operator: "||", left, right };
    }

    return left;
  }

  function parseTernary(): ExpressionNode | null {
    const condition = parseLogicalOr();
    if (!condition) return null;

    if (!match("?")) return condition;

    const whenTrue = parseTernary();
    if (!whenTrue) return null;
    if (!match(":")) return null;
    const whenFalse = parseTernary();
    if (!whenFalse) return null;

    return { type: "ternary", condition, whenTrue, whenFalse };
  }

  const node = parseTernary();
  skipWhitespace();
  if (!node || index !== source.length) return { ok: false };
  return { ok: true, node };
}

function toNumber(value: ExpressionValue | null): number | null {
  if (value === null) return null;
  if (typeof value === "boolean") return value ? 1 : 0;
  return Number.isFinite(value) ? value : null;
}

function toBoolean(value: ExpressionValue | null): boolean | null {
  if (value === null) return null;
  if (typeof value === "boolean") return value;
  return value !== 0;
}

function poseMemberValue(pose: PoseValue, member: string): number | null {
  if (member === "x") return pose.x;
  if (member === "y") return pose.y;
  if (member === "heading" || member === "h" || member === "deg" || member === "degrees")
    return pose.heading;
  return null;
}

function evaluateNode(
  node: ExpressionNode,
  scope: ExpressionScope,
): ExpressionValue | null {
  switch (node.type) {
    case "number":
      return node.value;

    case "boolean":
      return node.value;

    case "identifier": {
      const key = node.name.toLowerCase();
      if (node.member) {
        const pose = scope.poses.get(key);
        if (!pose) return null;
        return poseMemberValue(pose, node.member);
      }
      const scalar = scope.scalars.get(key);
      if (scalar !== undefined) return scalar;
      const constant = CONSTANTS[key];
      return constant ? constant.value : null;
    }

    case "unary": {
      if (node.operator === "!") {
        const value = toBoolean(evaluateNode(node.value, scope));
        return value === null ? null : !value;
      }
      const value = toNumber(evaluateNode(node.value, scope));
      if (value === null) return null;
      return node.operator === "-" ? -value : value;
    }

    case "ternary": {
      const condition = toBoolean(evaluateNode(node.condition, scope));
      if (condition === null) return null;
      return evaluateNode(condition ? node.whenTrue : node.whenFalse, scope);
    }

    case "call": {
      const fn = FUNCTIONS[node.name];
      if (!fn) return null;
      if (fn.arity !== "variadic" && node.args.length !== fn.arity) return null;
      if (fn.arity === "variadic" && node.args.length === 0) return null;

      const args: number[] = [];
      for (const arg of node.args) {
        const value = toNumber(evaluateNode(arg, scope));
        if (value === null) return null;
        args.push(value);
      }
      const result = fn.evaluate(args);
      return Number.isFinite(result) ? result : null;
    }

    case "binary": {
      if (node.operator === "&&" || node.operator === "||") {
        const left = toBoolean(evaluateNode(node.left, scope));
        if (left === null) return null;
        // Short-circuit, matching Java semantics.
        if (node.operator === "&&" && !left) return false;
        if (node.operator === "||" && left) return true;
        return toBoolean(evaluateNode(node.right, scope));
      }

      const leftRaw = evaluateNode(node.left, scope);
      const rightRaw = evaluateNode(node.right, scope);
      if (leftRaw === null || rightRaw === null) return null;

      if (node.operator === "==" || node.operator === "!=") {
        // Compare booleans directly; otherwise compare numerically.
        const bothBoolean =
          typeof leftRaw === "boolean" && typeof rightRaw === "boolean";
        const equal = bothBoolean
          ? leftRaw === rightRaw
          : toNumber(leftRaw) === toNumber(rightRaw);
        return node.operator === "==" ? equal : !equal;
      }

      const left = toNumber(leftRaw);
      const right = toNumber(rightRaw);
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
        case "%":
          return Math.abs(right) < 1e-9 ? null : left % right;
        case "<":
          return left < right;
        case "<=":
          return left <= right;
        case ">":
          return left > right;
        case ">=":
          return left >= right;
      }
      return null;
    }
  }
}

function isScalarVariable(variable: Variable): variable is ScalarVariable {
  return variable.type === "number" || variable.type === "boolean";
}

function isPoseVariable(variable: Variable): variable is PoseVariable {
  return variable.type === "pose";
}

/**
 * Builds the evaluation scope from the variable list.
 *
 * Variables may reference other variables, so each one is resolved lazily with
 * a "resolving" set guarding against reference cycles (a cycle yields an
 * unresolved variable rather than infinite recursion).
 */
export function buildExpressionScope(
  variables: Variable[] | undefined,
): ExpressionScope {
  const scope: ExpressionScope = { scalars: new Map(), poses: new Map() };
  const list = Array.isArray(variables) ? variables : [];

  const byAlias = new Map<string, Variable>();
  list.forEach((variable) => {
    expressionAliases(variable.name).forEach((alias) => {
      if (!byAlias.has(alias)) byAlias.set(alias, variable);
    });
  });

  const resolved = new Map<string, ExpressionValue | PoseValue | null>();
  const resolving = new Set<string>();

  function evaluateWithLazyScope(node: ExpressionNode): ExpressionValue | null {
    const lazyScope: ExpressionScope = {
      scalars: {
        get: (key: string) => {
          const value = resolveVariable(key);
          return value !== null && typeof value !== "object"
            ? (value as ExpressionValue)
            : undefined;
        },
      } as unknown as Map<string, ExpressionValue>,
      poses: {
        get: (key: string) => {
          const value = resolveVariable(key);
          return value !== null && typeof value === "object"
            ? (value as PoseValue)
            : undefined;
        },
      } as unknown as Map<string, PoseValue>,
    };
    return evaluateNode(node, lazyScope);
  }

  function evaluateField(
    expression: string | undefined,
    fallback: number,
  ): number {
    const raw = expression?.trim();
    if (!raw) return Number.isFinite(fallback) ? fallback : 0;
    const parsed = parseExpression(raw);
    if (!parsed.ok) return Number.isFinite(fallback) ? fallback : 0;
    const value = toNumber(evaluateWithLazyScope(parsed.node));
    return value === null ? (Number.isFinite(fallback) ? fallback : 0) : value;
  }

  function resolveVariable(alias: string): ExpressionValue | PoseValue | null {
    const key = alias.toLowerCase();
    if (resolved.has(key)) return resolved.get(key)!;

    const variable = byAlias.get(key);
    if (!variable) return null;

    if (resolving.has(key)) return null; // cycle
    resolving.add(key);

    let value: ExpressionValue | PoseValue | null = null;

    if (isScalarVariable(variable)) {
      const raw = variable.valueExpression?.trim();
      if (raw) {
        const parsed = parseExpression(raw);
        value = parsed.ok ? evaluateWithLazyScope(parsed.node) : null;
      }
      if (value === null) {
        value =
          variable.type === "boolean"
            ? Boolean(variable.value)
            : Number.isFinite(Number(variable.value))
              ? Number(variable.value)
              : 0;
      }
      if (variable.type === "boolean") value = toBoolean(value) ?? false;
    } else if (isPoseVariable(variable)) {
      value = {
        x: evaluateField(variable.xExpression, Number(variable.x) || 0),
        y: evaluateField(variable.yExpression, Number(variable.y) || 0),
        heading: evaluateField(
          variable.headingExpression,
          Number(variable.heading) || 0,
        ),
      };
    }

    resolving.delete(key);
    resolved.set(key, value);
    return value;
  }

  // Materialize every alias into concrete maps for fast repeated evaluation.
  byAlias.forEach((variable, alias) => {
    const value = resolveVariable(alias);
    if (value === null) return;
    if (typeof value === "object") {
      scope.poses.set(alias, value);
    } else {
      scope.scalars.set(alias, value);
    }
  });

  return scope;
}

/** Evaluate an expression to a raw value; `null` when it cannot be resolved. */
export function evaluateExpression(
  expression: string | undefined,
  variables: Variable[] | undefined,
  scope?: ExpressionScope,
): ExpressionValue | null {
  const raw = expression?.trim();
  if (!raw) return null;
  const parsed = parseExpression(raw);
  if (!parsed.ok) return null;
  return evaluateNode(parsed.node, scope || buildExpressionScope(variables));
}

/** True when the expression parses and every referenced name resolves. */
export function isExpressionValid(
  expression: string | undefined,
  variables: Variable[] | undefined,
  scope?: ExpressionScope,
): boolean {
  const raw = expression?.trim();
  if (!raw) return true;
  return evaluateExpression(raw, variables, scope) !== null;
}

export function resolveNumberExpression(
  expression: string | undefined,
  fallbackValue: number,
  variables: Variable[] | undefined,
  scope?: ExpressionScope,
): ExpressionResolution {
  const rawExpression = expression?.trim();
  const fallback = Number.isFinite(fallbackValue) ? fallbackValue : 0;

  if (!rawExpression) {
    return { rawExpression: undefined, value: fallback, valid: true };
  }

  const evaluated = evaluateExpression(rawExpression, variables, scope);
  const numeric = toNumber(evaluated);

  if (numeric === null) {
    return { rawExpression, value: fallback, valid: false };
  }

  return { rawExpression, value: numeric, valid: true };
}

export function resolveBooleanExpression(
  expression: string | undefined,
  fallbackValue: boolean,
  variables: Variable[] | undefined,
  scope?: ExpressionScope,
): BooleanExpressionResolution {
  const rawExpression = expression?.trim();

  if (!rawExpression) {
    return {
      rawExpression: undefined,
      value: Boolean(fallbackValue),
      valid: true,
    };
  }

  const evaluated = evaluateExpression(rawExpression, variables, scope);
  const boolean = toBoolean(evaluated);

  if (boolean === null) {
    return { rawExpression, value: Boolean(fallbackValue), valid: false };
  }

  return { rawExpression, value: boolean, valid: true };
}

/** Resolves each variable's own expression fields against the others. */
export function resolveVariableValues(variables: Variable[]): Variable[] {
  const scope = buildExpressionScope(variables);

  return variables.map((variable) => {
    if (variable.type === "number") {
      const resolved = resolveNumberExpression(
        variable.valueExpression,
        Number(variable.value) || 0,
        variables,
        scope,
      );
      return { ...variable, value: resolved.value };
    }

    if (variable.type === "boolean") {
      const resolved = resolveBooleanExpression(
        variable.valueExpression,
        Boolean(variable.value),
        variables,
        scope,
      );
      return { ...variable, value: resolved.value };
    }

    if (variable.type === "pose") {
      return {
        ...variable,
        x: resolveNumberExpression(
          variable.xExpression,
          Number(variable.x) || 0,
          variables,
          scope,
        ).value,
        y: resolveNumberExpression(
          variable.yExpression,
          Number(variable.y) || 0,
          variables,
          scope,
        ).value,
        heading: resolveNumberExpression(
          variable.headingExpression,
          Number(variable.heading) || 0,
          variables,
          scope,
        ).value,
      };
    }

    return variable;
  });
}

export function resolvePoseVariableExpressions(
  poseVariables: PoseVariable[],
  variables: Variable[],
): PoseVariable[] {
  const scope = buildExpressionScope(variables);

  return poseVariables.map((variable) => {
    const resolvedX = resolveNumberExpression(
      variable.xExpression,
      Number(variable.x) || 0,
      variables,
      scope,
    );
    const resolvedY = resolveNumberExpression(
      variable.yExpression,
      Number(variable.y) || 0,
      variables,
      scope,
    );
    const resolvedHeading = resolveNumberExpression(
      variable.headingExpression,
      Number(variable.heading) || 0,
      variables,
      scope,
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

/** Shows the raw expression when present, otherwise the numeric value. */
export function expressionDisplayValue(
  expression: string | undefined,
  value: number | undefined,
  fallback = "0",
): string {
  if (typeof expression === "string" && expression.trim()) return expression;
  const numeric = Number(value);
  return Number.isFinite(numeric) ? String(numeric) : fallback;
}

export function booleanExpressionDisplayValue(
  expression: string | undefined,
  value: boolean | undefined,
): string {
  if (typeof expression === "string" && expression.trim()) return expression;
  return value ? "true" : "false";
}

export function poseVariableFieldDisplayValue(
  variable: PoseVariable,
  field: PoseExpressionField,
): string {
  const expressionField = `${field}Expression` as const;
  return expressionDisplayValue(variable[expressionField], variable[field]);
}

export function pointCoordinateFieldDisplayValue(
  point: BasePoint,
  field: PointCoordinateField,
): string {
  const expressionField = `${field}Expression` as const;
  return expressionDisplayValue(point[expressionField], point[field]);
}

export function pointHeadingFieldDisplayValue(
  point: Point,
  field: PointHeadingField,
): string {
  const expressionField = `${field}Expression` as const;
  return expressionDisplayValue(point[expressionField], point[field]);
}

export function resolveBasePointExpressions<T extends BasePoint>(
  point: T,
  variables: Variable[],
  scope?: ExpressionScope,
): T {
  const activeScope = scope || buildExpressionScope(variables);
  const resolvedX = resolveNumberExpression(
    point.xExpression,
    Number(point.x) || 0,
    variables,
    activeScope,
  );
  const resolvedY = resolveNumberExpression(
    point.yExpression,
    Number(point.y) || 0,
    variables,
    activeScope,
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
  variables: Variable[],
  scope?: ExpressionScope,
): Point {
  const activeScope = scope || buildExpressionScope(variables);
  const base = resolveBasePointExpressions(point, variables, activeScope);

  if (base.heading === "constant") {
    const resolved = resolveNumberExpression(
      base.degreesExpression,
      Number(base.degrees) || 0,
      variables,
      activeScope,
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
      variables,
      activeScope,
    );
    const resolvedEnd = resolveNumberExpression(
      base.endDegExpression,
      Number(base.endDeg) || 0,
      variables,
      activeScope,
    );
    const resolvedCurve = resolveNumberExpression(
      base.headingCurveExpression,
      Number(base.headingCurve) || 0,
      variables,
      activeScope,
    );
    return {
      ...base,
      startDeg: resolvedStart.value,
      endDeg: resolvedEnd.value,
      headingCurve: base.headingCurveExpression
        ? resolvedCurve.value
        : base.headingCurve,
      startDegExpression: resolvedStart.rawExpression,
      endDegExpression: resolvedEnd.rawExpression,
    };
  }

  if (base.heading === "tangential" && base.reverseExpression?.trim()) {
    const resolved = resolveBooleanExpression(
      base.reverseExpression,
      Boolean(base.reverse),
      variables,
      activeScope,
    );
    return { ...base, reverse: resolved.value };
  }

  return base;
}

/**
 * Mirrors an expression so it keeps tracking the variables it references.
 *
 * A field holding a plain number keeps holding a plain number: the caller
 * already mirrors the literal, and turning `56` into `(141.5 - (56))` would
 * leave the editor showing arithmetic instead of a coordinate, nesting one
 * layer deeper on every mirror.
 */
export function mirrorPoseExpression(
  expression: string | undefined,
  _numericValue: number,
  transform: (inner: string) => string,
): string | undefined {
  const rawExpression = expression?.trim();
  return rawExpression ? transform(rawExpression) : undefined;
}

/** Lower-cased variable names an expression references, for dependency order. */
export function expressionIdentifiers(
  expression: string | undefined,
): string[] {
  const raw = expression?.trim();
  if (!raw) return [];

  const parsed = parseExpression(raw);
  if (!parsed.ok) return [];

  const names = new Set<string>();

  const walk = (node: ExpressionNode) => {
    switch (node.type) {
      case "identifier":
        names.add(node.name.toLowerCase());
        break;
      case "unary":
        walk(node.value);
        break;
      case "binary":
        walk(node.left);
        walk(node.right);
        break;
      case "ternary":
        walk(node.condition);
        walk(node.whenTrue);
        walk(node.whenFalse);
        break;
      case "call":
        node.args.forEach(walk);
        break;
    }
  };

  walk(parsed.node);
  return [...names];
}

/** Java identifiers a rendered expression can reference. */
export type JavaIdentifierMap = {
  scalars: Map<string, string>;
  /** alias -> Java constant holding a PathStep. */
  poses: Map<string, string>;
};

export function buildJavaIdentifierMap(
  variables: Variable[],
  constantById: Map<string, string>,
): JavaIdentifierMap {
  const scalars = new Map<string, string>();
  const poses = new Map<string, string>();

  variables.forEach((variable) => {
    const constantName = constantById.get(variable.id);
    if (!constantName) return;

    const target = variable.type === "pose" ? poses : scalars;
    expressionAliases(variable.name).forEach((alias) => {
      if (!target.has(alias)) target.set(alias, constantName);
    });
  });

  return { scalars, poses };
}

function renderNodeToJava(
  node: ExpressionNode,
  identifiers: JavaIdentifierMap,
): string | null {
  switch (node.type) {
    case "number":
      return node.source;

    case "boolean":
      return node.value ? "true" : "false";

    case "identifier": {
      const key = node.name.toLowerCase();
      if (node.member) {
        const constant = identifiers.poses.get(key);
        if (!constant) return null;
        if (node.member === "x") return `${constant}.x`;
        if (node.member === "y") return `${constant}.y`;
        // PathStep stores heading in radians; expressions work in degrees.
        return `Math.toDegrees(${constant}.heading)`;
      }
      const scalar = identifiers.scalars.get(key);
      if (scalar) return scalar;
      const constant = CONSTANTS[key];
      return constant ? constant.java : null;
    }

    case "unary": {
      const value = renderNodeToJava(node.value, identifiers);
      if (value === null) return null;
      return `${node.operator}(${value})`;
    }

    case "ternary": {
      const condition = renderNodeToJava(node.condition, identifiers);
      const whenTrue = renderNodeToJava(node.whenTrue, identifiers);
      const whenFalse = renderNodeToJava(node.whenFalse, identifiers);
      if (condition === null || whenTrue === null || whenFalse === null)
        return null;
      return `((${condition}) ? (${whenTrue}) : (${whenFalse}))`;
    }

    case "call": {
      const fn = FUNCTIONS[node.name];
      if (!fn) return null;
      const args: string[] = [];
      for (const arg of node.args) {
        const rendered = renderNodeToJava(arg, identifiers);
        if (rendered === null) return null;
        args.push(rendered);
      }
      if (fn.arity === "variadic" && args.length > 2) {
        // Math.min/max take two arguments in Java; fold them.
        return args.reduce((acc, arg) => `${fn.java}(${acc}, ${arg})`);
      }
      return `${fn.java}(${args.join(", ")})`;
    }

    case "binary": {
      const left = renderNodeToJava(node.left, identifiers);
      const right = renderNodeToJava(node.right, identifiers);
      if (left === null || right === null) return null;
      return `(${left} ${node.operator} ${right})`;
    }
  }
}

/**
 * Renders an expression as Java source. Falls back to the literal value when
 * the expression is missing, unparseable, or references something with no Java
 * counterpart, so generated code always compiles.
 */
export function buildJavaExpression(
  expression: string | undefined,
  fallbackExpression: string,
  identifiers: JavaIdentifierMap,
): string {
  const rawExpression = expression?.trim();
  if (!rawExpression) return fallbackExpression;

  const parsed = parseExpression(rawExpression);
  if (!parsed.ok) return fallbackExpression;

  return renderNodeToJava(parsed.node, identifiers) ?? fallbackExpression;
}

/** Back-compat wrapper taking a plain variable list. */
export function buildJavaExpressionFromNumberExpression(
  expression: string | undefined,
  fallbackExpression: string,
  variables: Variable[],
  constantById: Map<string, string>,
): string {
  return buildJavaExpression(
    expression,
    fallbackExpression,
    buildJavaIdentifierMap(variables, constantById),
  );
}

/** Names offered by the "insert variable" dropdowns, in menu order. */
export function variableInsertOptions(
  variables: Variable[] | undefined,
): { label: string; token: string; type: string }[] {
  const options: { label: string; token: string; type: string }[] = [];

  (variables || []).forEach((variable) => {
    const [alias] = expressionAliases(variable.name);
    if (!alias) return;
    const token = sanitizeExpressionIdentifier(variable.name) || alias;

    if (variable.type === "pose") {
      options.push(
        { label: `${variable.name}.x`, token: `${token}.x`, type: "pose" },
        { label: `${variable.name}.y`, token: `${token}.y`, type: "pose" },
        {
          label: `${variable.name}.heading`,
          token: `${token}.heading`,
          type: "pose",
        },
      );
      return;
    }

    if (variable.type === "path") return; // Not referenceable in expressions.

    options.push({ label: variable.name, token, type: variable.type });
  });

  return options;
}
