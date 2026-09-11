export type JavaValidationIssue = {
  level: "error" | "warning";
  message: string;
};

type Delimiter = "{" | "(" | "[";

const closingToOpening = new Map<string, Delimiter>([
  ["}", "{"],
  [")", "("],
  ["]", "["],
]);

const openingToClosing = new Map<Delimiter, string>([
  ["{", "}"],
  ["(", ")"],
  ["[", "]"],
]);

export function validateJavaDelimiters(source: string): JavaValidationIssue | null {
  const stack: Array<{ char: Delimiter; line: number; column: number }> = [];
  let line = 1;
  let column = 0;
  let state: "code" | "lineComment" | "blockComment" | "string" | "char" | "textBlock" = "code";

  for (let i = 0; i < source.length; i++) {
    const char = source[i];
    const next = source[i + 1];
    const next2 = source[i + 2];
    column++;

    if (char === "\n") {
      line++;
      column = 0;
      if (state === "lineComment") {
        state = "code";
      }
      continue;
    }

    if (state === "lineComment") continue;

    if (state === "blockComment") {
      if (char === "*" && next === "/") {
        i++;
        column++;
        state = "code";
      }
      continue;
    }

    if (state === "string") {
      if (char === "\\") {
        i++;
        column++;
      } else if (char === "\"") {
        state = "code";
      }
      continue;
    }

    if (state === "char") {
      if (char === "\\") {
        i++;
        column++;
      } else if (char === "'") {
        state = "code";
      }
      continue;
    }

    if (state === "textBlock") {
      if (char === "\"" && next === "\"" && next2 === "\"") {
        i += 2;
        column += 2;
        state = "code";
      }
      continue;
    }

    if (char === "/" && next === "/") {
      i++;
      column++;
      state = "lineComment";
      continue;
    }

    if (char === "/" && next === "*") {
      i++;
      column++;
      state = "blockComment";
      continue;
    }

    if (char === "\"" && next === "\"" && next2 === "\"") {
      i += 2;
      column += 2;
      state = "textBlock";
      continue;
    }

    if (char === "\"") {
      state = "string";
      continue;
    }

    if (char === "'") {
      state = "char";
      continue;
    }

    if (char === "{" || char === "(" || char === "[") {
      stack.push({ char, line, column });
      continue;
    }

    const expectedOpening = closingToOpening.get(char);
    if (expectedOpening) {
      const opening = stack.pop();
      if (!opening) {
        return {
          level: "error",
          message: `Generated Java has an unmatched "${char}" at line ${line}, column ${column}.`,
        };
      }

      if (opening.char !== expectedOpening) {
        return {
          level: "error",
          message: `Generated Java has "${char}" at line ${line}, column ${column}, but expected "${openingToClosing.get(opening.char)}" for "${opening.char}" from line ${opening.line}, column ${opening.column}.`,
        };
      }
    }
  }

  if (state === "blockComment") {
    return { level: "error", message: "Generated Java has an unterminated block comment." };
  }

  if (state === "string" || state === "char" || state === "textBlock") {
    return { level: "error", message: "Generated Java has an unterminated literal." };
  }

  const unclosed = stack.pop();
  if (unclosed) {
    return {
      level: "error",
      message: `Generated Java is missing "${openingToClosing.get(unclosed.char)}" for "${unclosed.char}" from line ${unclosed.line}, column ${unclosed.column}.`,
    };
  }

  return null;
}

/**
 * Pedro 2 API that no longer exists in the vendored Pedro 3.0.0, with what
 * replaced it. TeamCode compiles against Pedro 3 only, so any of these in
 * generated code is a compile failure waiting to happen - or, worse, a
 * regression back to an exporter that was ported off them.
 */
const PEDRO2_API: Array<{ pattern: RegExp; replacement: string }> = [
  { pattern: /\bcom\.pedropathing\.geometry\b/, replacement: "com.pedropathing.geometry is gone; Pose is com.pedropathing.math.Pose" },
  { pattern: /\bcom\.pedropathing\.ftc\b/, replacement: "com.pedropathing.ftc is gone; the FTC module is com.pedropathing.revhub" },
  { pattern: /\bcom\.pedropathing\.util\b/, replacement: "com.pedropathing.util is gone; it is com.pedropathing.utils" },
  { pattern: /\bPathChain\b/, replacement: "PathChain is gone; a chain is one Path from Paths.path(...)" },
  { pattern: /\.pathBuilder\s*\(/, replacement: "follower.pathBuilder() is gone; build paths with com.pedropathing.api.Paths" },
  { pattern: /\bBezier(?:Line|Curve)\s*\(/, replacement: "new BezierLine/BezierCurve is gone; use Paths.line / Paths.curve" },
  { pattern: /\.set(?:Linear|Constant|Tangent)?HeadingInterpolation\s*\(/, replacement: "set*HeadingInterpolation is gone; use Path.constant / heading(Interpolator) / tangent" },
  { pattern: /\.setReversed\s*\(/, replacement: "setReversed is gone; use Path.reverseTangent()" },
  // The lookahead sits before the whitespace: after it, `\s*` could backtrack to
  // zero characters and let a newline pass as "not a digit".
  { pattern: /\.add(?:Parametric|Temporal|Pose)Callback\s*\((?!\s*[0-9])/, replacement: "PathBuilder callbacks are gone; the generated AutoPath takes a segment index first" },
  { pattern: /\.followPath\s*\(/, replacement: "followPath is gone; use follower.follow(path) with follower.holdEnd" },
  { pattern: /\.setStartingPose\s*\(/, replacement: "setStartingPose is gone; use follower.setPose" },
  { pattern: /\.(?:startTeleopDrive|setTeleOpDrive)\s*\(/, replacement: "teleop drive calls are gone; use follower.manual(DrivePowers)" },
  { pattern: /\.getPose\s*\(\s*\)/, replacement: "getPose() is gone; use follower.pose()" },
  { pattern: /\.get(?:X|Y|Heading)\s*\(\s*\)/, replacement: "Pose.getX/getY/getHeading are gone; use x() / y() / heading()" },
];

/** Source with comments and string/char literals blanked out, line breaks kept. */
function codeOnly(source: string): string {
  return source
    .replace(/\/\*[\s\S]*?\*\//g, (match) => match.replace(/[^\n]/g, " "))
    .replace(/\/\/[^\n]*/g, "")
    .replace(/"(?:\\.|[^"\\\n])*"/g, '""')
    .replace(/'(?:\\.|[^'\\\n])*'/g, "''");
}

/** Every Pedro 2 call left in generated code, as errors. Comments and strings are ignored. */
export function findPedro2Api(source: string): JavaValidationIssue[] {
  const code = codeOnly(source);
  return PEDRO2_API.filter(({ pattern }) => pattern.test(code)).map(({ replacement }) => ({
    level: "error" as const,
    message: `Generated Java uses Pedro 2 API: ${replacement}.`,
  }));
}

export function validateTeamCodeAutoSource(
  source: string,
  className: string,
): JavaValidationIssue[] {
  const issues: JavaValidationIssue[] = [];

  if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(className)) {
    issues.push({ level: "error", message: "Invalid Java class name." });
  }

  if (!source.includes(`public class ${className}`)) {
    issues.push({
      level: "error",
      message: "Generated code does not match the requested class name.",
    });
  }

  const delimiterIssue = validateJavaDelimiters(source);
  if (delimiterIssue) {
    issues.push(delimiterIssue);
  }

  issues.push(...findPedro2Api(source));

  return issues;
}
