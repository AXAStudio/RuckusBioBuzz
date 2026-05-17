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

  return issues;
}
