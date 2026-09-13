/**
 * Pedro Path MCP - reviews a `.pp` route from a chat.
 *
 * An MCP server over stdio with no dependencies: MCP is JSON-RPC 2.0 in
 * line-delimited JSON, and hand-rolling the handful of methods a tool server
 * needs keeps this a single file that runs with plain `node`, with nothing to
 * install and nothing to keep in step with an SDK.
 *
 * The analysis is NOT reimplemented here. It is the visualizer's own modules -
 * the same loader the editor reads a file with, the same time calculator, the
 * same clearance checker - bundled in by esbuild, so a review in a chat says
 * what the editor would say.
 *
 * Build:  npm run build:mcp     (writes mcp/dist/pedro-path-mcp.mjs)
 * Run:    node mcp/dist/pedro-path-mcp.mjs
 */
import { readFile } from "node:fs/promises";
import { isAbsolute, resolve, basename } from "node:path";
import { homedir } from "node:os";
import { loadProjectData } from "../src/utils/projectFile";
import { formatPathReview, reviewProject } from "../src/utils/pathReview";
import { AVAILABLE_FIELD_MAPS, getDefaultShapes } from "../src/config/defaults";
import type { MidlineRule } from "../src/utils/clearance";

const SERVER_NAME = "pedro-path";
const SERVER_VERSION = "1.0.0";
/** Protocol revisions this server knows how to answer. */
const SUPPORTED_PROTOCOLS = ["2025-06-18", "2025-03-26", "2024-11-05"];

/* ------------------------------------------------------------------ tools -- */

const TOOLS = [
  {
    name: "review_path",
    description:
      "Review a Pedro Pathing .pp route: collision check against the field's " +
      "obstacles, walls and (optionally) the AUTO midline rule, plus the time " +
      "estimate against the 30 second AUTO period, per-path lengths and times, " +
      "and route warnings, including Pollen Pickup steps (what the camera can see " +
      "from where each starts, whether a single POLLEN there passes the pipeline's " +
      "area filter, and the worst case if every search runs to its timeout). " +
      "Measured with the robot's own footprint at the " +
      "heading it holds, using the visualizer's own modules. Pass either a " +
      "file path or the file's contents.",
    inputSchema: {
      type: "object",
      properties: {
        file: {
          type: "string",
          description:
            "Path to a .pp (or .json) route file. Absolute, or relative to the " +
            "server's working directory or PEDRO_PATH_ROOT.",
        },
        content: {
          type: "string",
          description: "The contents of a .pp file, as JSON text.",
        },
        alliance: {
          type: "string",
          enum: ["red", "blue", "off", "file"],
          description:
            "AUTO midline rule: which half the robot must stay in. BIOBUZZ G402 " +
            "makes TILE columns A-C the red side and D-F the blue. 'file' (the " +
            "default) uses the setting saved in the file.",
        },
        margin: {
          type: "number",
          description:
            "Safety margin in inches, overriding the file's. Anything closer than " +
            "this is reported as tight.",
        },
        field: {
          type: "string",
          description:
            "Field map to measure against, overriding the file's (for example " +
            "'biobuzz.webp'). Only changes the obstacles when the file carries " +
            "none of its own.",
        },
        step: {
          type: "number",
          description: "Footprint sample spacing along the route, inches (default 0.75).",
        },
        autoSeconds: {
          type: "number",
          description: "Length of the AUTO period in seconds (default 30).",
        },
      },
      additionalProperties: false,
    },
  },
  {
    name: "list_fields",
    description:
      "The field maps the visualizer knows and the obstacle preset each one " +
      "carries, with the obstacle outlines in field inches.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
  },
] as const;

/* --------------------------------------------------------------- helpers -- */

function expandPath(input: string): string {
  const trimmed = input.trim().replace(/^["']|["']$/g, "");
  const home = trimmed.startsWith("~")
    ? trimmed.replace(/^~(?=$|[/\\])/, homedir())
    : trimmed;
  if (isAbsolute(home)) return home;
  const root = process.env.PEDRO_PATH_ROOT;
  return root ? resolve(root, home) : resolve(process.cwd(), home);
}

function midlineFrom(alliance: unknown): MidlineRule | "off" | undefined {
  if (alliance === "red" || alliance === "blue") return { side: alliance };
  if (alliance === "off") return "off";
  return undefined; // "file", or not given: whatever the file says
}

async function runReview(args: Record<string, unknown>): Promise<string> {
  const file = typeof args.file === "string" ? args.file : "";
  const content = typeof args.content === "string" ? args.content : "";

  if (!file && !content) {
    throw new Error("Pass either `file` (a path to a .pp file) or `content` (its JSON).");
  }

  let text = content;
  let title = "pasted route";
  if (file) {
    const path = expandPath(file);
    try {
      text = await readFile(path, "utf8");
    } catch (error) {
      throw new Error(
        `Could not read ${path}: ${(error as Error).message}. Pass an absolute path, ` +
          "or set PEDRO_PATH_ROOT to the folder your .pp files live in.",
      );
    }
    title = basename(path);
  }

  let data: unknown;
  try {
    data = JSON.parse(text);
  } catch (error) {
    throw new Error(
      `That is not a readable .pp file — a .pp is JSON. ${(error as Error).message}`,
    );
  }
  if (!data || typeof data !== "object") {
    throw new Error("That file does not contain a route object.");
  }

  const project = loadProjectData(data);
  if (typeof args.field === "string" && args.field.trim()) {
    const field = args.field.trim();
    project.settings = { ...project.settings, fieldMap: field };
    // The field only changes what is in the way when the file brought nothing.
    if (project.usedFieldObstaclePreset) {
      project.shapes = getDefaultShapes(field);
    }
  }

  const review = reviewProject(project, {
    marginInches: typeof args.margin === "number" ? args.margin : undefined,
    midline: midlineFrom(args.alliance),
    stepInches: typeof args.step === "number" ? args.step : undefined,
    autoSeconds: typeof args.autoSeconds === "number" ? args.autoSeconds : undefined,
  });

  return formatPathReview(review, title);
}

function listFields(): string {
  const out: string[] = ["# Field maps"];
  for (const field of AVAILABLE_FIELD_MAPS) {
    if (field.value === "custom") continue;
    const shapes = getDefaultShapes(field.value);
    out.push("", `## ${field.label} (\`${field.value}\`)`);
    if (!shapes.length) {
      out.push("- No obstacle preset: a route on this field is measured against the walls only.");
      continue;
    }
    for (const shape of shapes) {
      const points = shape.vertices
        .map((vertex) => `(${vertex.x.toFixed(2)}, ${vertex.y.toFixed(2)})`)
        .join(" ");
      out.push(`- **${shape.name || shape.id}**: ${points}`);
    }
  }
  out.push(
    "",
    "Coordinates are field inches on the visualizer's 141.5in square, x = 0 at the " +
      "red wall and y = 0 at the audience wall.",
  );
  return out.join("\n");
}

async function callTool(name: string, args: Record<string, unknown>): Promise<string> {
  switch (name) {
    case "review_path":
      return runReview(args);
    case "list_fields":
      return listFields();
    default:
      throw new Error(`Unknown tool: ${name}`);
  }
}

/* ------------------------------------------------------------ the server -- */

type JsonRpcId = string | number | null;

function send(message: unknown): void {
  process.stdout.write(`${JSON.stringify(message)}\n`);
}

function respond(id: JsonRpcId, result: unknown): void {
  send({ jsonrpc: "2.0", id, result });
}

function respondError(id: JsonRpcId, code: number, message: string): void {
  send({ jsonrpc: "2.0", id, error: { code, message } });
}

async function handle(message: any): Promise<void> {
  const { id, method, params } = message ?? {};
  const isNotification = id === undefined;

  try {
    switch (method) {
      case "initialize": {
        const asked = params?.protocolVersion;
        respond(id, {
          protocolVersion: SUPPORTED_PROTOCOLS.includes(asked)
            ? asked
            : SUPPORTED_PROTOCOLS[0],
          capabilities: { tools: { listChanged: false } },
          serverInfo: { name: SERVER_NAME, version: SERVER_VERSION },
          instructions:
            "Reviews Pedro Pathing .pp routes: collisions, the AUTO midline rule, " +
            "and the time estimate against the 30s AUTO period.",
        });
        return;
      }

      case "notifications/initialized":
      case "notifications/cancelled":
        return;

      case "ping":
        respond(id, {});
        return;

      case "tools/list":
        respond(id, { tools: TOOLS });
        return;

      case "tools/call": {
        const name = params?.name;
        const args = (params?.arguments ?? {}) as Record<string, unknown>;
        try {
          const text = await callTool(name, args);
          respond(id, { content: [{ type: "text", text }] });
        } catch (error) {
          // A bad file or a bad argument is the caller's problem to fix, not a
          // protocol failure: hand it back as tool output so the model can read
          // it and try again.
          respond(id, {
            content: [{ type: "text", text: `Error: ${(error as Error).message}` }],
            isError: true,
          });
        }
        return;
      }

      // Nothing is served under these, but clients ask on connect.
      case "resources/list":
        respond(id, { resources: [] });
        return;
      case "resources/templates/list":
        respond(id, { resourceTemplates: [] });
        return;
      case "prompts/list":
        respond(id, { prompts: [] });
        return;

      default:
        if (isNotification) return;
        respondError(id, -32601, `Method not found: ${method}`);
    }
  } catch (error) {
    if (!isNotification) {
      respondError(id, -32603, (error as Error).message);
    }
  }
}

/**
 * Requests still being answered. Piped input reaches end-of-stream long before
 * an async handler finishes, so exiting on "end" would cut a review off midway.
 */
let inFlight = 0;
let inputEnded = false;

function maybeExit(): void {
  if (inputEnded && inFlight === 0) process.exit(0);
}

function main(): void {
  let buffer = "";
  process.stdin.setEncoding("utf8");
  process.stdin.on("data", (chunk: string) => {
    buffer += chunk;
    let newline = buffer.indexOf("\n");
    while (newline !== -1) {
      const line = buffer.slice(0, newline).trim();
      buffer = buffer.slice(newline + 1);
      if (line) {
        try {
          const message = JSON.parse(line);
          inFlight++;
          void handle(message).finally(() => {
            inFlight--;
            maybeExit();
          });
        } catch {
          respondError(null, -32700, "Parse error");
        }
      }
      newline = buffer.indexOf("\n");
    }
  });
  process.stdin.on("end", () => {
    inputEnded = true;
    maybeExit();
  });
}

main();
