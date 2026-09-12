# Pedro Path MCP

Reviews a Pedro Pathing `.pp` route from a Claude chat: collisions, the AUTO
midline rule, the time estimate against the 30 second AUTO period, per-path
lengths and times, and the route warnings the visualizer raises.

The analysis is **not** a second implementation. The server bundles the
visualizer's own modules — the loader the editor reads a file with
(`src/utils/projectFile.ts`), the time calculator, and the clearance checker —
so a review in a chat says what the editor would say. That is also why the
bundle is built rather than committed: a stale copy would quietly disagree with
the editor, which is the one thing this must not do.

## Build

```bash
cd tools/pedro-visualizer
npm run build:mcp          # -> mcp/dist/pedro-path-mcp.mjs
```

or `./gradlew pathMcp` from the repo root, which builds it and prints the
config snippet with the right absolute path filled in. Rebuild after changing
anything under `src/utils/` — that is the point of it being a bundle.

Needs nothing installed: no MCP SDK, no runtime dependencies. esbuild comes
with Vite, and the server is plain `node` (18+).

## Register it

**Claude Code** (per project, from the repo root):

```bash
claude mcp add pedro-path -- node "<repo>/tools/pedro-visualizer/mcp/dist/pedro-path-mcp.mjs"
```

**Claude Desktop** — Settings → Developer → Edit Config, then add:

```jsonc
{
  "mcpServers": {
    "pedro-path": {
      "command": "node",
      "args": ["C:\\Users\\<you>\\StudioProjects\\RuckusBioBuzz\\tools\\pedro-visualizer\\mcp\\dist\\pedro-path-mcp.mjs"],
      "env": {
        // Optional: lets you say "review autos/left.pp" instead of a full path.
        "PEDRO_PATH_ROOT": "C:\\Users\\<you>\\StudioProjects\\RuckusBioBuzz\\tools\\pedro-visualizer\\public\\paths"
      }
    }
  }
}
```

Restart Claude Desktop afterwards.

**claude.ai in a browser cannot run this.** A local stdio server needs a client
on the same machine — Claude Desktop or Claude Code. In a browser chat, paste
the file's contents instead and it can be reviewed through the `content`
argument, or point the browser session at a remote connector, which this is not.

## Tools

### `review_path`

Pass **either** `file` (a path to a `.pp`, absolute or relative to
`PEDRO_PATH_ROOT`) **or** `content` (the file's JSON text).

| argument | meaning |
|---|---|
| `alliance` | `red` / `blue` / `off` / `file` (default). The AUTO midline rule: which half the robot must stay in. BIOBUZZ G402 makes TILE columns A–C the red side and D–F the blue. |
| `margin` | Safety margin in inches, overriding the file's. |
| `field` | Field map to measure against (e.g. `biobuzz.webp`). Only changes the obstacles when the file carries none of its own. |
| `step` | Footprint sample spacing along the route, inches (default 0.75). |
| `autoSeconds` | Length of the AUTO period (default 30). |

Returns a written review: time against the AUTO period, the field and robot it
was measured with, findings worst-first, clearance detail with the pose the
robot is in where it first touches, and a per-path table.

A file that carries no obstacles of its own is measured against its field map's
obstacle preset, and the review says so.

### `list_fields`

The field maps the visualizer knows and the obstacle preset each one carries,
with outlines in field inches. Useful for asking "what is actually in the way
this season".

## Asking for it

- "Review autos/red-left.pp for the red alliance."
- "Does this auto fit in 30 seconds, and does it hit anything?" (with the file pasted)
- "Re-check it with a 2 inch margin."
- "What obstacles does the BIOBUZZ preset have?"

## Coordinates

Field inches on the visualizer's 141.5 in square: `x = 0` is the red wall,
`y = 0` the audience wall, matching `FIELD_SIZE` and the field images.
