import { defineConfig } from "vite";
import type { Plugin, ViteDevServer } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";
import { execFile } from "node:child_process";
import fs from "node:fs";
import type { IncomingMessage, ServerResponse } from "node:http";
import path from "node:path";
import { promisify } from "node:util";
import { fileURLToPath } from "node:url";
import { validateTeamCodeAutoSource } from "./src/utils/javaValidation";

const visualizerDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(visualizerDir, "..", "..");
const execFileAsync = promisify(execFile);
const teamCodeAutoDir = path.resolve(
  repoRoot,
  "TeamCode",
  "src",
  "main",
  "java",
  "org",
  "firstinspires",
  "ftc",
  "teamcode",
  "auto",
);
const androidStudioJbr = "/Applications/Android Studio.app/Contents/jbr/Contents/Home";

function trimCompileOutput(output: string): string {
  const trimmed = output.trim();
  if (trimmed.length <= 12000) return trimmed;
  return `${trimmed.slice(0, 4000)}\n\n... output truncated ...\n\n${trimmed.slice(-7500)}`;
}

async function compileTeamCodeAuto(): Promise<{
  ok: boolean;
  output: string;
  exitCode?: number | string;
}> {
  const gradleWrapper = path.resolve(repoRoot, "gradlew");
  const env = fs.existsSync(androidStudioJbr)
    ? { ...process.env, JAVA_HOME: androidStudioJbr }
    : process.env;

  try {
    const result = await execFileAsync(
      gradleWrapper,
      [":TeamCode:compileDebugJavaWithJavac", "--rerun-tasks"],
      {
        cwd: repoRoot,
        env,
        timeout: 120000,
        maxBuffer: 8 * 1024 * 1024,
      },
    );

    return {
      ok: true,
      output: trimCompileOutput(`${result.stdout || ""}\n${result.stderr || ""}`),
    };
  } catch (error) {
    const execError = error as Error & {
      stdout?: string;
      stderr?: string;
      code?: number | string;
    };
    return {
      ok: false,
      exitCode: execError.code,
      output: trimCompileOutput(
        `${execError.stdout || ""}\n${execError.stderr || ""}` ||
          execError.message,
      ),
    };
  }
}

function teamCodeAutoWriter(): Plugin {
  return {
    name: "ruckus-teamcode-auto-writer",
    configureServer(server: ViteDevServer) {
      server.middlewares.use("/api/teamcode-autos", (req: IncomingMessage, res: ServerResponse, next: () => void) => {
        if (req.method !== "POST") {
          next();
          return;
        }

        let body = "";
        req.on("data", (chunk) => {
          body += chunk;
        });

        req.on("end", async () => {
          try {
            const payload = JSON.parse(body || "{}");
            const className = String(payload.className || "");
            const content = String(payload.content || "");
            const validationIssues = validateTeamCodeAutoSource(content, className);
            const validationError = validationIssues.find((issue) => issue.level === "error");
            if (validationError) {
              res.statusCode = 400;
              res.end(
                JSON.stringify({
                  error: validationError.message,
                  validationIssues,
                }),
              );
              return;
            }

            const targetPath = path.resolve(teamCodeAutoDir, `${className}.java`);
            if (!targetPath.startsWith(`${teamCodeAutoDir}${path.sep}`)) {
              res.statusCode = 400;
              res.end(JSON.stringify({ error: "Refusing to write outside TeamCode auto." }));
              return;
            }

            await fs.promises.mkdir(teamCodeAutoDir, { recursive: true });
            const existed = fs.existsSync(targetPath);
            const previousContent = existed
              ? await fs.promises.readFile(targetPath, "utf8")
              : null;
            await fs.promises.writeFile(targetPath, content, "utf8");
            const compileResult = await compileTeamCodeAuto();

            if (!compileResult.ok) {
              if (previousContent !== null) {
                await fs.promises.writeFile(targetPath, previousContent, "utf8");
              } else {
                await fs.promises.rm(targetPath, { force: true });
              }

              res.statusCode = 400;
              res.setHeader("Content-Type", "application/json");
              res.end(
                JSON.stringify({
                  error: `TeamCode compile failed for ${className}.java. The file was ${existed ? "restored to its previous version" : "not kept"}.`,
                  compileOutput: compileResult.output,
                  compileExitCode: compileResult.exitCode,
                  restored: true,
                  validationIssues,
                }),
              );
              return;
            }

            res.setHeader("Content-Type", "application/json");
            res.end(
              JSON.stringify({
                path: targetPath,
                overwritten: existed,
                compiled: true,
                compileOutput: compileResult.output,
              }),
            );
          } catch (error) {
            res.statusCode = 500;
            res.end(
              JSON.stringify({
                error: error instanceof Error ? error.message : String(error),
              }),
            );
          }
        });
      });
    },
  };
}

export default defineConfig({
  plugins: [svelte(), teamCodeAutoWriter()],
  build: {
    outDir: "dist",
    // Increase chunk size warning limit to 1.2 MB to avoid noisy warnings
    chunkSizeWarningLimit: 1200,
    rollupOptions: {
      output: {
        entryFileNames: `assets/[name].js`,
        chunkFileNames: `assets/[name].js`,
        assetFileNames: `assets/[name].[ext]`,
      },
    },
  },
  base: "./",
});
