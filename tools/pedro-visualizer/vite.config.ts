import { defineConfig } from "vite";
import type { Plugin, ViteDevServer } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";
import fs from "node:fs";
import type { IncomingMessage, ServerResponse } from "node:http";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { validateTeamCodeAutoSource } from "./src/utils/javaValidation";

const visualizerDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(visualizerDir, "..", "..");
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
            await fs.promises.writeFile(targetPath, content, "utf8");

            res.setHeader("Content-Type", "application/json");
            res.end(JSON.stringify({ path: targetPath, overwritten: existed }));
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
