import { defineConfig } from "vitest/config";

export default defineConfig({
  base: "./",
  server: {
    host: "127.0.0.1",
    port: 5173
  },
  preview: {
    host: "127.0.0.1",
    port: 4173
  },
  build: {
    target: "es2020",
    sourcemap: true
  },
  test: {
    exclude: ["tests/e2e/**", "node_modules/**", "dist/**"]
  }
});
