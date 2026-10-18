import { expect, test } from "@playwright/test";
import path from "node:path";

test("imports actual MATLAB results and renders Earth and spacecraft views", async ({ page }, testInfo) => {
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/");
  await page.locator("#replay-file").setInputFiles(process.env.ADCS_REPLAY_PATH ?? path.resolve("src/__tests__/fixtures/matlab-300s-subset.json"));
  await expect(page.locator("#source-mode-output")).toContainText("MATLAB replay");
  await expect(page.locator("#provenance-output")).toContainText("adcs_sim");
  await page.locator("#play-toggle").click();
  const end = Number(await page.locator("#time-scrubber").getAttribute("max"));
  await page.locator("#time-scrubber").fill(String(end));
  const endLabel = `T+${String(Math.floor(end/3600)).padStart(2,"0")}:${String(Math.floor(end/60)%60).padStart(2,"0")}:${String(end%60).padStart(2,"0")}`;
  await expect(page.locator("#mission-time")).toContainText(endLabel);
  await page.locator("#view-earth").click();
  await page.screenshot({ path: testInfo.outputPath("actual-replay-earth.png"), fullPage: true });
  await page.locator("#camera-select").selectOption("spacecraft");
  await expect(page.locator("#camera-readout")).toContainText("Spacecraft");
  await page.screenshot({ path: testInfo.outputPath("actual-replay-spacecraft.png"), fullPage: true });
  expect(errors).toEqual([]);
});

test("renders an interactive nonblank WebGL mission scene", async ({ page }) => {
  await page.goto("/");
  const canvas = page.getByTestId("mission-canvas");
  await expect(canvas).toBeVisible();
  await expect(page.getByTestId("control-panel")).toBeVisible();
  await expect(page.getByTestId("telemetry-panel")).toBeVisible();
  await expect(page.locator("#mission-time")).toContainText("T+");

  await page.waitForTimeout(700);
  const dataUrlLength = await canvas.evaluate((node) => (node as HTMLCanvasElement).toDataURL("image/png").length);
  expect(dataUrlLength).toBeGreaterThan(8000);

  const box = await canvas.boundingBox();
  expect(box).not.toBeNull();
  if (box) {
    await page.mouse.move(box.x + box.width * 0.52, box.y + box.height * 0.48);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width * 0.68, box.y + box.height * 0.54, { steps: 8 });
    await page.mouse.up();
  }

  await page.locator("input[name='startAltitudeKm']").fill("900");
  await page.locator("input[name='endAltitudeKm']").fill("1100");
  await page.locator("select[name='pointingMode']").selectOption("target-track");
  await page.locator("#apply-mission").click();
  await expect(page.locator("#dv-output")).toContainText("km/s");
  await expect(page.locator("#mode-output")).toContainText("Target");
  await expect(page.locator("#operation-label")).toContainText("Target");
  await expect(page.locator("#operation-progress-output")).toContainText("%");
});

test("ambient mode hides controls without removing the WebGL canvas", async ({ page }) => {
  await page.goto("/");
  await page.locator("#ambient-toggle").click();
  await expect(page.locator("body")).toHaveClass(/ambient/);
  await expect(page.getByTestId("mission-canvas")).toBeVisible();
});

test("detumbling operation exposes live rate and progress", async ({ page }) => {
  await page.goto("/");
  await page.locator("select[name='pointingMode']").selectOption("detumble");
  await page.locator("#apply-mission").click();

  await expect(page.locator("#mode-output")).toContainText("Detumble");
  await expect(page.locator("#operation-label")).toContainText("Detumble");
  await expect(page.locator("#operation-detail-output")).toContainText("deg/s");
  await expect(page.locator("#operation-phase-output")).toContainText("%");

  await expect
    .poll(async () => {
      const width = await page.locator("#operation-progress").evaluate((node) => (node as HTMLElement).style.width);
      return Number.parseFloat(width);
    })
    .toBeGreaterThan(0);
});

test("loads MATLAB replay, scrubs UTC timeline, switches cameras, and toggles layers", async ({ page }) => {
  await page.goto("/");

  await page.locator("#load-demo-replay").click();
  await expect(page.locator("#source-mode-output")).toContainText("MATLAB replay");
  await expect(page.locator("#schema-output")).toContainText("utat.adcs.replay.v1");
  await expect(page.locator("#provenance-output")).toContainText("adcs_sim demo export");
  await expect(page.locator("#quality-output")).toContainText("attitude");
  await expect(page.locator("#wheel-output")).toContainText("Nms");

  await page.locator("#time-scrubber").fill("300");
  await expect(page.locator("#mission-time")).toContainText("T+00:05:00");

  await page.locator("#camera-select").selectOption("spacecraft");
  await expect(page.locator("#camera-readout")).toContainText("Spacecraft");
  await page.locator("#view-earth").click();
  await expect(page.locator("#camera-readout")).toContainText("Earth");

  await page.locator("input[data-layer='footprint']").uncheck();
  await expect(page.locator("#layer-count-output")).toContainText("9 on");
  await page.locator("input[data-layer='footprint']").check();
  await expect(page.locator("#layer-count-output")).toContainText("10 on");

  const plotLength = await page.locator("#telemetry-plot").evaluate((node) => (node as HTMLCanvasElement).toDataURL("image/png").length);
  expect(plotLength).toBeGreaterThan(5000);
});
