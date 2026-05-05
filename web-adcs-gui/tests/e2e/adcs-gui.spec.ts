import { expect, test } from "@playwright/test";

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
