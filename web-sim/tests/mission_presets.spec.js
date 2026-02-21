const { test, expect } = require("@playwright/test");

test.describe("Mission Presets & Controls Synchronization", () => {
  test.beforeEach(async ({ page }) => {
    // Navigate to the local server
    await page.goto("http://localhost:8097/");
    // Expand the mission guide block
    await page.locator("#mission-explanation-block .dropdown-toggle").click();
  });

  test("Preset selection updates task type, difficulty, target site, and rover strategy", async ({
    page,
  }) => {
    // Select the CY4 preset
    await page
      .locator("#mission-preset-select")
      .selectOption("cy4-sample-return");

    // Expected default for CY4:
    // task_type: movement, difficulty: L3, target_site: Candidate Site CY4-7, rover: rover-2
    await expect(page.locator("#task-type-select")).toHaveValue("movement");
    await expect(page.locator("#task-difficulty-select")).toHaveValue("L3");
    await expect(page.locator("#target-site-input")).toHaveValue(
      "Candidate Site CY4-7",
    );
    await expect(page.locator("#rover-target-select")).toHaveValue("rover-2");
  });

  test("Next step synchronizes configuration correctly", async ({ page }) => {
    // Select the CY3 preset
    await page.locator("#mission-preset-select").selectOption("cy3-pragyan");

    // Click Next Step
    await page.locator("#mission-next-step-btn").click();

    // The second step in CY3 is:
    // task_type: photo, difficulty: L1, target_site: Panorama Ridge, rover: rover-1 (default config via preset)
    await expect(page.locator("#task-type-select")).toHaveValue("photo");
    await expect(page.locator("#task-difficulty-select")).toHaveValue("L1");
    await expect(page.locator("#target-site-input")).toHaveValue(
      "Panorama Ridge",
    );
    await expect(page.locator("#rover-target-select")).toHaveValue("rover-1");
  });

  test("Manual override remains possible without breaking preset progression", async ({
    page,
  }) => {
    // Select LUPEX preset
    await page
      .locator("#mission-preset-select")
      .selectOption("lupex-polar-ice");

    // Verify first step defaults
    await expect(page.locator("#task-type-select")).toHaveValue("movement");

    // Manually override the task type and difficulty
    await page.locator("#task-type-select").selectOption("digging");
    await page.locator("#task-difficulty-select").selectOption("L5");
    await page.locator("#rover-target-select").selectOption("rover-1");

    await expect(page.locator("#task-type-select")).toHaveValue("digging");
    await expect(page.locator("#rover-target-select")).toHaveValue("rover-1");

    // Click next step
    await page.locator("#mission-next-step-btn").click();

    // Verify that progression continues - Step 2 (high-fidelity imaging survey) should load
    await expect(page.locator("#task-type-select")).toHaveValue("photo");
    await expect(page.locator("#task-difficulty-select")).toHaveValue("L2");
    await expect(page.locator("#rover-target-select")).toHaveValue("rover-3"); // Re-syncs to preset default
  });

  test("Apply preset button opens mission controls card", async ({ page }) => {
    // Expand a different card and ensure mission controls are closed
    await page.locator(".dropdown-toggle").first().click();

    // Target the mission apply button
    await page.locator("#mission-apply-btn").click();

    // Check if the mission controls block is now expanded
    const attr = await page
      .locator("#mission-controls-block .dropdown-toggle")
      .getAttribute("aria-expanded");
    expect(attr).toBe("true");
  });
});
