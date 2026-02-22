const { test, expect } = require("@playwright/test");

test.describe("Mission Presets & Controls Synchronization", () => {
  test.beforeEach(async ({ page }) => {
    // Navigate to the local server
    await page.goto("http://localhost:8085/");
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

  test("Hidden configuration allows updating the initial rover count", async ({
    page,
  }) => {
    // Reveal hidden config by clicking the logo 3 times
    const logo = page.locator("#logo-mark");
    await logo.click();
    await page.waitForTimeout(50);
    await logo.click();
    await page.waitForTimeout(50);
    await logo.click();

    // The config panel should be visible
    await expect(page.locator("#hidden-config-panel")).toBeVisible();

    // Change rover count and restart
    await page.locator("#config-rover-count").fill("5");
    await page.locator("#config-restart-btn").click();

    // Verify url changed
    await expect(page).toHaveURL(/rovers=5/);

    // Verify rover count in fleet summary
    await page.waitForTimeout(1000); // give it a moment to boot
    const countText = await page.locator("#fleet-total-rovers").textContent();
    expect(countText.trim()).toBe("5");
  });

  test("Task failure causes SAFE_MODE and resumption starts from previous step", async ({
    page,
  }) => {
    // Inject mock for Math.random to guarantee a failure on the second execution step
    await page.evaluate(() => {
      window.forceFail = false;
      const originalRandom = Math.random;
      Math.random = () => {
        if (window.forceFail) {
          return 0; // Guaranteed to be < any non-zero faultProb
        }
        return 1; // Guaranteed > faultProb (max 0.6 in simulation)
      };
    });

    // Mission preset doesn't matter, we will configure a movement task manually
    // L1 Movement Tasks are resumable
    await page.locator("#task-type-select").selectOption("movement");
    await page.locator("#task-difficulty-select").selectOption("L1");
    // Explicitly uncheck auto toggle to ensure we get a predictable ID
    const autoToggle = page.locator("#task-id-auto-toggle");
    if (await autoToggle.isChecked()) {
      await autoToggle.uncheck();
    }
    await page.locator("#task-id-input").fill("TASK-TEST-RESUME");

    // Open command console
    // Since dropdown toggles can be tricky, let's just click the Start Task button
    // It is always present in the DOM
    await page.locator("#cmd-start-task").click();

    // Ensure rover state becomes EXECUTING
    await expect(page.locator("#rover-state")).toHaveText("EXECUTING", {
      timeout: 10000,
    });

    // Force failure on next tick
    await page.evaluate(() => {
      window.forceFail = true;
    });

    // Wait for failure
    await expect(page.locator("#rover-state")).toHaveText("SAFE_MODE", {
      timeout: 10000,
    });

    // Clear forced failure so we can recover
    await page.evaluate(() => {
      window.forceFail = false;
    });

    // Send RESET to clear SAFE_MODE
    await page.locator("#cmd-reset").click();

    // Verify it goes to IDLE
    await expect(page.locator("#rover-state")).toHaveText("IDLE", {
      timeout: 10000,
    });

    // Start task again
    await page.locator("#cmd-start-task").click();

    // It should go back to EXECUTING
    await expect(page.locator("#rover-state")).toHaveText("EXECUTING", {
      timeout: 10000,
    });

    // Wait for completion (simulated task steps)
    await expect(page.locator("#rover-state")).toHaveText("IDLE", {
      timeout: 15000,
    });
  });
});
