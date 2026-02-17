/* ═══════════════════════════════════════════════════════
   LSOAS — Application UI Controller
   Connects the Simulation Engine to the DOM.
   ═══════════════════════════════════════════════════════ */

(async function () {
  "use strict";

  // ─── DOM References ───
  const $ = (sel) => document.querySelector(sel);
  const $$ = (sel) => document.querySelectorAll(sel);

  async function loadTaskCatalog() {
    try {
      const response = await fetch("assets/chandrayaan_task_catalog.json", {
        cache: "no-cache",
      });
      if (!response.ok) return null;
      return await response.json();
    } catch (_err) {
      return null;
    }
  }

  // ─── Boot ───
  const taskCatalog = await loadTaskCatalog();
  const sim = new SimulationController({ taskCatalog });
  const bus = sim.bus;

  // ─── Orbital Visualization ───
  let viz = null;
  if (window.THREE) {
    try {
      viz = new VisualizationController(bus, "visualization-container");
    } catch (err) {
      // Keep mission control usable even when WebGL is unavailable (e.g. headless test runs).
      console.warn("Orbital visualization disabled:", err);
      viz = null;
    }
  }

  // Toggle Orbital Panel
  const floatingPanel = $("#floating-panel");
  const btnToggle = $("#btn-3d-toggle");
  const btnToggleLabel = $("#btn-3d-toggle-label");
  const btnClose = $("#btn-close-3d");
  const panelTelemetry = $("#panel-telemetry");
  let panelLayoutTimer = null;

  function set3DPanel(open) {
    if (panelLayoutTimer) {
      clearTimeout(panelLayoutTimer);
      panelLayoutTimer = null;
    }

    if (btnToggle) {
      btnToggle.setAttribute("aria-expanded", open ? "true" : "false");
    }
    if (btnToggleLabel) {
      btnToggleLabel.textContent = open ? "Close Orbital" : "Orbital View";
    }

    if (open) {
      floatingPanel.setAttribute("aria-hidden", "false");
      document.body.classList.add("three-panel-open");
      syncAccordionLayoutMode(true);
      if (panelTelemetry) panelTelemetry.classList.add("compact-log");
      updateFleetUi();

      requestAnimationFrame(() => {
        requestAnimationFrame(() => {
          floatingPanel.classList.add("panel-open");
        });
      });
    } else {
      floatingPanel.classList.remove("panel-open");
      floatingPanel.setAttribute("aria-hidden", "true");
      hideFleetHoverCard();
      panelLayoutTimer = setTimeout(() => {
        document.body.classList.remove("three-panel-open");
        syncAccordionLayoutMode(false);
        if (panelTelemetry) {
          panelTelemetry.classList.remove("compact-log");
        }
        updateFleetUi();
      }, 260);
    }

    // Trigger resize after transition allows renderer to catch up
    setTimeout(() => {
      if (viz) viz.onResize();
    }, 400); // slightly longer than CSS transition
  }

  function toggle3D() {
    const shouldOpen = !floatingPanel.classList.contains("panel-open");
    set3DPanel(shouldOpen);
  }

  if (btnToggle) btnToggle.addEventListener("click", toggle3D);
  if (btnClose)
    btnClose.addEventListener("click", () => {
      set3DPanel(false);
    });

  const dom = {
    metValue: $("#met-value"),
    sessionTime: $("#session-time"),
    roverState: $("#rover-state"),
    topoRoverLabel: $("#topo-rover .topo-node-label"),
    leftDropdownStack: $("#left-dropdown-stack"),
    rightDropdownStack: $("#right-dropdown-stack"),
    fleetGrid: $("#fleet-grid"),
    fleetGridSummary: $("#fleet-grid-summary"),
    fleetHoverCard: $("#fleet-hover-card"),
    toggleFleetScopeBtn: $("#btn-toggle-fleet-scope"),
    fleetTotalRovers: $("#fleet-total-rovers"),
    fleetStateDistribution: $("#fleet-state-distribution"),
    fleetAvgBattery: $("#fleet-avg-battery"),
    fleetAvgSolar: $("#fleet-avg-solar"),
    fleetAvgRisk: $("#fleet-avg-risk"),
    fleetCommandAck: $("#fleet-command-ack"),
    fleetMissionContext: $("#fleet-mission-context"),
    telemetryFeed: $("#telemetry-feed"),
    telemetryLastValue: $("#telemetry-last-value"),
    lunarMeta: $("#lunar-meta"),
    commandLog: $("#command-log"),
    pendingAcks: $("#pending-acks"),
    pendingCount: $("#pending-count"),
    packetsSent: $("#packets-sent"),
    packetsReceived: $("#packets-received"),
    packetsDropped: $("#packets-dropped"),
    missionPresetSelect: $("#mission-preset-select"),
    missionControlsBlock: $("#mission-controls-block"),
    missionExplanationBlock: $("#mission-explanation-block"),
    missionBriefCode: $("#mission-brief-code"),
    missionBriefPhase: $("#mission-brief-phase"),
    missionBriefSummary: $("#mission-brief-summary"),
    missionBriefTags: $("#mission-brief-tags"),
    missionStepList: $("#mission-step-list"),
    missionApplyBtn: $("#mission-apply-btn"),
    missionNextStepBtn: $("#mission-next-step-btn"),
    missionResetBtn: $("#mission-reset-btn"),
    taskIdInput: $("#task-id-input"),
    taskIdAutoToggle: $("#task-id-auto-toggle"),
    taskIdRegenerateBtn: $("#task-id-regenerate-btn"),
    taskIdPreview: $("#task-id-preview"),
    taskTypeSelect: $("#task-type-select"),
    taskDifficultySelect: $("#task-difficulty-select"),
    targetSiteInput: $("#target-site-input"),
    roverTargetSelect: $("#rover-target-select"),
    latencySlider: $("#latency-slider"),
    latencyValue: $("#latency-value"),
    jitterSlider: $("#jitter-slider"),
    jitterValue: $("#jitter-value"),
    dropSlider: $("#drop-slider"),
    dropValue: $("#drop-value"),
    faultSlider: $("#fault-slider"),
    faultProbValue: $("#fault-prob-value"),
    topologyVisual: $("#topology-visual"),
  };

  function getAccordionBlocks(groupName) {
    return Array.from(
      document.querySelectorAll(
        `.dropdown-block[data-accordion-group="${groupName}"]`,
      ),
    );
  }

  function setAccordionBlockOpen(block, open) {
    if (!block) return;
    const content = block.querySelector(".dropdown-content");
    const toggle = block.querySelector(".dropdown-toggle");
    block.classList.toggle("is-open", open);
    if (content) content.hidden = !open;
    if (toggle) toggle.setAttribute("aria-expanded", open ? "true" : "false");
    if (open && block.id === "left-topology-block") {
      requestAnimationFrame(() => {
        requestAnimationFrame(() => drawTopologyLines());
      });
    }
  }

  function toggleAccordionBlock(block) {
    if (!block) return;
    const group = String(block.dataset.accordionGroup || "");
    const singleOpen = block.dataset.singleOpen === "true";
    const nextOpen = !block.classList.contains("is-open");

    if (singleOpen && nextOpen && group) {
      getAccordionBlocks(group).forEach((candidate) => {
        if (candidate !== block) setAccordionBlockOpen(candidate, false);
      });
    }

    setAccordionBlockOpen(block, nextOpen);
  }

  function configureAccordionGroup(groupName, { singleOpen = false } = {}) {
    getAccordionBlocks(groupName).forEach((block) => {
      block.dataset.singleOpen = singleOpen ? "true" : "false";
    });
  }

  function syncAccordionLayoutMode(compact) {
    const leftBlocks = getAccordionBlocks("left-orbital");
    const telemetryBlocks = getAccordionBlocks("telemetry-orbital");

    if (leftBlocks.length > 0) {
      configureAccordionGroup("left-orbital", { singleOpen: compact });
      if (compact) {
        leftBlocks.forEach((block) => setAccordionBlockOpen(block, false));
      } else {
        leftBlocks.forEach((block) => {
          const defaultOpen = block.dataset.defaultOpen !== "false";
          setAccordionBlockOpen(block, defaultOpen);
        });
      }
    }

    if (telemetryBlocks.length > 0) {
      configureAccordionGroup("telemetry-orbital", { singleOpen: compact });
      if (compact) {
        telemetryBlocks.forEach((block) => setAccordionBlockOpen(block, false));
        const preferred = telemetryBlocks.find(
          (block) => block.id === "telemetry-stream-block",
        );
        setAccordionBlockOpen(preferred || telemetryBlocks[0], true);
      } else {
        telemetryBlocks.forEach((block) => {
          const defaultOpen = block.dataset.defaultOpen !== "false";
          setAccordionBlockOpen(block, defaultOpen);
        });
      }
    }
  }

  function initializeAccordions() {
    const blocks = Array.from(document.querySelectorAll(".dropdown-block"));
    blocks.forEach((block) => {
      const toggle = block.querySelector(".dropdown-toggle");
      if (!toggle) return;
      toggle.addEventListener("click", () => toggleAccordionBlock(block));
      const isOpen = block.classList.contains("is-open");
      setAccordionBlockOpen(block, isOpen);
    });
    configureAccordionGroup("right-controls", { singleOpen: false });
    configureAccordionGroup("telemetry-orbital", { singleOpen: false });
    syncAccordionLayoutMode(document.body.classList.contains("three-panel-open"));
  }

  function openMissionControlsCard() {
    if (!dom.missionControlsBlock) return;
    setAccordionBlockOpen(dom.missionControlsBlock, true);
  }

  function hydrateTaskSelectorsFromCatalog() {
    if (!taskCatalog || !dom.taskTypeSelect || !dom.taskDifficultySelect) return;

    const taskTypes = Object.keys(taskCatalog.task_types || {});
    if (taskTypes.length > 0) {
      dom.taskTypeSelect.innerHTML = taskTypes
        .map((taskType) => {
          const display = taskCatalog.task_types[taskType]?.display_name || taskType;
          return `<option value="${taskType}">${display}</option>`;
        })
        .join("");
      if (taskTypes.includes("science")) {
        dom.taskTypeSelect.value = "science";
      }
    }

    const difficultyLevels = Object.keys(taskCatalog.difficulty_levels || {});
    if (difficultyLevels.length > 0) {
      dom.taskDifficultySelect.innerHTML = difficultyLevels
        .sort()
        .map((difficulty) => `<option value="${difficulty}">${difficulty}</option>`)
        .join("");
      if (difficultyLevels.includes("L2")) {
        dom.taskDifficultySelect.value = "L2";
      }
    }
  }
  hydrateTaskSelectorsFromCatalog();

  const TASK_TYPE_CODES = {
    movement: "MOV",
    science: "SCI",
    digging: "DIG",
    pushing: "PSH",
    photo: "IMG",
    "sample-handling": "SMP",
  };

  const MISSION_PRESETS = {
    "cy3-pragyan": {
      id_code: "CY3",
      mission_phase: "CY3-ops",
      title: "Chandrayaan-3 Pragyan Surface Ops",
      summary:
        "Traverse, image, and run in-situ surface science analogs around a Pragyan-style landing zone.",
      payload_tags: ["Pragyan", "LIBS", "APXS", "NavCam"],
      default_target_site: "Shiv Shakti Point Sector-A",
      steps: [
        {
          title: "Traverse to scan waypoint",
          task_type: "movement",
          difficulty_level: "L2",
          mission_phase: "CY3-ops",
          target_site: "Traverse Corridor-1",
          note: "Position rover safely before instrument operations.",
        },
        {
          title: "Panoramic imaging sweep",
          task_type: "photo",
          difficulty_level: "L1",
          mission_phase: "CY3-ops",
          target_site: "Panorama Ridge",
          note: "Capture terrain mosaic for route and science planning.",
        },
        {
          title: "In-situ elemental science pass",
          task_type: "science",
          difficulty_level: "L3",
          mission_phase: "CY3-ops",
          target_site: "Surface Patch LIBS-02",
          note: "Teach payload-style analysis sequencing.",
        },
      ],
    },
    "cy4-sample-return": {
      id_code: "CY4",
      mission_phase: "CY4-sample-chain",
      title: "Chandrayaan-4 Sample Return Chain",
      summary:
        "A teaching sequence for prospecting, extraction, sample custody, and transfer preparation.",
      payload_tags: ["Sampling Drill", "Transfer Canister", "Surface Relay"],
      default_target_site: "Sample Depot Alpha",
      steps: [
        {
          title: "Prospecting traverse to candidate site",
          task_type: "movement",
          difficulty_level: "L3",
          mission_phase: "CY4-sample-chain",
          target_site: "Candidate Site CY4-7",
          note: "Reach site with enough battery and comm margin.",
        },
        {
          title: "Volatile prospecting scan",
          task_type: "science",
          difficulty_level: "L3",
          mission_phase: "CY4-sample-chain",
          target_site: "Candidate Site CY4-7",
          note: "Run spectroscopy-like measurement cycle.",
        },
        {
          title: "Core extraction",
          task_type: "digging",
          difficulty_level: "L4",
          mission_phase: "CY4-sample-chain",
          target_site: "Drill Spot CY4-7B",
          note: "Higher thermal and terrain stress during drilling.",
        },
        {
          title: "Sample transfer and custody check",
          task_type: "sample-handling",
          difficulty_level: "L4",
          mission_phase: "CY4-sample-chain",
          target_site: "Sample Depot Alpha",
          note: "Validate chain-of-custody workflow for return prep.",
        },
      ],
    },
    "lupex-polar-ice": {
      id_code: "LXP",
      mission_phase: "LUPEX-prospecting",
      title: "LUPEX Polar Ice Prospecting",
      summary:
        "Polar traverse and subsurface investigation sequence focused on volatile resource mapping.",
      payload_tags: ["Polar Traverse", "Volatile Mapping", "Drill Ops"],
      default_target_site: "Polar Shadow Boundary",
      steps: [
        {
          title: "Low-light approach traverse",
          task_type: "movement",
          difficulty_level: "L4",
          mission_phase: "LUPEX-prospecting",
          target_site: "Shadow Boundary Route-3",
          note: "Night/terminator margins strongly influence risk.",
        },
        {
          title: "High-fidelity imaging survey",
          task_type: "photo",
          difficulty_level: "L2",
          mission_phase: "LUPEX-prospecting",
          target_site: "Polar Survey Grid",
          note: "Map hazard zones before drilling.",
        },
        {
          title: "Subsurface excavation run",
          task_type: "digging",
          difficulty_level: "L5",
          mission_phase: "LUPEX-prospecting",
          target_site: "Ice Prospect Borehole",
          note: "Max difficulty case for environment-aware planning.",
        },
      ],
    },
    "cy-future-base": {
      id_code: "CYB",
      mission_phase: "base-build",
      title: "Future Chandrayaan Base Build",
      summary:
        "Multipurpose swarm scenario for regolith logistics, infrastructure staging, and base readiness.",
      payload_tags: ["Swarm Ops", "Regolith Handling", "Infrastructure Build"],
      default_target_site: "Bharati Base Site-01",
      steps: [
        {
          title: "Route clearance traverse",
          task_type: "movement",
          difficulty_level: "L2",
          mission_phase: "base-build",
          target_site: "Access Corridor-B",
          note: "Prepare route before heavy logistics tasks.",
        },
        {
          title: "Regolith push and berm shaping",
          task_type: "pushing",
          difficulty_level: "L4",
          mission_phase: "base-build",
          target_site: "Shield Berm Segment-2",
          note: "Material handling under traction limits.",
        },
        {
          title: "Excavation for foundation trench",
          task_type: "digging",
          difficulty_level: "L4",
          mission_phase: "base-build",
          target_site: "Foundation Trench-A",
          note: "Excavation workload for habitat anchoring.",
        },
        {
          title: "Sample transfer to processing bay",
          task_type: "sample-handling",
          difficulty_level: "L3",
          mission_phase: "base-build",
          target_site: "ISRU Processing Bay",
          note: "Logistics hand-off between rover roles.",
        },
        {
          title: "Post-build imaging verification",
          task_type: "photo",
          difficulty_level: "L2",
          mission_phase: "base-build",
          target_site: "Base Site Panorama Mast",
          note: "Visual QA pass for teaching mission closure.",
        },
      ],
    },
  };

  const DEFAULT_MISSION_PRESET = "cy3-pragyan";
  const missionPresetKeys = Object.keys(MISSION_PRESETS);
  const taskIdCounters = {};
  const missionGuideState = {
    presetKey: DEFAULT_MISSION_PRESET,
    completedCount: 0,
    activeStepIndex: 0,
    currentMissionPhase: MISSION_PRESETS[DEFAULT_MISSION_PRESET].mission_phase,
  };
  let taskIdDirty = false;
  let suppressTaskIdInputTracking = false;

  function safeMissionPresetKey(rawKey) {
    const key = String(rawKey || "").trim();
    return missionPresetKeys.includes(key) ? key : DEFAULT_MISSION_PRESET;
  }

  function getActiveMissionPreset() {
    return MISSION_PRESETS[safeMissionPresetKey(missionGuideState.presetKey)];
  }

  function getTaskTypeConfig(taskType) {
    return taskCatalog?.task_types?.[taskType] || null;
  }

  function sanitizeTaskToken(value, fallback = "TASK") {
    const token = String(value || "")
      .trim()
      .toUpperCase()
      .replace(/[^A-Z0-9]+/g, "-")
      .replace(/(^-+|-+$)/g, "");
    return token || fallback;
  }

  function getTaskIdContext() {
    const preset = getActiveMissionPreset();
    const taskType = String(dom.taskTypeSelect?.value || "movement")
      .trim()
      .toLowerCase();
    const difficulty = String(dom.taskDifficultySelect?.value || "L2")
      .trim()
      .toUpperCase();
    const missionCode = sanitizeTaskToken(preset.id_code || "CYX", "CYX");
    const taskCode = TASK_TYPE_CODES[taskType] || sanitizeTaskToken(taskType, "TASK").slice(0, 4);
    const difficultyCode = /^L[1-5]$/.test(difficulty) ? difficulty : "L2";

    return {
      missionCode,
      taskCode,
      difficultyCode,
    };
  }

  function formatTaskId(context, sequence) {
    return `${context.missionCode}-${context.taskCode}-${context.difficultyCode}-${String(sequence).padStart(3, "0")}`;
  }

  function peekGeneratedTaskId() {
    const context = getTaskIdContext();
    const counterKey = `${context.missionCode}:${context.taskCode}:${context.difficultyCode}`;
    const sequence = taskIdCounters[counterKey] || 1;
    return formatTaskId(context, sequence);
  }

  function reserveGeneratedTaskId() {
    const context = getTaskIdContext();
    const counterKey = `${context.missionCode}:${context.taskCode}:${context.difficultyCode}`;
    const sequence = taskIdCounters[counterKey] || 1;
    taskIdCounters[counterKey] = sequence + 1;
    return formatTaskId(context, sequence);
  }

  function updateTaskIdPreview() {
    if (!dom.taskIdPreview) return;
    const suggested = peekGeneratedTaskId();
    if (!dom.taskIdAutoToggle?.checked) {
      dom.taskIdPreview.textContent = `Manual mode · Suggested ${suggested}`;
      return;
    }
    dom.taskIdPreview.textContent = taskIdDirty
      ? `Manual override · Suggested ${suggested}`
      : `Next ${suggested}`;
  }

  function syncTaskIdInput({ force = false } = {}) {
    if (!dom.taskIdInput) return null;
    const generated = peekGeneratedTaskId();
    const current = String(dom.taskIdInput.value || "").trim();
    const canApplyAuto = dom.taskIdAutoToggle?.checked && !taskIdDirty;
    if (force || !current || canApplyAuto) {
      suppressTaskIdInputTracking = true;
      dom.taskIdInput.value = generated;
      suppressTaskIdInputTracking = false;
      taskIdDirty = false;
    }
    updateTaskIdPreview();
    return generated;
  }

  function markTaskIdAsDispatched() {
    reserveGeneratedTaskId();
    if (dom.taskIdAutoToggle?.checked) {
      taskIdDirty = false;
      syncTaskIdInput({ force: true });
    } else {
      updateTaskIdPreview();
    }
  }

  function getMissionStepList() {
    return getActiveMissionPreset().steps || [];
  }

  function getActiveMissionStep() {
    const steps = getMissionStepList();
    if (steps.length === 0) return null;
    const clampedIndex = Math.max(0, Math.min(missionGuideState.activeStepIndex, steps.length - 1));
    return steps[clampedIndex];
  }

  function setMissionGuidePhase(nextPhase) {
    missionGuideState.currentMissionPhase = String(nextPhase || "").trim() || getActiveMissionPreset().mission_phase;
    if (dom.missionBriefPhase) {
      dom.missionBriefPhase.textContent = missionGuideState.currentMissionPhase;
    }
  }

  function renderMissionBrief() {
    const preset = getActiveMissionPreset();
    if (dom.missionPresetSelect) {
      dom.missionPresetSelect.value = safeMissionPresetKey(preset && missionGuideState.presetKey);
    }
    if (dom.missionBriefCode) dom.missionBriefCode.textContent = preset.id_code;
    if (dom.missionBriefSummary) dom.missionBriefSummary.textContent = preset.summary;
    setMissionGuidePhase(missionGuideState.currentMissionPhase || preset.mission_phase);
    if (dom.missionBriefTags) {
      dom.missionBriefTags.innerHTML = (preset.payload_tags || [])
        .map((tag) => `<span class="mission-brief-tag">${escapeHtml(tag)}</span>`)
        .join("");
    }
  }

  function renderMissionStepList() {
    if (!dom.missionStepList) return;
    const steps = getMissionStepList();
    if (steps.length === 0) {
      dom.missionStepList.innerHTML = "";
      return;
    }

    const nextIndex = Math.min(missionGuideState.completedCount, steps.length - 1);
    dom.missionStepList.innerHTML = steps
      .map((step, index) => {
        const isComplete = index < missionGuideState.completedCount;
        const isNext = missionGuideState.completedCount < steps.length && index === nextIndex;
        const isApplied = index === missionGuideState.activeStepIndex;
        const classes = [
          "mission-step-item",
          isComplete ? "is-complete" : "",
          isNext ? "is-next" : "",
          isApplied ? "is-applied" : "",
        ]
          .filter(Boolean)
          .join(" ");

        return `
          <li class="${classes}" data-step-index="${index}">
            <div class="mission-step-title">
              <span>${index + 1}. ${escapeHtml(step.title || "Step")}</span>
              <span class="mission-step-badge">${isComplete ? "DONE" : isNext ? "NEXT" : "PLAN"}</span>
            </div>
            <div class="mission-step-meta">
              <span class="mission-step-chip">${escapeHtml(step.task_type || "movement")}</span>
              <span class="mission-step-chip">${escapeHtml(step.difficulty_level || "L2")}</span>
            </div>
            <div class="mission-step-note">${escapeHtml(step.note || "")}</div>
          </li>
        `;
      })
      .join("");
  }

  function applyMissionStepToControls(
    step,
    { announce = false, forceTaskId = false } = {},
  ) {
    if (!step) return;

    if (dom.taskTypeSelect && step.task_type && dom.taskTypeSelect.querySelector(`option[value="${step.task_type}"]`)) {
      dom.taskTypeSelect.value = step.task_type;
    }
    if (
      dom.taskDifficultySelect &&
      step.difficulty_level &&
      dom.taskDifficultySelect.querySelector(`option[value="${step.difficulty_level}"]`)
    ) {
      dom.taskDifficultySelect.value = step.difficulty_level;
    }
    if (dom.targetSiteInput) {
      dom.targetSiteInput.value = step.target_site || getActiveMissionPreset().default_target_site || "";
    }
    setMissionGuidePhase(step.mission_phase || getActiveMissionPreset().mission_phase);
    if (forceTaskId) {
      taskIdDirty = false;
      syncTaskIdInput({ force: true });
    } else {
      syncTaskIdInput();
    }
    renderMissionStepList();

    if (announce) {
      addFeedLine(
        "system",
        `📘 Mission step ready: ${step.title} (${step.task_type}/${step.difficulty_level})`,
      );
    }
  }

  function setMissionPreset(presetKey, { announce = false, resetProgress = true } = {}) {
    missionGuideState.presetKey = safeMissionPresetKey(presetKey);
    const preset = getActiveMissionPreset();
    if (resetProgress) {
      missionGuideState.completedCount = 0;
      missionGuideState.activeStepIndex = 0;
    } else {
      missionGuideState.activeStepIndex = Math.max(
        0,
        Math.min(
          missionGuideState.activeStepIndex,
          Math.max(0, (preset.steps || []).length - 1),
        ),
      );
    }
    setMissionGuidePhase(preset.mission_phase);
    renderMissionBrief();
    renderMissionStepList();
    applyMissionStepToControls(getActiveMissionStep(), {
      announce: false,
      forceTaskId: true,
    });

    if (announce) {
      addFeedLine("system", `🛰️ Mission preset loaded: ${preset.title}`);
    }
  }

  function setMissionActiveStep(index, { announce = false } = {}) {
    const steps = getMissionStepList();
    if (steps.length === 0) return;
    missionGuideState.activeStepIndex = Math.max(0, Math.min(index, steps.length - 1));
    applyMissionStepToControls(getActiveMissionStep(), {
      announce,
      forceTaskId: true,
    });
  }

  function moveMissionActiveStep(delta) {
    const steps = getMissionStepList();
    if (steps.length === 0) return;
    setMissionActiveStep(missionGuideState.activeStepIndex + delta, { announce: true });
  }

  function advanceMissionGuideAfterDispatch() {
    const steps = getMissionStepList();
    if (steps.length === 0) return;

    if (missionGuideState.activeStepIndex >= missionGuideState.completedCount) {
      missionGuideState.completedCount = Math.min(
        steps.length,
        missionGuideState.activeStepIndex + 1,
      );
    }

    if (missionGuideState.completedCount >= steps.length) {
      missionGuideState.activeStepIndex = steps.length - 1;
      renderMissionStepList();
      addFeedLine("system", "✅ Mission guide sequence complete. Use Reset Guide to run again.");
      return;
    }

    missionGuideState.activeStepIndex = missionGuideState.completedCount;
    applyMissionStepToControls(getActiveMissionStep(), {
      announce: false,
      forceTaskId: true,
    });
  }

  function ensureDispatchTaskId() {
    const existing = String(dom.taskIdInput?.value || "").trim();
    if (existing) return existing;
    taskIdDirty = false;
    syncTaskIdInput({ force: true });
    return String(dom.taskIdInput?.value || "").trim() || peekGeneratedTaskId();
  }

  const AUTO_ROVER_MODE = "auto";
  const roverIdCollator = new Intl.Collator(undefined, {
    numeric: true,
    sensitivity: "base",
  });
  const fleetSnapshots = {};
  const trafficCounters = {
    commandsSent: 0,
    acksReceived: 0,
  };
  const pinStorageKey = "lsoas:pinned-rovers";
  const pinnedRoverIds = new Set();
  let roverTargetMode = AUTO_ROVER_MODE;
  let showAllRovers = false;

  function setVisualizationMode() {
    if (!viz || typeof viz.setViewMode !== "function") return;
    viz.setViewMode("orbital");
  }
  setVisualizationMode();

  function clamp01(value) {
    return Math.max(0, Math.min(1, value));
  }

  function normalizeState(rawState) {
    return String(rawState || "UNKNOWN").toUpperCase();
  }

  function stateToClass(state) {
    return `state-${normalizeState(state).toLowerCase().replace("_", "-")}`;
  }

  function formatRoverLabel(roverId) {
    if (!roverId) return "Unknown";
    return roverId.replace(/^rover-/i, "Rover-");
  }

  function getSelectedTaskConfig(taskIdOverride = null) {
    const taskIdRaw = taskIdOverride || dom.taskIdInput?.value || "TASK-001";
    const taskType = String(dom.taskTypeSelect?.value || "movement").trim().toLowerCase();
    const difficultyLevel = String(dom.taskDifficultySelect?.value || "L2")
      .trim()
      .toUpperCase();
    const taskTypeCfg = getTaskTypeConfig(taskType);
    const requiredCapabilities = Array.isArray(taskTypeCfg?.required_capabilities)
      ? [...taskTypeCfg.required_capabilities]
      : [];
    const targetSiteRaw = String(dom.targetSiteInput?.value || "").trim();

    return {
      task_id: taskIdRaw.trim() || "TASK-001",
      task_type: taskType || "movement",
      difficulty_level: difficultyLevel || "L2",
      required_capabilities: requiredCapabilities,
      mission_phase:
        missionGuideState.currentMissionPhase ||
        taskTypeCfg?.mission_phase ||
        getActiveMissionPreset().mission_phase,
      target_site: targetSiteRaw || null,
    };
  }

  function isCompactFleetMode() {
    return panelTelemetry?.classList.contains("compact-log");
  }

  function loadPinnedRovers() {
    try {
      const raw = window.localStorage.getItem(pinStorageKey);
      if (!raw) return;
      const parsed = JSON.parse(raw);
      if (!Array.isArray(parsed)) return;
      parsed.forEach((roverId) => {
        if (typeof roverId === "string" && roverId.length > 0) {
          pinnedRoverIds.add(roverId);
        }
      });
    } catch (_err) {
      // Ignore storage parse errors and continue with empty pins.
    }
  }

  function persistPinnedRovers() {
    try {
      window.localStorage.setItem(
        pinStorageKey,
        JSON.stringify([...pinnedRoverIds]),
      );
    } catch (_err) {
      // Ignore persistence failures (private mode / storage unavailable).
    }
  }

  function inferSolarExposure(snapshot) {
    if (Number.isFinite(snapshot?.solar_exposure)) {
      return clamp01(snapshot.solar_exposure);
    }

    if (Number.isFinite(snapshot?.position?.lon)) {
      const angle = (snapshot.position.lon * Math.PI) / 180;
      return clamp01((Math.cos(angle) + 1) / 2);
    }

    return 0.5;
  }

  function upsertFleetSnapshot(roverId, incoming = {}) {
    if (!roverId) return;

    const previous = fleetSnapshots[roverId] || {
      rover_id: roverId,
      state: "IDLE",
      battery: 1,
      task_id: null,
      task_progress: null,
      task_total_steps: null,
      position: null,
      fault: null,
      solar_exposure: 0.5,
    };

    const merged = {
      ...previous,
      ...incoming,
      rover_id: roverId,
    };

    const batteryValue = Number(merged.battery);
    merged.battery = Number.isFinite(batteryValue) ? clamp01(batteryValue) : 0;
    merged.solar_exposure = inferSolarExposure(merged);

    fleetSnapshots[roverId] = merged;
  }

  function getFleetEntries() {
    return Object.values(fleetSnapshots).sort((a, b) =>
      roverIdCollator.compare(a.rover_id || "", b.rover_id || ""),
    );
  }

  function getAttentionScore(snapshot) {
    const state = normalizeState(snapshot.state);
    const battery = Number(snapshot.battery || 0);
    const hasFault = Boolean(snapshot.fault);
    const staleSeconds = snapshot.last_seen
      ? Math.max(0, Date.now() / 1000 - snapshot.last_seen)
      : 0;

    let score = 0;
    if (state === "ERROR") score += 120;
    else if (state === "SAFE_MODE") score += 100;
    else if (state === "EXECUTING") score += 35;

    if (hasFault) score += 45;
    if (battery < 0.2) score += 40;
    else if (battery < 0.35) score += 24;
    else if (battery < 0.5) score += 12;

    if (staleSeconds > 8) score += 18;

    return score + (1 - clamp01(battery)) * 5;
  }

  function getPriorityFleetEntries(entries) {
    const ranked = [...entries].sort((a, b) => {
      const diff = getAttentionScore(b) - getAttentionScore(a);
      if (diff !== 0) return diff;
      return roverIdCollator.compare(a.rover_id || "", b.rover_id || "");
    });

    const topAttention = ranked.slice(0, 3);
    const pinned = ranked.filter((snapshot) =>
      pinnedRoverIds.has(snapshot.rover_id),
    );

    const merged = [];
    [...pinned, ...topAttention].forEach((snapshot) => {
      if (!merged.some((item) => item.rover_id === snapshot.rover_id)) {
        merged.push(snapshot);
      }
    });

    return merged;
  }

  function buildFleetCardMarkup(snapshot, compactMode) {
    const state = normalizeState(snapshot.state);
    const stateClass = stateToClass(state);
    const batteryPct = Math.round((snapshot.battery || 0) * 100);
    const solarExposure = snapshot.solar_exposure || 0;
    const solarText = solarExposure >= 0.5 ? "☀ Sun" : "🌑 Dark";
    const solarPct = Math.round(solarExposure * 100);
    const taskText = snapshot.task_id
      ? `${snapshot.task_id}${snapshot.task_progress !== null && snapshot.task_progress !== undefined ? ` (${snapshot.task_progress}/${snapshot.task_total_steps || "?"})` : ""}`
      : "—";
    const pinClass = pinnedRoverIds.has(snapshot.rover_id) ? "pinned" : "";

    return `
      <article class="fleet-card ${stateClass} ${pinClass} ${compactMode ? "compact" : ""}" data-rover-id="${snapshot.rover_id}" tabindex="0">
        <div class="fleet-card-header">
          <span class="fleet-card-rover-id">${(snapshot.rover_id || "rover-?").toUpperCase()}</span>
          <div class="fleet-card-actions">
            <button type="button" class="fleet-pin-btn ${pinClass}" data-rover-id="${snapshot.rover_id}" title="${pinClass ? "Unpin rover" : "Pin rover"}">${pinClass ? "★" : "☆"}</button>
          </div>
          <span class="fleet-card-state">${state}</span>
        </div>
        <div class="fleet-card-metrics">
          <div class="fleet-card-metric">
            <span class="fleet-card-label">Battery</span>
            <span class="fleet-card-value">🔋 ${batteryPct}%</span>
          </div>
          <div class="fleet-card-metric">
            <span class="fleet-card-label">Solar</span>
            <span class="fleet-card-value">${solarText} (${solarPct}%)</span>
          </div>
          <div class="fleet-card-metric">
            <span class="fleet-card-label">Task</span>
            <span class="fleet-card-value">${taskText}</span>
          </div>
        </div>
      </article>
    `;
  }

  function hideFleetHoverCard() {
    if (!dom.fleetHoverCard) return;
    dom.fleetHoverCard.hidden = true;
    dom.fleetHoverCard.innerHTML = "";
  }

  function showFleetHoverCard(roverId, anchorEl) {
    if (!dom.fleetHoverCard || !isCompactFleetMode()) return;
    const snapshot = fleetSnapshots[roverId];
    if (!snapshot || !anchorEl) return;

    dom.fleetHoverCard.innerHTML = buildFleetCardMarkup(snapshot, false);
    dom.fleetHoverCard.hidden = false;

    const rect = anchorEl.getBoundingClientRect();
    const x = Math.min(window.innerWidth - 300, rect.right + 8);
    const y = Math.min(window.innerHeight - 180, Math.max(8, rect.top));
    dom.fleetHoverCard.style.left = `${Math.max(8, x)}px`;
    dom.fleetHoverCard.style.top = `${Math.max(8, y)}px`;
  }

  function renderFleetGrid() {
    if (!dom.fleetGrid) return;

    const entries = getFleetEntries();
    if (entries.length === 0) {
      dom.fleetGrid.innerHTML =
        '<div class="feed-empty"><span class="feed-empty-icon">🤖</span><span>Awaiting fleet telemetry…</span></div>';
      if (dom.fleetGridSummary) {
        dom.fleetGridSummary.textContent = "Showing 0 of 0 rovers";
      }
      return;
    }

    const compactMode = isCompactFleetMode();
    const visibleEntries = showAllRovers ? entries : getPriorityFleetEntries(entries);

    dom.fleetGrid.innerHTML = visibleEntries
      .map((snapshot) => buildFleetCardMarkup(snapshot, compactMode))
      .join("");

    if (dom.fleetGridSummary) {
      const mode = showAllRovers ? "all" : "priority";
      dom.fleetGridSummary.textContent =
        `Showing ${visibleEntries.length} of ${entries.length} rovers (${mode})`;
    }
    if (dom.toggleFleetScopeBtn) {
      dom.toggleFleetScopeBtn.textContent = showAllRovers
        ? "Show Priority"
        : "Show All";
    }
  }

  function updateFleetBanner() {
    const entries = getFleetEntries();
    const total = entries.length;
    const counts = {
      IDLE: 0,
      EXECUTING: 0,
      SAFE_MODE: 0,
      ERROR: 0,
      UNKNOWN: 0,
    };

    let batterySum = 0;
    let solarSum = 0;
    let riskSum = 0;
    entries.forEach((snapshot) => {
      const state = normalizeState(snapshot.state);
      counts[state] = (counts[state] || 0) + 1;
      batterySum += snapshot.battery || 0;
      solarSum += snapshot.solar_exposure || 0;
      riskSum += Number(snapshot.predicted_fault_probability || 0);
    });

    const avgBattery = total === 0 ? 0 : Math.round((batterySum / total) * 100);
    const avgSolar = total === 0 ? 0 : Math.round((solarSum / total) * 100);
    const avgRisk = total === 0 ? 0 : (riskSum / total) * 100;

    if (dom.fleetTotalRovers) dom.fleetTotalRovers.textContent = String(total);
    if (dom.fleetStateDistribution) {
      dom.fleetStateDistribution.textContent =
        `${counts.IDLE || 0} IDLE · ${counts.EXECUTING || 0} EXECUTING · ${counts.SAFE_MODE || 0} SAFE_MODE`;
    }
    if (dom.fleetAvgBattery) dom.fleetAvgBattery.textContent = `${avgBattery}%`;
    if (dom.fleetAvgSolar) dom.fleetAvgSolar.textContent = `${avgSolar}%`;
    if (dom.fleetAvgRisk) dom.fleetAvgRisk.textContent = `${avgRisk.toFixed(1)}%`;
    if (dom.fleetCommandAck) {
      dom.fleetCommandAck.textContent =
        `${trafficCounters.commandsSent} / ${trafficCounters.acksReceived}`;
    }
    if (dom.fleetMissionContext) {
      const steps = getMissionStepList();
      const stepTotal = steps.length;
      const stepCurrent = stepTotal > 0
        ? Math.min(missionGuideState.activeStepIndex + 1, stepTotal)
        : 0;
      dom.fleetMissionContext.textContent =
        `${missionGuideState.currentMissionPhase || "mission"} · Step ${stepCurrent}/${stepTotal || 0}`;
    }
  }

  function refreshRoverSelectOptions() {
    if (!dom.roverTargetSelect) return;

    const currentValue = dom.roverTargetSelect.value || roverTargetMode;
    const roverIds = getFleetEntries().map((entry) => entry.rover_id);

    dom.roverTargetSelect.innerHTML =
      '<option value="auto">Auto-Select (Best Rover)</option>';

    roverIds.forEach((roverId) => {
      const option = document.createElement("option");
      option.value = roverId;
      option.textContent = formatRoverLabel(roverId);
      dom.roverTargetSelect.appendChild(option);
    });

    if (
      currentValue &&
      (currentValue === AUTO_ROVER_MODE || roverIds.includes(currentValue))
    ) {
      dom.roverTargetSelect.value = currentValue;
    } else {
      dom.roverTargetSelect.value = AUTO_ROVER_MODE;
    }

    roverTargetMode = dom.roverTargetSelect.value;
  }

  function updateFleetUi() {
    renderFleetGrid();
    updateFleetBanner();
    refreshRoverSelectOptions();
    if (!isCompactFleetMode()) {
      hideFleetHoverCard();
    }
  }

  function togglePinnedRover(roverId) {
    if (!roverId) return;
    if (pinnedRoverIds.has(roverId)) pinnedRoverIds.delete(roverId);
    else pinnedRoverIds.add(roverId);
    persistPinnedRovers();
    updateFleetUi();
  }

  function attachFleetInteractions() {
    if (dom.toggleFleetScopeBtn) {
      dom.toggleFleetScopeBtn.addEventListener("click", () => {
        showAllRovers = !showAllRovers;
        updateFleetUi();
      });
    }

    if (!dom.fleetGrid) return;

    dom.fleetGrid.addEventListener("click", (event) => {
      const pinBtn = event.target.closest(".fleet-pin-btn");
      if (pinBtn) {
        event.preventDefault();
        event.stopPropagation();
        togglePinnedRover(pinBtn.dataset.roverId);
        return;
      }

      const card = event.target.closest(".fleet-card");
      if (!card) return;
      const roverId = card.dataset.roverId;
      if (roverId) setSelectedRover(roverId);
    });

    dom.fleetGrid.addEventListener("mouseover", (event) => {
      if (!isCompactFleetMode()) return;
      const card = event.target.closest(".fleet-card");
      if (!card) return;
      showFleetHoverCard(card.dataset.roverId, card);
    });

    dom.fleetGrid.addEventListener("focusin", (event) => {
      if (!isCompactFleetMode()) return;
      const card = event.target.closest(".fleet-card");
      if (!card) return;
      showFleetHoverCard(card.dataset.roverId, card);
    });

    dom.fleetGrid.addEventListener("mouseout", (event) => {
      if (!isCompactFleetMode()) return;
      const related = event.relatedTarget;
      if (related && related.closest && related.closest(".fleet-card")) return;
      hideFleetHoverCard();
    });

    dom.fleetGrid.addEventListener("focusout", () => {
      if (!isCompactFleetMode()) return;
      hideFleetHoverCard();
    });
  }

  function setSelectedRover(roverId) {
    if (!roverId) return;
    sim.setSelectedRover(roverId);
    if (dom.topoRoverLabel) {
      dom.topoRoverLabel.textContent = formatRoverLabel(roverId);
    }
  }

  function scoreRover(snapshot) {
    const battery = Number(snapshot.battery || 0);
    const solar = Number(snapshot.solar_exposure || 0);
    const totalSteps = Number(snapshot.task_total_steps || 10) || 10;
    const progress = Number(snapshot.task_progress || 0) / totalSteps;
    return battery * 0.65 + solar * 0.25 + progress * 0.1;
  }

  function chooseAutoRover(commandType) {
    const entries = getFleetEntries();
    if (entries.length === 0) return null;

    const idle = entries.filter((entry) => normalizeState(entry.state) === "IDLE");
    const executing = entries.filter(
      (entry) => normalizeState(entry.state) === "EXECUTING",
    );
    const safeMode = entries.filter(
      (entry) => normalizeState(entry.state) === "SAFE_MODE",
    );

    const highestScore = (list) =>
      [...list].sort((a, b) => scoreRover(b) - scoreRover(a))[0] || null;

    if (commandType === "START_TASK") {
      return (highestScore(idle) || highestScore(entries))?.rover_id || null;
    }

    if (commandType === "ABORT") {
      const byProgress = [...executing].sort(
        (a, b) => (b.task_progress || 0) - (a.task_progress || 0),
      );
      return (byProgress[0] || highestScore(entries))?.rover_id || null;
    }

    if (commandType === "GO_SAFE") {
      return (highestScore(executing) || highestScore(entries))?.rover_id || null;
    }

    if (commandType === "RESET") {
      const lowestBatterySafe = [...safeMode].sort(
        (a, b) => (a.battery || 0) - (b.battery || 0),
      );
      return (lowestBatterySafe[0] || highestScore(entries))?.rover_id || null;
    }

    return highestScore(entries)?.rover_id || null;
  }

  function resolveTargetRover(commandType) {
    if (roverTargetMode !== AUTO_ROVER_MODE) {
      return roverTargetMode;
    }
    return chooseAutoRover(commandType);
  }

  function dispatchCommand(commandType, taskId = null) {
    let roverId = resolveTargetRover(commandType);
    let taskOptions = null;
    let resolvedTaskId = taskId;

    if (commandType === "START_TASK") {
      resolvedTaskId = taskId || ensureDispatchTaskId();
      const selectedTask = getSelectedTaskConfig(resolvedTaskId);
      if (roverTargetMode === AUTO_ROVER_MODE) {
        const assignment = sim.selectBestRoverForTask(
          selectedTask.task_id,
          selectedTask,
        );
        roverId = assignment?.selected_rover || null;
        if (!roverId) {
          addFeedLine(
            "ack-fail",
            `No rover available for ${selectedTask.task_type}/${selectedTask.difficulty_level}: ${assignment?.reject_reason || "feasibility check failed"}`,
          );
          return null;
        }
        const topScore = assignment?.scored_candidates?.[0];
        taskOptions = {
          ...assignment.task_request,
          selected_rover: roverId,
          predicted_fault_probability: topScore?.predicted_fault_probability ?? null,
          assignment_score_breakdown: topScore?.score_breakdown ?? null,
          reject_reason: assignment?.reject_reason || null,
        };
      } else {
        taskOptions = {
          ...selectedTask,
          selected_rover: roverId,
        };
      }
    }

    if (!roverId) {
      addFeedLine("ack-fail", "No rover available for command dispatch");
      return null;
    }

    setSelectedRover(roverId);
    const cmdId = sim.sendCommand(commandType, resolvedTaskId, roverId, taskOptions);
    if (roverTargetMode === AUTO_ROVER_MODE && cmdId) {
      const taskSuffix =
        commandType === "START_TASK" && taskOptions
          ? ` (${taskOptions.task_type}/${taskOptions.difficulty_level})`
          : "";
      addFeedLine("system", `🎯 Auto-selected ${formatRoverLabel(roverId)} for ${commandType}${taskSuffix}`);
    }
    if (commandType === "START_TASK" && cmdId) {
      advanceMissionGuideAfterDispatch();
      markTaskIdAsDispatched();
    }
    return cmdId;
  }

  function setRoverTargetMode(nextMode) {
    if (!nextMode) return;

    if (nextMode === AUTO_ROVER_MODE) {
      roverTargetMode = AUTO_ROVER_MODE;
      if (dom.roverTargetSelect) dom.roverTargetSelect.value = AUTO_ROVER_MODE;
      addFeedLine("system", "🎯 Auto-select enabled for command routing");
      return;
    }

    const fleet = sim.getFleetState();
    if (!fleet[nextMode]) return;
    roverTargetMode = nextMode;
    if (dom.roverTargetSelect) dom.roverTargetSelect.value = nextMode;
    setSelectedRover(nextMode);
    addFeedLine("system", `🎮 Manual rover selection: ${formatRoverLabel(nextMode)}`);
  }

  function applyScenarioFromUrl() {
    const params = new URLSearchParams(window.location.search || "");
    const scenario = String(params.get("scenario") || "").toLowerCase();
    const missionPreset = safeMissionPresetKey(
      params.get("mission") || missionGuideState.presetKey,
    );
    setMissionPreset(missionPreset, { announce: false, resetProgress: true });

    const taskId = String(params.get("task") || dom.taskIdInput?.value || "").trim();
    const taskType = String(params.get("task_type") || dom.taskTypeSelect?.value || "movement")
      .trim()
      .toLowerCase();
    const difficultyLevel = String(
      params.get("difficulty") || dom.taskDifficultySelect?.value || "L2",
    )
      .trim()
      .toUpperCase();
    const targetSite = String(params.get("target_site") || "").trim();
    const requestedView = String(params.get("view") || "").toLowerCase();
    const open3d = String(params.get("open3d") || "").toLowerCase();
    const delayMs = Math.max(
      0,
      Math.round(Number(params.get("delay_ms")) || 1800),
    );

    if (dom.taskIdInput && taskId) {
      dom.taskIdInput.value = taskId;
      taskIdDirty = true;
    }
    if (dom.taskTypeSelect && taskType) dom.taskTypeSelect.value = taskType;
    if (dom.taskDifficultySelect && difficultyLevel) {
      dom.taskDifficultySelect.value = difficultyLevel;
    }
    if (dom.targetSiteInput && targetSite) {
      dom.targetSiteInput.value = targetSite;
    }
    if (!taskId) {
      taskIdDirty = false;
      syncTaskIdInput();
    } else {
      updateTaskIdPreview();
    }

    if (open3d === "1" || open3d === "true" || open3d === "yes") {
      setTimeout(() => set3DPanel(true), 80);
    }

    if (requestedView === "orbital") {
      setTimeout(() => setVisualizationMode(), 120);
    }

    const theme = String(params.get("theme") || "").toLowerCase();
    if (theme === "light") {
      isLight = true;
      document.body.classList.add("theme-light");
    } else if (theme === "dark") {
      isLight = false;
      document.body.classList.remove("theme-light");
    }

    const explicitTarget = params.get("target");
    if (explicitTarget) {
      setRoverTargetMode(
        explicitTarget.toLowerCase() === AUTO_ROVER_MODE
          ? AUTO_ROVER_MODE
          : explicitTarget,
      );
    }

    const autoStart = String(params.get("autostart") || "").toLowerCase();
    if (autoStart === "1" || autoStart === "true" || autoStart === "start_task") {
      setTimeout(() => dispatchCommand("START_TASK", taskId), delayMs);
    }

    if (params.get("go_safe") === "1") {
      setTimeout(() => dispatchCommand("GO_SAFE"), delayMs + 2500);
    }

    if (params.get("reset") === "1") {
      setTimeout(() => dispatchCommand("RESET"), delayMs + 5000);
    }

    switch (scenario) {
      case "basic-auto":
        setRoverTargetMode(AUTO_ROVER_MODE);
        setTimeout(() => dispatchCommand("START_TASK", taskId || ensureDispatchTaskId()), delayMs);
        break;
      case "manual-select": {
        const manualRover = params.get("manual_rover") || "rover-2";
        setRoverTargetMode(manualRover);
        setTimeout(() => dispatchCommand("START_TASK", taskId || ensureDispatchTaskId()), delayMs);
        break;
      }
      case "safe-mode":
        setRoverTargetMode(AUTO_ROVER_MODE);
        setTimeout(() => dispatchCommand("START_TASK", taskId || ensureDispatchTaskId()), delayMs);
        setTimeout(() => dispatchCommand("GO_SAFE"), delayMs + 3000);
        break;
      case "battery-select":
        setRoverTargetMode(AUTO_ROVER_MODE);
        setTimeout(() => dispatchCommand("START_TASK", taskId || ensureDispatchTaskId()), delayMs);
        break;
      default:
        break;
    }
  }

  function syncFleetFromController() {
    const fleet = sim.getFleetState();
    Object.keys(fleet || {}).forEach((roverId) => {
      upsertFleetSnapshot(roverId, fleet[roverId]);
    });
    updateFleetUi();
  }

  // Initialize fleet cache from simulation startup state
  initializeAccordions();
  loadPinnedRovers();
  attachFleetInteractions();

  const initialFleet = sim.getFleetState();
  Object.keys(initialFleet).forEach((roverId) => {
    upsertFleetSnapshot(roverId, initialFleet[roverId]);
  });
  setSelectedRover(sim.getSelectedRover());
  updateFleetUi();
  setInterval(syncFleetFromController, 1500);

  // ─── Telemetry feed ───
  let feedInitialized = false;
  const MAX_FEED_LINES = 200;

  function addFeedLine(tag, text) {
    if (!feedInitialized) {
      dom.telemetryFeed.innerHTML = "";
      feedInitialized = true;
    }

    const line = document.createElement("div");
    line.className = "feed-line";

    const now = new Date();
    const ts = [now.getHours(), now.getMinutes(), now.getSeconds()]
      .map((n) => String(n).padStart(2, "0"))
      .join(":");

    line.innerHTML = `<span class="feed-ts">${ts}</span><span class="feed-tag tag-${tag}">[${tag.toUpperCase()}]</span> ${escapeHtml(text)}`;

    dom.telemetryFeed.appendChild(line);
    if (dom.telemetryLastValue) {
      dom.telemetryLastValue.textContent = `${ts} [${tag.toUpperCase()}] ${text}`;
    }

    // Trim old lines
    while (dom.telemetryFeed.children.length > MAX_FEED_LINES) {
      dom.telemetryFeed.removeChild(dom.telemetryFeed.firstChild);
    }

    // Auto-scroll
    dom.telemetryFeed.scrollTop = dom.telemetryFeed.scrollHeight;
  }

  function escapeHtml(text) {
    const div = document.createElement("div");
    div.textContent = text;
    return div.innerHTML;
  }

  // ─── Telemetry Display ───
  bus.on("telemetry:display", (data) => {
    const roverId = data.rover_id || sim.getSelectedRover();
    const stateClass = stateToClass(data.state);

    if (roverId) {
      upsertFleetSnapshot(roverId, data);
      updateFleetUi();
      if (dom.topoRoverLabel) {
        dom.topoRoverLabel.textContent = formatRoverLabel(roverId);
      }
    }

    // Selected rover topo node state
    dom.roverState.textContent = normalizeState(data.state);
    dom.roverState.className = `topo-node-state ${stateClass}`;

    // Add telemetry to feed
    const stateIcons = {
      IDLE: "🟢",
      EXECUTING: "🔵",
      SAFE_MODE: "🟡",
      ERROR: "🔴",
    };
    const normalizedState = normalizeState(data.state);
    const icon = stateIcons[normalizedState] || "⚪";
    const batteryValue = Number(data.battery || 0);
    const batteryPct = Math.round(batteryValue * 100);
    const batteryIcon = batteryValue < 0.2 ? "🔋❗" : batteryValue < 0.5 ? "🔋⚠️" : "🔋";

    let feedText =
      `${icon} ${normalizedState} | ${batteryIcon} ${batteryPct}%` +
      `${roverId ? ` | 🤖 ${formatRoverLabel(roverId)}` : ""}`;
    if (data.task_id) feedText += ` | 📋 ${data.task_id}`;
    if (data.task_progress) feedText += ` (${data.task_progress}/${data.task_total_steps || "?"})`;
    if (data.active_task_type && data.active_task_difficulty) {
      feedText += ` | 🧭 ${data.active_task_type}/${data.active_task_difficulty}`;
    }
    if (Number.isFinite(Number(data.predicted_fault_probability))) {
      feedText += ` | 📉 risk ${(Number(data.predicted_fault_probability) * 100).toFixed(1)}%`;
    }
    if (data.fault) feedText += ` | ⚠️ ${data.fault}`;

    addFeedLine("tlm", feedText);

    if (dom.lunarMeta && data.position && !document.body.classList.contains("three-panel-open")) {
      const lat = Number(data.position.lat || 0).toFixed(2);
      const lon = Number(data.position.lon || 0).toFixed(2);
      dom.lunarMeta.textContent = `Lat ${lat}, Lon ${lon}`;
    }
  });

  bus.on("earth:telemetry", (data) => {
    const roverId = data?.rover_id;
    if (!roverId) return;
    upsertFleetSnapshot(roverId, data);
    updateFleetUi();
  });

  bus.on("fleet:update", (fleetData) => {
    Object.keys(fleetData || {}).forEach((roverId) => {
      upsertFleetSnapshot(roverId, fleetData[roverId]);
    });
    updateFleetUi();
  });

  bus.on("earth:selected-rover", (data) => {
    if (data?.rover_id && dom.topoRoverLabel) {
      dom.topoRoverLabel.textContent = formatRoverLabel(data.rover_id);
    }
  });

  // ─── Log events to feed ───
  bus.on("log", (data) => {
    addFeedLine(data.tag, data.text);
  });

  // ─── Command Log ───
  let cmdLogInitialized = false;
  const cmdEntries = {};

  bus.on("cmd:sent", (data) => {
    trafficCounters.commandsSent++;
    updateFleetBanner();

    if (!cmdLogInitialized) {
      dom.commandLog.innerHTML = "";
      cmdLogInitialized = true;
    }

    const entry = document.createElement("div");
    entry.className = "cmd-log-entry";
    entry.id = `cmd-entry-${data.cmdId}`;
    entry.innerHTML = `
      <span class="cmd-log-id">${data.cmdId}</span>
      <span class="cmd-log-type">${data.cmdType}${data.taskId ? " " + data.taskId : ""}${data.taskType ? ` (${data.taskType}/${data.difficultyLevel || "L2"})` : ""}</span>
      <span class="cmd-log-target">[${formatRoverLabel(data.roverId)}]</span>
      <span class="cmd-log-status pending">PENDING</span>
    `;

    dom.commandLog.insertBefore(entry, dom.commandLog.firstChild);
    cmdEntries[data.cmdId] = entry;
  });

  bus.on("ack:resolved", (data) => {
    const entry = cmdEntries[data.cmdId];
    if (!entry) return;

    const statusEl = entry.querySelector(".cmd-log-status");
    if (data.status === "ACCEPTED") {
      statusEl.textContent = `✅ ACCEPTED (${data.rtt?.toFixed(2) || "?"}s)`;
      statusEl.className = "cmd-log-status accepted";
    } else if (data.status === "TIMEOUT") {
      statusEl.textContent = "⏰ TIMEOUT";
      statusEl.className = "cmd-log-status timeout";
    } else {
      statusEl.textContent = `❌ REJECTED`;
      statusEl.className = "cmd-log-status rejected";
    }
  });

  bus.on("earth:ack", () => {
    trafficCounters.acksReceived++;
    updateFleetBanner();
  });

  // ─── Pending ACKs ───
  bus.on("pending:update", (pending) => {
    const keys = Object.keys(pending);
    dom.pendingCount.textContent = keys.length;

    if (keys.length === 0) {
      dom.pendingAcks.innerHTML =
        '<span class="pending-none">No pending commands</span>';
      return;
    }

    dom.pendingAcks.innerHTML = keys
      .map((id) => {
        const info = pending[id];
        const elapsed = (Date.now() / 1000 - info.sentAt).toFixed(1);
        return `<div class="pending-item">
        <span class="pending-id">${id}</span>
        <span>${info.cmdType} [${formatRoverLabel(info.roverId)}]</span>
        <span class="pending-timer">${elapsed}s</span>
      </div>`;
      })
      .join("");
  });

  // ─── Stats ───
  bus.on("stats:update", (stats) => {
    dom.packetsSent.textContent = stats.sent;
    dom.packetsReceived.textContent = stats.received;
    dom.packetsDropped.textContent = stats.dropped;
  });

  // ─── Signal Animation ───
  bus.on("signal:active", ({ direction, delay }) => {
    animateSignal(direction, delay);
  });

  function animateSignal(direction, duration) {
    const particle = document.createElement("div");
    particle.className = `signal-particle ${direction === "UPLINK" ? "uplink" : "downlink"}`;
    dom.topologyVisual.appendChild(particle);

    const earthNode = document.getElementById("topo-earth");
    const spaceLinkNode = document.getElementById("topo-spacelink");
    const roverNode = document.getElementById("topo-rover");

    // Get positions relative to topology container
    const containerRect = dom.topologyVisual.getBoundingClientRect();

    let startEl, endEl;
    if (direction === "UPLINK") {
      startEl = earthNode;
      endEl = roverNode;
    } else {
      startEl = roverNode;
      endEl = earthNode;
    }

    const startRect = startEl.getBoundingClientRect();
    const midRect = spaceLinkNode.getBoundingClientRect();
    const endRect = endEl.getBoundingClientRect();

    const startX = startRect.left + startRect.width / 2 - containerRect.left;
    const startY = startRect.top + startRect.height / 2 - containerRect.top;
    const midX = midRect.left + midRect.width / 2 - containerRect.left;
    const midY = midRect.top + midRect.height / 2 - containerRect.top;
    const endX = endRect.left + endRect.width / 2 - containerRect.left;
    const endY = endRect.top + endRect.height / 2 - containerRect.top;

    particle.style.left = startX + "px";
    particle.style.top = startY + "px";

    const animDuration = Math.min(duration * 1000, 3000);
    const halfDur = animDuration / 2;

    // Animate to midpoint then to endpoint
    const anim = particle.animate(
      [
        { left: startX + "px", top: startY + "px", opacity: 0.2, offset: 0 },
        { left: startX + "px", top: startY + "px", opacity: 1, offset: 0.05 },
        { left: midX + "px", top: midY + "px", opacity: 1, offset: 0.5 },
        { left: endX + "px", top: endY + "px", opacity: 1, offset: 0.92 },
        { left: endX + "px", top: endY + "px", opacity: 0, offset: 1 },
      ],
      {
        duration: animDuration,
        easing: "ease-in-out",
        fill: "forwards",
      },
    );

    anim.onfinish = () => particle.remove();
  }

  // ─── Topology Lines ───
  function drawTopologyLines() {
    const svg = document.getElementById("topology-lines");
    if (!svg || !dom.topologyVisual) return;
    const containerRect = dom.topologyVisual.getBoundingClientRect();
    if (containerRect.width <= 0 || containerRect.height <= 0) return;

    const nodes = [
      "topo-earth",
      "topo-spacelink",
      "topo-rover",
      "topo-telemetry",
    ];
    const positions = {};

    nodes.forEach((id) => {
      const el = document.getElementById(id);
      if (!el) return;
      const rect = el.getBoundingClientRect();
      positions[id] = {
        x: rect.left + rect.width / 2 - containerRect.left,
        y: rect.top + rect.height / 2 - containerRect.top,
      };
    });

    if (!positions["topo-earth"] || !positions["topo-spacelink"] || !positions["topo-rover"] || !positions["topo-telemetry"]) {
      return;
    }

    svg.innerHTML = "";
    svg.setAttribute(
      "viewBox",
      `0 0 ${containerRect.width} ${containerRect.height}`,
    );

    // Earth ↔ Space Link
    addLine(svg, positions["topo-earth"], positions["topo-spacelink"]);
    // Space Link ↔ Rover
    addLine(svg, positions["topo-spacelink"], positions["topo-rover"]);
    // Earth ↔ Telemetry Monitor (dashed)
    addLine(svg, positions["topo-earth"], positions["topo-telemetry"], true);
  }

  function addLine(svg, p1, p2, dashed = false) {
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttribute("x1", p1.x);
    line.setAttribute("y1", p1.y);
    line.setAttribute("x2", p2.x);
    line.setAttribute("y2", p2.y);
    if (dashed) {
      line.setAttribute("stroke-dasharray", "4 4");
    }
    svg.appendChild(line);
  }

  // Draw lines after layout
  setTimeout(drawTopologyLines, 100);
  window.addEventListener("resize", () => {
    requestAnimationFrame(drawTopologyLines);
    hideFleetHoverCard();
  });

  // ─── Command Buttons ───
  if (dom.roverTargetSelect) {
    dom.roverTargetSelect.addEventListener("change", (e) => {
      roverTargetMode = e.target.value;
      if (roverTargetMode !== AUTO_ROVER_MODE) {
        setSelectedRover(roverTargetMode);
        addFeedLine(
          "system",
          `🎮 Manual rover selection: ${formatRoverLabel(roverTargetMode)}`,
        );
      } else {
        addFeedLine("system", "🎯 Auto-select enabled for command routing");
      }
    });
  }

  if (dom.missionPresetSelect) {
    dom.missionPresetSelect.addEventListener("change", (event) => {
      setMissionPreset(event.target.value, { announce: true, resetProgress: true });
      openMissionControlsCard();
    });
  }

  if (dom.missionApplyBtn) {
    dom.missionApplyBtn.addEventListener("click", () => {
      applyMissionStepToControls(getActiveMissionStep(), {
        announce: true,
        forceTaskId: true,
      });
      openMissionControlsCard();
      pulseButton(dom.missionApplyBtn);
    });
  }

  if (dom.missionNextStepBtn) {
    dom.missionNextStepBtn.addEventListener("click", () => {
      moveMissionActiveStep(1);
      openMissionControlsCard();
      pulseButton(dom.missionNextStepBtn);
    });
  }

  if (dom.missionResetBtn) {
    dom.missionResetBtn.addEventListener("click", () => {
      missionGuideState.completedCount = 0;
      missionGuideState.activeStepIndex = 0;
      setMissionGuidePhase(getActiveMissionPreset().mission_phase);
      applyMissionStepToControls(getActiveMissionStep(), {
        announce: true,
        forceTaskId: true,
      });
      openMissionControlsCard();
      pulseButton(dom.missionResetBtn);
    });
  }

  if (dom.missionStepList) {
    dom.missionStepList.addEventListener("click", (event) => {
      const item = event.target.closest(".mission-step-item");
      if (!item) return;
      const stepIndex = Number(item.dataset.stepIndex);
      if (!Number.isFinite(stepIndex)) return;
      setMissionActiveStep(stepIndex, { announce: true });
      openMissionControlsCard();
    });
  }

  if (dom.taskIdInput) {
    dom.taskIdInput.addEventListener("input", () => {
      if (suppressTaskIdInputTracking) return;
      taskIdDirty = true;
      updateTaskIdPreview();
    });
  }

  if (dom.taskIdAutoToggle) {
    dom.taskIdAutoToggle.addEventListener("change", () => {
      if (dom.taskIdAutoToggle.checked) {
        taskIdDirty = false;
        syncTaskIdInput({ force: true });
        addFeedLine("system", "🧾 Auto task ID generation enabled");
      } else {
        updateTaskIdPreview();
        addFeedLine("system", "🧾 Manual task ID mode enabled");
      }
    });
  }

  if (dom.taskIdRegenerateBtn) {
    dom.taskIdRegenerateBtn.addEventListener("click", () => {
      taskIdDirty = false;
      syncTaskIdInput({ force: true });
      addFeedLine("system", `🧾 Task ID regenerated: ${dom.taskIdInput.value}`);
      pulseButton(dom.taskIdRegenerateBtn);
    });
  }

  if (dom.taskTypeSelect) {
    dom.taskTypeSelect.addEventListener("change", () => {
      syncTaskIdInput();
      updateTaskIdPreview();
    });
  }

  if (dom.taskDifficultySelect) {
    dom.taskDifficultySelect.addEventListener("change", () => {
      syncTaskIdInput();
      updateTaskIdPreview();
    });
  }

  $("#cmd-start-task").addEventListener("click", () => {
    const taskId = ensureDispatchTaskId();
    dispatchCommand("START_TASK", taskId);
    pulseButton($("#cmd-start-task"));
  });

  $("#cmd-abort").addEventListener("click", () => {
    dispatchCommand("ABORT");
    pulseButton($("#cmd-abort"));
  });

  $("#cmd-go-safe").addEventListener("click", () => {
    dispatchCommand("GO_SAFE");
    pulseButton($("#cmd-go-safe"));
  });

  $("#cmd-reset").addEventListener("click", () => {
    dispatchCommand("RESET");
    pulseButton($("#cmd-reset"));
  });

  function pulseButton(btn) {
    btn.style.transform = "scale(0.95)";
    setTimeout(() => {
      btn.style.transform = "";
    }, 150);
  }

  // ─── Clear Telemetry ───
  $("#btn-clear-telemetry").addEventListener("click", () => {
    dom.telemetryFeed.innerHTML =
      '<div class="feed-empty"><span class="feed-empty-icon">📡</span><span>Awaiting telemetry…</span></div>';
    feedInitialized = false;
    if (dom.telemetryLastValue) {
      dom.telemetryLastValue.textContent = "Awaiting telemetry…";
    }
  });

  // ─── Slider Controls ───
  dom.latencySlider.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    sim.updateConfig("baseLatency", val);
    dom.latencyValue.textContent = val.toFixed(1) + "s";
  });

  dom.jitterSlider.addEventListener("input", (e) => {
    const val = parseFloat(e.target.value);
    sim.updateConfig("jitter", val);
    dom.jitterValue.textContent = "±" + val.toFixed(2) + "s";
  });

  dom.dropSlider.addEventListener("input", (e) => {
    const val = parseInt(e.target.value);
    sim.updateConfig("dropRate", val);
    dom.dropValue.textContent = val + "%";
  });

  dom.faultSlider.addEventListener("input", (e) => {
    const val = parseInt(e.target.value);
    sim.updateConfig("faultProbability", val);
    dom.faultProbValue.textContent = `${val > 0 ? "+" : ""}${val}%`;
  });

  // ─── Mission Elapsed Time Clock ───
  function updateMET() {
    const elapsed = sim.getElapsedTime();
    const h = Math.floor(elapsed / 3600);
    const m = Math.floor((elapsed % 3600) / 60);
    const s = Math.floor(elapsed % 60);
    dom.metValue.textContent =
      String(h).padStart(2, "0") +
      ":" +
      String(m).padStart(2, "0") +
      ":" +
      String(s).padStart(2, "0");

    dom.sessionTime.textContent = formatDuration(elapsed);
  }

  function formatDuration(seconds) {
    if (seconds < 60) return Math.floor(seconds) + "s";
    if (seconds < 3600)
      return Math.floor(seconds / 60) + "m " + Math.floor(seconds % 60) + "s";
    return (
      Math.floor(seconds / 3600) +
      "h " +
      Math.floor((seconds % 3600) / 60) +
      "m"
    );
  }

  setInterval(updateMET, 1000);

  // ─── Theme Toggle ───
  let isLight = false;
  $("#btn-theme").addEventListener("click", () => {
    isLight = !isLight;
    document.body.classList.toggle("theme-light", isLight);
  });

  // ─── Node Click Highlighting ───
  $$(".topo-node").forEach((node) => {
    node.addEventListener("click", () => {
      $$(".topo-node").forEach((n) => n.classList.remove("active"));
      node.classList.add("active");
    });
  });

  // ─── Keyboard Shortcuts ───
  document.addEventListener("keydown", (e) => {
    // Don't capture if user is typing in an input
    if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName)) return;

    if (e.key === "Escape" && floatingPanel.classList.contains("panel-open")) {
      set3DPanel(false);
      return;
    }

    switch (e.key.toLowerCase()) {
      case "s":
        if (e.shiftKey) {
          dispatchCommand("GO_SAFE");
          pulseButton($("#cmd-go-safe"));
        } else {
          const taskId = ensureDispatchTaskId();
          dispatchCommand("START_TASK", taskId);
          pulseButton($("#cmd-start-task"));
        }
        break;
      case "a":
        dispatchCommand("ABORT");
        pulseButton($("#cmd-abort"));
        break;
      case "r":
        dispatchCommand("RESET");
        pulseButton($("#cmd-reset"));
        break;
    }
  });

  // ─── Initial Log ───
  addFeedLine("system", "🌍 Earth Station online");
  addFeedLine("system", "🛰️ Space Link relay initialized");
  addFeedLine("system", "🤖 Rover fleet active — awaiting commands");
  addFeedLine("system", "📡 Telemetry monitor listening");
  addFeedLine("system", "───────────────────────────────");
  applyScenarioFromUrl();

  console.log(
    "%c LSOAS Mission Control ",
    "background: #0a0a0f; color: #22d3ee; font-size: 16px; font-weight: bold; padding: 8px 16px; border-radius: 4px;",
  );
  console.log(
    "%c Keyboard: S=Start  A=Abort  Shift+S=Safe  R=Reset",
    "color: #888; font-size: 11px;",
  );
})();
