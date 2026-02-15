/* ═══════════════════════════════════════════════════════
   LSOAS — Application UI Controller
   Connects the Simulation Engine to the DOM.
   ═══════════════════════════════════════════════════════ */

(function () {
  "use strict";

  // ─── DOM References ───
  const $ = (sel) => document.querySelector(sel);
  const $$ = (sel) => document.querySelectorAll(sel);

  // ─── Boot ───
  const sim = new SimulationController();
  const bus = sim.bus;

  // ─── 3D Visualization ───
  let viz = null;
  if (window.THREE) {
    viz = new VisualizationController(bus, "visualization-container");
  }

  // Toggle 3D Panel
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
      btnToggleLabel.textContent = open ? "Close 3D" : "3D View";
    }

    if (open) {
      floatingPanel.setAttribute("aria-hidden", "false");
      document.body.classList.add("three-panel-open");
      if (panelTelemetry) panelTelemetry.classList.add("compact-log");

      requestAnimationFrame(() => {
        requestAnimationFrame(() => {
          floatingPanel.classList.add("panel-open");
        });
      });
    } else {
      floatingPanel.classList.remove("panel-open");
      floatingPanel.setAttribute("aria-hidden", "true");
      panelLayoutTimer = setTimeout(() => {
        document.body.classList.remove("three-panel-open");
        if (panelTelemetry) panelTelemetry.classList.remove("compact-log");
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
    fleetGrid: $("#fleet-grid"),
    fleetTotalRovers: $("#fleet-total-rovers"),
    fleetStateDistribution: $("#fleet-state-distribution"),
    fleetAvgBattery: $("#fleet-avg-battery"),
    fleetCommandAck: $("#fleet-command-ack"),
    telemetryFeed: $("#telemetry-feed"),
    telemetryLastValue: $("#telemetry-last-value"),
    lunarMeta: $("#lunar-meta"),
    commandLog: $("#command-log"),
    pendingAcks: $("#pending-acks"),
    pendingCount: $("#pending-count"),
    packetsSent: $("#packets-sent"),
    packetsReceived: $("#packets-received"),
    packetsDropped: $("#packets-dropped"),
    taskIdInput: $("#task-id-input"),
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
  let roverTargetMode = AUTO_ROVER_MODE;

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

  function renderFleetGrid() {
    if (!dom.fleetGrid) return;

    const entries = getFleetEntries();
    if (entries.length === 0) {
      dom.fleetGrid.innerHTML =
        '<div class="feed-empty"><span class="feed-empty-icon">🤖</span><span>Awaiting fleet telemetry…</span></div>';
      return;
    }

    dom.fleetGrid.innerHTML = entries
      .map((snapshot) => {
        const state = normalizeState(snapshot.state);
        const stateClass = stateToClass(state);
        const batteryPct = Math.round((snapshot.battery || 0) * 100);
        const solarExposure = snapshot.solar_exposure || 0;
        const solarText = solarExposure >= 0.5 ? "☀ Sun" : "🌑 Dark";
        const solarPct = Math.round(solarExposure * 100);
        const taskText = snapshot.task_id
          ? `${snapshot.task_id}${snapshot.task_progress !== null && snapshot.task_progress !== undefined ? ` (${snapshot.task_progress}/10)` : ""}`
          : "—";

        return `
          <article class="fleet-card ${stateClass}" data-rover-id="${snapshot.rover_id}">
            <div class="fleet-card-header">
              <span class="fleet-card-rover-id">${(snapshot.rover_id || "rover-?").toUpperCase()}</span>
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
      })
      .join("");
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
    entries.forEach((snapshot) => {
      const state = normalizeState(snapshot.state);
      counts[state] = (counts[state] || 0) + 1;
      batterySum += snapshot.battery || 0;
    });

    const avgBattery = total === 0 ? 0 : Math.round((batterySum / total) * 100);

    if (dom.fleetTotalRovers) dom.fleetTotalRovers.textContent = String(total);
    if (dom.fleetStateDistribution) {
      dom.fleetStateDistribution.textContent =
        `${counts.IDLE || 0} IDLE · ${counts.EXECUTING || 0} EXECUTING · ${counts.SAFE_MODE || 0} SAFE_MODE`;
    }
    if (dom.fleetAvgBattery) dom.fleetAvgBattery.textContent = `${avgBattery}%`;
    if (dom.fleetCommandAck) {
      dom.fleetCommandAck.textContent =
        `${trafficCounters.commandsSent} / ${trafficCounters.acksReceived}`;
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
    const progress = Number(snapshot.task_progress || 0) / 10;
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
    const roverId = resolveTargetRover(commandType);
    if (!roverId) {
      addFeedLine("ack-fail", "No rover available for command dispatch");
      return null;
    }

    setSelectedRover(roverId);
    const cmdId = sim.sendCommand(commandType, taskId, roverId);
    if (roverTargetMode === AUTO_ROVER_MODE && cmdId) {
      addFeedLine("system", `🎯 Auto-selected ${formatRoverLabel(roverId)} for ${commandType}`);
    }
    return cmdId;
  }

  // Initialize fleet cache from simulation startup state
  const initialFleet = sim.getFleetState();
  Object.keys(initialFleet).forEach((roverId) => {
    upsertFleetSnapshot(roverId, initialFleet[roverId]);
  });
  setSelectedRover(sim.getSelectedRover());
  updateFleetUi();

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
    if (data.task_progress) feedText += ` (${data.task_progress}/10)`;
    if (data.fault) feedText += ` | ⚠️ ${data.fault}`;

    addFeedLine("tlm", feedText);

    if (dom.lunarMeta && data.position) {
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
      <span class="cmd-log-type">${data.cmdType}${data.taskId ? " " + data.taskId : ""}</span>
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
    const containerRect = dom.topologyVisual.getBoundingClientRect();

    const nodes = [
      "topo-earth",
      "topo-spacelink",
      "topo-rover",
      "topo-telemetry",
    ];
    const positions = {};

    nodes.forEach((id) => {
      const el = document.getElementById(id);
      const rect = el.getBoundingClientRect();
      positions[id] = {
        x: rect.left + rect.width / 2 - containerRect.left,
        y: rect.top + rect.height / 2 - containerRect.top,
      };
    });

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

  $("#cmd-start-task").addEventListener("click", () => {
    const taskId = dom.taskIdInput.value.trim() || "TASK-001";
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
    dom.faultProbValue.textContent = val + "%";
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
          const taskId = dom.taskIdInput.value.trim() || "TASK-001";
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

  console.log(
    "%c LSOAS Mission Control ",
    "background: #0a0a0f; color: #22d3ee; font-size: 16px; font-weight: bold; padding: 8px 16px; border-radius: 4px;",
  );
  console.log(
    "%c Keyboard: S=Start  A=Abort  Shift+S=Safe  R=Reset",
    "color: #888; font-size: 11px;",
  );
})();
