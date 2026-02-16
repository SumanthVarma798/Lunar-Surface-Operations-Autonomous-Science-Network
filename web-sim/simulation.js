/* ═══════════════════════════════════════════════════════
   LSOAS — Simulation Engine
   Ports all ROS node logic to in-browser JavaScript.
   ═══════════════════════════════════════════════════════ */

const DEFAULT_ROVER_IDS = ["rover-1", "rover-2", "rover-3"];
const MIN_ROVER_COUNT = 1;
const MAX_ROVER_COUNT = 8;

function buildRoverIds(count) {
  const roverCount = Math.max(
    MIN_ROVER_COUNT,
    Math.min(MAX_ROVER_COUNT, Number(count) || DEFAULT_ROVER_IDS.length),
  );
  return Array.from({ length: roverCount }, (_, idx) => `rover-${idx + 1}`);
}

function resolveRoverIds() {
  const globalConfig = window.LSOAS_CONFIG || {};
  const configuredIds = globalConfig.roverIds;
  if (Array.isArray(configuredIds) && configuredIds.length > 0) {
    return configuredIds
      .map((id) => String(id || "").trim())
      .filter((id) => id.length > 0);
  }

  const params = new URLSearchParams(window.location.search || "");
  const roverCount = Number(params.get("rovers"));
  if (Number.isFinite(roverCount)) {
    return buildRoverIds(roverCount);
  }

  return [...DEFAULT_ROVER_IDS];
}

function parseBoundedNumber(rawValue, fallback, min, max) {
  const parsed = Number(rawValue);
  if (!Number.isFinite(parsed)) return fallback;
  return Math.max(min, Math.min(max, parsed));
}

function parseRoverMap(rawValue, parser) {
  if (!rawValue) return {};

  const parsed = {};
  rawValue
    .split(",")
    .map((chunk) => chunk.trim())
    .filter((chunk) => chunk.length > 0)
    .forEach((chunk) => {
      const [roverIdRaw, roverValueRaw] = chunk.split(":");
      const roverId = String(roverIdRaw || "").trim();
      if (!roverId) return;
      const roverValue = parser(String(roverValueRaw || "").trim());
      if (roverValue === null || roverValue === undefined) return;
      parsed[roverId] = roverValue;
    });

  return parsed;
}

function resolveRuntimeOptions() {
  const params = new URLSearchParams(window.location.search || "");

  return {
    baseLatency: parseBoundedNumber(params.get("latency"), 1.3, 0, 10),
    jitter: parseBoundedNumber(params.get("jitter"), 0.2, 0, 5),
    dropRate: parseBoundedNumber(params.get("drop"), 5, 0, 100),
    faultProbability: parseBoundedNumber(params.get("fault"), 10, 0, 100),
    batteryByRover: parseRoverMap(params.get("battery"), (raw) => {
      const value = Number(raw);
      if (!Number.isFinite(value)) return null;
      const normalized = value > 1 ? value / 100 : value;
      return Math.max(0, Math.min(1, normalized));
    }),
    stateByRover: parseRoverMap(params.get("state"), (raw) =>
      String(raw || "")
        .trim()
        .toUpperCase(),
    ),
  };
}

function roverTopic(roverId, channel) {
  return `${roverId}:${channel}`;
}

class EventBus {
  constructor() {
    this._listeners = {};
  }
  on(event, fn) {
    (this._listeners[event] ||= []).push(fn);
  }
  off(event, fn) {
    this._listeners[event] = (this._listeners[event] || []).filter(
      (f) => f !== fn,
    );
  }
  emit(event, data) {
    (this._listeners[event] || []).forEach((fn) => fn(data));
  }
}

// ─── Rover Node ───
class RoverNode {
  static STATE_IDLE = "IDLE";
  static STATE_EXECUTING = "EXECUTING";
  static STATE_SAFE_MODE = "SAFE_MODE";
  static STATE_ERROR = "ERROR";

  constructor(bus, config, roverId, startPosition = null) {
    this.bus = bus;
    this.config = config;
    this.roverId = roverId;
    this.state = RoverNode.STATE_IDLE;
    this.currentTaskId = null;
    this.taskCounter = 0;
    this.batteryLevel = 1.0;
    this.lastFault = null;
    this.commandTopic = roverTopic(this.roverId, "command");
    this.telemetryTopic = roverTopic(this.roverId, "telemetry");
    this.ackTopic = roverTopic(this.roverId, "ack");

    // Initial position (Near crater Tycho, offset by rover)
    this.position = startPosition || { lat: -43.3, lon: -11.2 };

    this.commandListener = (data) => this.commandCallback(data);
    this.bus.on(this.commandTopic, this.commandListener);

    // Start telemetry publishing (every 2s)
    this.telemetryInterval = setInterval(() => this.publishTelemetry(), 2000);
    // Start task execution tick (every 1s)
    this.taskInterval = setInterval(() => this.executeTaskStep(), 1000);
  }

  commandCallback(cmdData) {
    const cmdId = cmdData.cmd_id || "unknown";
    const cmdType = cmdData.type;
    const taskId = cmdData.task_id;

    this.bus.emit("log", {
      tag: "cmd",
      text: `[${this.roverId}] received [${cmdId}]: ${cmdType}`,
    });

    let success, reason;

    switch (cmdType) {
      case "START_TASK":
        [success, reason] = this.handleStartTask(taskId);
        break;
      case "ABORT":
        [success, reason] = this.handleAbort();
        break;
      case "GO_SAFE":
        [success, reason] = this.handleGoSafe();
        break;
      case "RESET":
        [success, reason] = this.handleReset();
        break;
      default:
        success = false;
        reason = `Unknown command type: ${cmdType}`;
    }

    this.sendAck(cmdId, success, reason);
  }

  handleStartTask(taskId) {
    if (this.state === RoverNode.STATE_SAFE_MODE) {
      return [
        false,
        "Cannot start task: Rover in SAFE_MODE. Send RESET first.",
      ];
    }
    if (this.state === RoverNode.STATE_EXECUTING) {
      return [
        false,
        `Cannot start task: Already executing task ${this.currentTaskId}`,
      ];
    }
    this.state = RoverNode.STATE_EXECUTING;
    this.currentTaskId = taskId;
    this.taskCounter = 0;
    this.bus.emit("log", {
      tag: "task",
      text: `✅ [${this.roverId}] Started task: ${taskId}`,
    });
    return [true, null];
  }

  handleAbort() {
    if (this.state === RoverNode.STATE_EXECUTING) {
      this.bus.emit("log", {
        tag: "system",
        text: `⏹ [${this.roverId}] Aborting task: ${this.currentTaskId}`,
      });
      this.state = RoverNode.STATE_IDLE;
      this.currentTaskId = null;
      this.taskCounter = 0;
    }
    return [true, null];
  }

  handleGoSafe() {
    this.state = RoverNode.STATE_SAFE_MODE;
    this.currentTaskId = null;
    this.taskCounter = 0;
    if (!this.lastFault) this.lastFault = "Commanded to SAFE_MODE";
    this.bus.emit("log", {
      tag: "system",
      text: `🛡 [${this.roverId}] entering SAFE_MODE`,
    });
    return [true, null];
  }

  handleReset() {
    if (this.state === RoverNode.STATE_SAFE_MODE) {
      this.state = RoverNode.STATE_IDLE;
      this.currentTaskId = null;
      this.taskCounter = 0;
      this.lastFault = null;
      this.bus.emit("log", {
        tag: "system",
        text: `↻ [${this.roverId}] RESET: SAFE_MODE → IDLE`,
      });
    }
    return [true, null];
  }

  sendAck(cmdId, success, reason = null) {
    const ackData = {
      ack_id: cmdId,
      rover_id: this.roverId,
      status: success ? "ACCEPTED" : "REJECTED",
      reason,
      ts: Date.now() / 1000,
    };
    this.bus.emit(this.ackTopic, ackData);
  }

  executeTaskStep() {
    if (this.state !== RoverNode.STATE_EXECUTING) return;

    this.taskCounter++;
    this.batteryLevel = Math.max(0.0, this.batteryLevel - 0.005);

    // Simulate movement
    this.position.lat += (Math.random() - 0.5) * 0.01;
    this.position.lon += (Math.random() - 0.5) * 0.01;

    // Fault detection
    const faultProb = (this.config.faultProbability ?? 10) / 100;
    if (Math.random() < faultProb) {
      const faultMsg = `Fault during task execution (step ${this.taskCounter})`;
      this.bus.emit("log", {
        tag: "fault",
        text: `🚨 [${this.roverId}] FAULT DETECTED during task ${this.currentTaskId}!`,
      });
      this.state = RoverNode.STATE_SAFE_MODE;
      this.lastFault = faultMsg;
      this.currentTaskId = null;
      this.taskCounter = 0;
      return;
    }

    // Task completion after 10 steps
    if (this.taskCounter >= 10) {
      this.bus.emit("log", {
        tag: "task",
        text: `✅ [${this.roverId}] Task ${this.currentTaskId} completed!`,
      });
      this.state = RoverNode.STATE_IDLE;
      this.currentTaskId = null;
      this.taskCounter = 0;
    } else {
      this.bus.emit("log", {
        tag: "task",
        text: `⚙️ [${this.roverId}] Executing ${this.currentTaskId}: step ${this.taskCounter}/10`,
      });
    }
  }

  publishTelemetry() {
    const telemetry = {
      ts: Date.now() / 1000,
      rover_id: this.roverId,
      state: this.state,
      task_id: this.currentTaskId,
      battery: Math.round(this.batteryLevel * 100) / 100,
      fault: this.lastFault,
      task_progress:
        this.state === RoverNode.STATE_EXECUTING ? this.taskCounter : null,
      position: { lat: this.position.lat, lon: this.position.lon },
    };
    this.bus.emit(this.telemetryTopic, telemetry);
  }

  destroy() {
    clearInterval(this.telemetryInterval);
    clearInterval(this.taskInterval);
    this.bus.off(this.commandTopic, this.commandListener);
  }
}

// ─── Space Link Node ───
class SpaceLinkNode {
  constructor(bus, config) {
    this.bus = bus;
    this.config = config;
    this.stats = { sent: 0, received: 0, dropped: 0 };
    this.defaultRoverId = (config.roverIds || [DEFAULT_ROVER_IDS[0]])[0];
    this.subscribedRovers = new Set();

    // Uplink: earth:uplink_cmd → (delay) → <rover-id>:command
    this.bus.on("earth:uplink_cmd", (data) =>
      this.relayCommand(data),
    );

    (this.config.roverIds || [this.defaultRoverId]).forEach((roverId) =>
      this.ensureRoverSubscriptions(roverId),
    );
  }

  ensureRoverSubscriptions(roverId) {
    if (this.subscribedRovers.has(roverId)) return;
    this.subscribedRovers.add(roverId);

    this.bus.on(roverTopic(roverId, "telemetry"), (data) =>
      this.relay(
        { ...data, rover_id: data.rover_id || roverId },
        "earth:telemetry",
        "DOWNLINK-TLM",
        roverId,
      ),
    );

    this.bus.on(roverTopic(roverId, "ack"), (data) =>
      this.relay(
        { ...data, rover_id: data.rover_id || roverId },
        "earth:ack",
        "DOWNLINK-ACK",
        roverId,
      ),
    );
  }

  relayCommand(data) {
    const roverId = data?.rover_id || this.defaultRoverId;
    this.ensureRoverSubscriptions(roverId);
    this.relay(
      { ...data, rover_id: roverId },
      roverTopic(roverId, "command"),
      "UPLINK",
      roverId,
    );
  }

  relay(data, targetEvent, direction, roverId = null) {
    this.stats.sent++;

    const dropRate = (this.config.dropRate ?? 5) / 100;
    if (Math.random() < dropRate) {
      this.stats.dropped++;
      this.bus.emit("log", {
        tag: "drop",
        text: `❌ ${direction}${roverId ? ` [${roverId}]` : ""} DROPPED`,
      });
      this.bus.emit("stats:update", this.stats);
      return;
    }

    const baseLatency = this.config.baseLatency ?? 1.3;
    const jitter = this.config.jitter ?? 0.2;
    const jitterValue = (Math.random() * 2 - 1) * jitter;
    const delay = Math.max(0.05, baseLatency + jitterValue);

    this.bus.emit("log", {
      tag: "relay",
      text: `📡 ${direction}${roverId ? ` [${roverId}]` : ""} relay (${delay.toFixed(2)}s delay)`,
    });
    this.bus.emit("signal:active", { direction, delay, rover_id: roverId });

    setTimeout(() => {
      this.stats.received++;
      this.bus.emit(targetEvent, data);
      this.bus.emit("stats:update", this.stats);
    }, delay * 1000);
  }
}

// ─── Earth Node ───
class EarthNode {
  constructor(bus, config) {
    this.bus = bus;
    this.config = config;
    this.roverIds =
      Array.isArray(config.roverIds) && config.roverIds.length > 0
        ? [...config.roverIds]
        : [DEFAULT_ROVER_IDS[0]];
    this.selectedRoverId = this.roverIds[0];
    this.cmdCounter = 0;
    this.pendingCommands = {};
    this.fleetState = {};
    this.ackTimeout = 5.0;
    this.maxRetries = 3;

    this.roverIds.forEach((roverId) => {
      this.fleetState[roverId] = this.buildDefaultFleetEntry(roverId);
    });

    // Listen for ACKs arriving at Earth
    this.bus.on("earth:ack", (data) => this.ackCallback(data));
    this.bus.on("earth:telemetry", (data) => this.telemetryCallback(data));

    // Check for timeouts every 1s
    this.timeoutInterval = setInterval(() => this.checkTimeouts(), 1000);
    this.bus.emit("fleet:update", this.getFleetState());
  }

  buildDefaultFleetEntry(roverId) {
    return {
      rover_id: roverId,
      state: RoverNode.STATE_IDLE,
      battery: 1.0,
      task_id: null,
      fault: null,
      task_progress: null,
      position: null,
      ts: null,
      last_seen: null,
    };
  }

  telemetryCallback(data) {
    const roverId = data.rover_id || this.selectedRoverId || this.roverIds[0];
    if (!roverId) return;

    if (!this.fleetState[roverId]) {
      this.fleetState[roverId] = this.buildDefaultFleetEntry(roverId);
      this.roverIds.push(roverId);
    }

    this.fleetState[roverId] = {
      ...this.fleetState[roverId],
      ...data,
      rover_id: roverId,
      last_seen: Date.now() / 1000,
    };

    this.bus.emit("fleet:update", this.getFleetState());
  }

  getFleetState() {
    const snapshot = {};
    Object.keys(this.fleetState).forEach((roverId) => {
      snapshot[roverId] = { ...this.fleetState[roverId] };
    });
    return snapshot;
  }

  getSelectedRover() {
    return this.selectedRoverId;
  }

  setSelectedRover(roverId) {
    if (!roverId) return false;

    if (!this.fleetState[roverId]) {
      this.fleetState[roverId] = this.buildDefaultFleetEntry(roverId);
      this.roverIds.push(roverId);
    }

    this.selectedRoverId = roverId;
    this.bus.emit("earth:selected-rover", { rover_id: roverId });
    this.bus.emit("log", {
      tag: "system",
      text: `🎯 Selected rover set to [${roverId}]`,
    });
    return true;
  }

  ackCallback(data) {
    const ackId = data.ack_id;
    const status = data.status;
    const reason = data.reason;
    const roverId = data.rover_id || "unknown";

    if (this.pendingCommands[ackId]) {
      const cmdInfo = this.pendingCommands[ackId];
      const rtt = Date.now() / 1000 - cmdInfo.sentAt;

      if (status === "ACCEPTED") {
        this.bus.emit("log", {
          tag: "ack",
          text: `✅ ACK ${ackId} [${roverId}]: ACCEPTED (RTT: ${rtt.toFixed(2)}s)`,
        });
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `❌ ACK ${ackId} [${roverId}]: REJECTED${reason ? ` (${reason})` : ""}`,
        });
      }

      this.bus.emit("ack:resolved", {
        cmdId: ackId,
        status,
        reason,
        rtt,
        roverId,
      });
      delete this.pendingCommands[ackId];
    } else {
      if (status === "ACCEPTED") {
        this.bus.emit("log", {
          tag: "ack",
          text: `✅ ACK ${ackId} [${roverId}]: ACCEPTED (late)`,
        });
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `❌ ACK ${ackId} [${roverId}]: REJECTED${reason ? ` (${reason})` : ""}`,
        });
      }
    }

    this.bus.emit("pending:update", this.pendingCommands);
  }

  checkTimeouts() {
    const now = Date.now() / 1000;
    const timedOut = [];

    for (const [cmdId, info] of Object.entries(this.pendingCommands)) {
      if (now - info.sentAt > this.ackTimeout) {
        timedOut.push(cmdId);
      }
    }

    for (const cmdId of timedOut) {
      const info = this.pendingCommands[cmdId];
      if (info.attempt < this.maxRetries) {
        info.attempt++;
        info.sentAt = Date.now() / 1000;
        this.bus.emit("log", {
          tag: "system",
          text: `⏰ No ACK for ${cmdId} [${info.roverId}], retrying (attempt ${info.attempt}/${this.maxRetries})`,
        });
        this.bus.emit("earth:uplink_cmd", info.cmdData);
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `❌ Command ${cmdId} [${info.roverId}] failed after ${this.maxRetries} attempts`,
        });
        this.bus.emit("ack:resolved", {
          cmdId,
          status: "TIMEOUT",
          reason: "Max retries exceeded",
          roverId: info.roverId,
        });
        delete this.pendingCommands[cmdId];
      }
    }

    this.bus.emit("pending:update", this.pendingCommands);
  }

  sendCommand(cmdType, taskId = null, roverId = null) {
    const targetRoverId = roverId || this.selectedRoverId || this.roverIds[0];
    if (!targetRoverId) return null;

    if (!this.fleetState[targetRoverId]) {
      this.fleetState[targetRoverId] =
        this.buildDefaultFleetEntry(targetRoverId);
      this.roverIds.push(targetRoverId);
    }

    this.cmdCounter++;
    const cmdId = `c-${String(this.cmdCounter).padStart(5, "0")}`;

    const cmdData = {
      cmd_id: cmdId,
      rover_id: targetRoverId,
      type: cmdType,
      ts: Date.now() / 1000,
    };

    if (taskId) cmdData.task_id = taskId;

    // Track pending
    this.pendingCommands[cmdId] = {
      sentAt: Date.now() / 1000,
      cmdType,
      attempt: 1,
      cmdData,
      roverId: targetRoverId,
    };

    // Send to space link
    this.bus.emit("earth:uplink_cmd", cmdData);
    this.bus.emit("log", {
      tag: "cmd",
      text: `📤 Sent ${cmdId} -> [${targetRoverId}]: ${cmdType}${taskId ? " " + taskId : ""}`,
    });
    this.bus.emit("cmd:sent", { cmdId, cmdType, taskId, roverId: targetRoverId });
    this.bus.emit("pending:update", this.pendingCommands);

    return cmdId;
  }

  destroy() {
    clearInterval(this.timeoutInterval);
  }
}

// ─── Telemetry Monitor ───
class TelemetryMonitor {
  constructor(bus, earthNode) {
    this.bus = bus;
    this.earthNode = earthNode;
    this.latestTelemetry = null;
    this.latestTelemetryByRover = {};

    this.bus.on("earth:telemetry", (data) => {
      const roverId = data.rover_id || this.earthNode.getSelectedRover();
      if (!roverId) return;

      this.latestTelemetryByRover[roverId] = data;
      if (roverId === this.earthNode.getSelectedRover()) {
        this.latestTelemetry = data;
        this.bus.emit("telemetry:display", data);
      }
    });

    this.bus.on("earth:selected-rover", (data) => {
      const roverId = data?.rover_id;
      if (!roverId) return;
      const latest = this.latestTelemetryByRover[roverId];
      if (latest) {
        this.latestTelemetry = latest;
        this.bus.emit("telemetry:display", latest);
      }
    });

    this.bus.on("earth:ack", (data) => {
      this.bus.emit("ack:display", data);
    });
  }
}

// ─── Simulation Controller ───
class SimulationController {
  constructor() {
    const roverIds = resolveRoverIds();
    const runtimeOptions = resolveRuntimeOptions();
    this.bus = new EventBus();
    this.config = {
      baseLatency: runtimeOptions.baseLatency,
      jitter: runtimeOptions.jitter,
      dropRate: runtimeOptions.dropRate,
      faultProbability: runtimeOptions.faultProbability,
      roverIds,
    };

    this.startTime = Date.now();
    this.rovers = this.config.roverIds.map(
      (roverId, index) =>
        new RoverNode(
          this.bus,
          this.config,
          roverId,
          this.getInitialPositionForRover(index),
        ),
    );
    this.applyRoverRuntimeOverrides(runtimeOptions);
    this.spaceLink = new SpaceLinkNode(this.bus, this.config);
    this.earth = new EarthNode(this.bus, this.config);
    this.telemetryMonitor = new TelemetryMonitor(this.bus, this.earth);
  }

  applyRoverRuntimeOverrides(runtimeOptions) {
    const roversById = new Map(this.rovers.map((rover) => [rover.roverId, rover]));
    const validStates = new Set([
      RoverNode.STATE_IDLE,
      RoverNode.STATE_EXECUTING,
      RoverNode.STATE_SAFE_MODE,
      RoverNode.STATE_ERROR,
    ]);

    Object.entries(runtimeOptions.batteryByRover || {}).forEach(
      ([roverId, batteryLevel]) => {
        const rover = roversById.get(roverId);
        if (!rover) return;
        rover.batteryLevel = batteryLevel;
      },
    );

    Object.entries(runtimeOptions.stateByRover || {}).forEach(
      ([roverId, state]) => {
        const rover = roversById.get(roverId);
        if (!rover) return;
        if (!validStates.has(state)) return;

        rover.state = state;
        rover.taskCounter = 0;
        rover.currentTaskId =
          state === RoverNode.STATE_EXECUTING
            ? `PRESET-${roverId.toUpperCase()}`
            : null;

        if (state === RoverNode.STATE_SAFE_MODE || state === RoverNode.STATE_ERROR) {
          if (!rover.lastFault) rover.lastFault = `Preset ${state}`;
        } else {
          rover.lastFault = null;
        }
      },
    );
  }

  getInitialPositionForRover(index) {
    const base = { lat: -43.3, lon: -11.2 };
    const offsets = [
      { lat: 0.0, lon: 0.0 },
      { lat: 0.08, lon: 0.06 },
      { lat: -0.07, lon: -0.05 },
    ];
    const offset = offsets[index] || { lat: index * 0.03, lon: index * 0.03 };
    return { lat: base.lat + offset.lat, lon: base.lon + offset.lon };
  }

  sendCommand(type, taskId, roverId = null) {
    return this.earth.sendCommand(type, taskId, roverId);
  }

  setSelectedRover(roverId) {
    return this.earth.setSelectedRover(roverId);
  }

  getSelectedRover() {
    return this.earth.getSelectedRover();
  }

  getFleetState() {
    return this.earth.getFleetState();
  }

  updateConfig(key, value) {
    this.config[key] = value;
  }

  getElapsedTime() {
    return (Date.now() - this.startTime) / 1000;
  }

  getRoverState(roverId = null) {
    const targetRoverId = roverId || this.earth.getSelectedRover();
    const rover = this.rovers.find((item) => item.roverId === targetRoverId);
    if (!rover) return null;

    return {
      roverId: rover.roverId,
      state: rover.state,
      battery: rover.batteryLevel,
      taskId: rover.currentTaskId,
      fault: rover.lastFault,
      taskProgress: rover.taskCounter,
    };
  }

  destroy() {
    this.rovers.forEach((rover) => rover.destroy());
    this.earth.destroy();
  }
}

// Export for use by app.js
window.SimulationController = SimulationController;
