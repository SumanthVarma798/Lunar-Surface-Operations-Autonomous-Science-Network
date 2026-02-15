/* ═══════════════════════════════════════════════════════
   LSOAS — Simulation Engine
   Ports all ROS node logic to in-browser JavaScript.
   ═══════════════════════════════════════════════════════ */

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

  constructor(bus, config) {
    this.bus = bus;
    this.config = config;
    this.state = RoverNode.STATE_IDLE;
    this.currentTaskId = null;
    this.taskCounter = 0;
    this.batteryLevel = 1.0;
    this.lastFault = null;
    // Initial position (Near crater Tycho)
    this.position = { lat: -43.3, lon: -11.2 };

    // Listen for commands arriving at rover
    this.bus.on("rover:command", (data) => this.commandCallback(data));

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
      text: `Rover received [${cmdId}]: ${cmdType}`,
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
    this.bus.emit("log", { tag: "task", text: `✅ Started task: ${taskId}` });
    return [true, null];
  }

  handleAbort() {
    if (this.state === RoverNode.STATE_EXECUTING) {
      this.bus.emit("log", {
        tag: "system",
        text: `⏹ Aborting task: ${this.currentTaskId}`,
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
      text: `🛡 Rover entering SAFE_MODE`,
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
        text: `↻ RESET: SAFE_MODE → IDLE`,
      });
    }
    return [true, null];
  }

  sendAck(cmdId, success, reason = null) {
    const ackData = {
      ack_id: cmdId,
      status: success ? "ACCEPTED" : "REJECTED",
      reason,
      ts: Date.now() / 1000,
    };
    // Emit to space link downlink
    this.bus.emit("rover:ack", ackData);
  }

  executeTaskStep() {
    if (this.state !== RoverNode.STATE_EXECUTING) return;

    this.taskCounter++;
    this.batteryLevel = Math.max(0.0, this.batteryLevel - 0.005);

    // Simulate movement
    this.position.lat += (Math.random() - 0.5) * 0.01;
    this.position.lon += (Math.random() - 0.5) * 0.01;

    // Fault detection
    const faultProb = (this.config.faultProbability || 10) / 100;
    if (Math.random() < faultProb) {
      const faultMsg = `Fault during task execution (step ${this.taskCounter})`;
      this.bus.emit("log", {
        tag: "fault",
        text: `🚨 FAULT DETECTED during task ${this.currentTaskId}!`,
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
        text: `✅ Task ${this.currentTaskId} completed!`,
      });
      this.state = RoverNode.STATE_IDLE;
      this.currentTaskId = null;
      this.taskCounter = 0;
    } else {
      this.bus.emit("log", {
        tag: "task",
        text: `⚙️ Executing ${this.currentTaskId}: step ${this.taskCounter}/10`,
      });
    }
  }

  publishTelemetry() {
    const telemetry = {
      ts: Date.now() / 1000,
      state: this.state,
      task_id: this.currentTaskId,
      battery: Math.round(this.batteryLevel * 100) / 100,
      fault: this.lastFault,
      task_progress:
        this.state === RoverNode.STATE_EXECUTING ? this.taskCounter : null,
      position: this.position,
    };
    this.bus.emit("rover:telemetry", telemetry);
  }

  destroy() {
    clearInterval(this.telemetryInterval);
    clearInterval(this.taskInterval);
  }
}

// ─── Space Link Node ───
class SpaceLinkNode {
  constructor(bus, config) {
    this.bus = bus;
    this.config = config;
    this.stats = { sent: 0, received: 0, dropped: 0 };

    // Uplink: earth:uplink_cmd → (delay) → rover:command
    this.bus.on("earth:uplink_cmd", (data) =>
      this.relay(data, "rover:command", "UPLINK"),
    );

    // Downlink telemetry: rover:telemetry → (delay) → earth:telemetry
    this.bus.on("rover:telemetry", (data) =>
      this.relay(data, "earth:telemetry", "DOWNLINK-TLM"),
    );

    // Downlink ACKs: rover:ack → (delay) → earth:ack
    this.bus.on("rover:ack", (data) =>
      this.relay(data, "earth:ack", "DOWNLINK-ACK"),
    );
  }

  relay(data, targetEvent, direction) {
    this.stats.sent++;

    const dropRate = (this.config.dropRate || 5) / 100;
    if (Math.random() < dropRate) {
      this.stats.dropped++;
      this.bus.emit("log", { tag: "drop", text: `❌ ${direction} DROPPED` });
      this.bus.emit("stats:update", this.stats);
      return;
    }

    const baseLatency = this.config.baseLatency || 1.3;
    const jitter = this.config.jitter || 0.2;
    const jitterValue = (Math.random() * 2 - 1) * jitter;
    const delay = Math.max(0.05, baseLatency + jitterValue);

    this.bus.emit("log", {
      tag: "relay",
      text: `📡 ${direction} relay (${delay.toFixed(2)}s delay)`,
    });
    this.bus.emit("signal:active", { direction, delay });

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
    this.cmdCounter = 0;
    this.pendingCommands = {};
    this.ackTimeout = 5.0;
    this.maxRetries = 3;

    // Listen for ACKs arriving at Earth
    this.bus.on("earth:ack", (data) => this.ackCallback(data));

    // Check for timeouts every 1s
    this.timeoutInterval = setInterval(() => this.checkTimeouts(), 1000);
  }

  ackCallback(data) {
    const ackId = data.ack_id;
    const status = data.status;
    const reason = data.reason;

    if (this.pendingCommands[ackId]) {
      const cmdInfo = this.pendingCommands[ackId];
      const rtt = Date.now() / 1000 - cmdInfo.sentAt;

      if (status === "ACCEPTED") {
        this.bus.emit("log", {
          tag: "ack",
          text: `✅ ACK ${ackId}: ACCEPTED (RTT: ${rtt.toFixed(2)}s)`,
        });
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `❌ ACK ${ackId}: REJECTED${reason ? ` (${reason})` : ""}`,
        });
      }

      this.bus.emit("ack:resolved", { cmdId: ackId, status, reason, rtt });
      delete this.pendingCommands[ackId];
    } else {
      if (status === "ACCEPTED") {
        this.bus.emit("log", {
          tag: "ack",
          text: `✅ ACK ${ackId}: ACCEPTED (late)`,
        });
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `❌ ACK ${ackId}: REJECTED${reason ? ` (${reason})` : ""}`,
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
          text: `⏰ No ACK for ${cmdId}, retrying (attempt ${info.attempt}/${this.maxRetries})`,
        });
        this.bus.emit("earth:uplink_cmd", info.cmdData);
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `❌ Command ${cmdId} failed after ${this.maxRetries} attempts`,
        });
        this.bus.emit("ack:resolved", {
          cmdId,
          status: "TIMEOUT",
          reason: "Max retries exceeded",
        });
        delete this.pendingCommands[cmdId];
      }
    }

    this.bus.emit("pending:update", this.pendingCommands);
  }

  sendCommand(cmdType, taskId = null) {
    this.cmdCounter++;
    const cmdId = `c-${String(this.cmdCounter).padStart(5, "0")}`;

    const cmdData = {
      cmd_id: cmdId,
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
    };

    // Send to space link
    this.bus.emit("earth:uplink_cmd", cmdData);
    this.bus.emit("log", {
      tag: "cmd",
      text: `📤 Sent ${cmdId}: ${cmdType}${taskId ? " " + taskId : ""}`,
    });
    this.bus.emit("cmd:sent", { cmdId, cmdType, taskId });
    this.bus.emit("pending:update", this.pendingCommands);

    return cmdId;
  }

  destroy() {
    clearInterval(this.timeoutInterval);
  }
}

// ─── Telemetry Monitor ───
class TelemetryMonitor {
  constructor(bus) {
    this.bus = bus;
    this.latestTelemetry = null;

    this.bus.on("earth:telemetry", (data) => {
      this.latestTelemetry = data;
      this.bus.emit("telemetry:display", data);
    });

    this.bus.on("earth:ack", (data) => {
      this.bus.emit("ack:display", data);
    });
  }
}

// ─── Simulation Controller ───
class SimulationController {
  constructor() {
    this.bus = new EventBus();
    this.config = {
      baseLatency: 1.3,
      jitter: 0.2,
      dropRate: 5,
      faultProbability: 10,
    };

    this.startTime = Date.now();
    this.rover = new RoverNode(this.bus, this.config);
    this.spaceLink = new SpaceLinkNode(this.bus, this.config);
    this.earth = new EarthNode(this.bus, this.config);
    this.telemetryMonitor = new TelemetryMonitor(this.bus);
  }

  sendCommand(type, taskId) {
    return this.earth.sendCommand(type, taskId);
  }

  updateConfig(key, value) {
    this.config[key] = value;
  }

  getElapsedTime() {
    return (Date.now() - this.startTime) / 1000;
  }

  getRoverState() {
    return {
      state: this.rover.state,
      battery: this.rover.batteryLevel,
      taskId: this.rover.currentTaskId,
      fault: this.rover.lastFault,
      taskProgress: this.rover.taskCounter,
    };
  }

  destroy() {
    this.rover.destroy();
    this.earth.destroy();
  }
}

// Export for use by app.js
window.SimulationController = SimulationController;
