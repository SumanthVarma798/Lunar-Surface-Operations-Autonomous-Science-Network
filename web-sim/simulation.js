/* ═══════════════════════════════════════════════════════
   LSOAS — Simulation Engine
   Ports all ROS node logic to in-browser JavaScript.
   ═══════════════════════════════════════════════════════ */

window.LSOASTime = new (class {
  constructor() {
    this.multiplier = 1.0;
    this.baseReal = Date.now();
    this.baseSim = this.baseReal;
    this.timers = new Map();
    this.timerId = 1;

    // Start tick loop
    this.tick = this.tick.bind(this);
    if (typeof requestAnimationFrame !== "undefined") {
      requestAnimationFrame(this.tick);
    } else {
      setInterval(this.tick, 16);
    }
  }

  now() {
    return this.baseSim + (Date.now() - this.baseReal) * this.multiplier;
  }

  setMultiplier(m) {
    this.baseSim = this.now();
    this.baseReal = Date.now();
    this.multiplier = m;
  }

  setTimeout(fn, delayRealMs) {
    const id = this.timerId++;
    const targetSimTime = this.now() + delayRealMs;
    this.timers.set(id, { type: "timeout", target: targetSimTime, fn });
    return id;
  }

  setInterval(fn, intervalRealMs) {
    const id = this.timerId++;
    this.timers.set(id, {
      type: "interval",
      interval: intervalRealMs,
      next: this.now() + intervalRealMs,
      fn,
    });
    return id;
  }

  clearTimeout(id) {
    this.timers.delete(id);
  }
  clearInterval(id) {
    this.timers.delete(id);
  }

  tick() {
    if (typeof requestAnimationFrame !== "undefined") {
      requestAnimationFrame(this.tick);
    }
    const currentSim = this.now();

    for (const [id, timer] of Array.from(this.timers.entries())) {
      if (timer.type === "timeout") {
        if (currentSim >= timer.target) {
          this.timers.delete(id);
          timer.fn();
        }
      } else if (timer.type === "interval") {
        if (currentSim >= timer.next) {
          // Trigger once and advance next target. To handle high multipliers gracefully,
          // ensure next is strictly monotonic without triggering 100 times per tick.
          timer.next = currentSim + timer.interval;
          timer.fn();
        }
      }
    }
  }
})();

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
    faultProbability: parseBoundedNumber(params.get("fault"), 0, -20, 20),
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

const FALLBACK_TASK_CATALOG = {
  schema_version: "2.0.0",
  difficulty_levels: {
    L1: {
      base_fault_rate: 0.01,
      duration_multiplier: 0.7,
      min_battery: 0.12,
      energy_drain_per_step: 0.002,
    },
    L2: {
      base_fault_rate: 0.03,
      duration_multiplier: 0.9,
      min_battery: 0.18,
      energy_drain_per_step: 0.003,
    },
    L3: {
      base_fault_rate: 0.06,
      duration_multiplier: 1.1,
      min_battery: 0.25,
      energy_drain_per_step: 0.004,
    },
    L4: {
      base_fault_rate: 0.1,
      duration_multiplier: 1.35,
      min_battery: 0.32,
      energy_drain_per_step: 0.0055,
    },
    L5: {
      base_fault_rate: 0.18,
      duration_multiplier: 1.7,
      min_battery: 0.42,
      energy_drain_per_step: 0.007,
    },
  },
  task_types: {
    movement: {
      display_name: "Movement/Traverse",
      mission_phase: "CY3-ops",
      required_capabilities: ["mobility"],
      base_steps: 8,
      min_steps: 4,
      max_steps: 30,
      terrain_sensitivity: 1.0,
      solar_sensitivity: 0.7,
      comm_sensitivity: 0.5,
      thermal_sensitivity: 0.6,
      battery_sensitivity: 0.9,
      terrain_duration_sensitivity: 0.45,
      risk_floor: 0.005,
    },
    science: {
      display_name: "Science Investigation",
      mission_phase: "CY3-ops",
      required_capabilities: ["science", "imaging"],
      base_steps: 10,
      min_steps: 5,
      max_steps: 36,
      terrain_sensitivity: 0.5,
      solar_sensitivity: 0.6,
      comm_sensitivity: 0.7,
      thermal_sensitivity: 0.8,
      battery_sensitivity: 0.7,
      terrain_duration_sensitivity: 0.25,
      risk_floor: 0.008,
    },
    digging: {
      display_name: "Digging/Drilling/Excavation",
      mission_phase: "LUPEX-prospecting",
      required_capabilities: ["excavation"],
      base_steps: 12,
      min_steps: 6,
      max_steps: 42,
      terrain_sensitivity: 0.8,
      solar_sensitivity: 0.5,
      comm_sensitivity: 0.4,
      thermal_sensitivity: 0.9,
      battery_sensitivity: 1.0,
      terrain_duration_sensitivity: 0.35,
      risk_floor: 0.01,
    },
    pushing: {
      display_name: "Pushing/Transport/Regolith Handling",
      mission_phase: "base-build",
      required_capabilities: ["manipulation", "mobility"],
      base_steps: 11,
      min_steps: 6,
      max_steps: 40,
      terrain_sensitivity: 0.9,
      solar_sensitivity: 0.4,
      comm_sensitivity: 0.4,
      thermal_sensitivity: 0.7,
      battery_sensitivity: 1.0,
      terrain_duration_sensitivity: 0.4,
      risk_floor: 0.01,
    },
    photo: {
      display_name: "Photo/Imaging/Survey",
      mission_phase: "CY3-ops",
      required_capabilities: ["imaging"],
      base_steps: 6,
      min_steps: 3,
      max_steps: 24,
      terrain_sensitivity: 0.3,
      solar_sensitivity: 0.5,
      comm_sensitivity: 0.8,
      thermal_sensitivity: 0.4,
      battery_sensitivity: 0.4,
      terrain_duration_sensitivity: 0.15,
      risk_floor: 0.004,
    },
    "sample-handling": {
      display_name: "Sample Handling/Transfer",
      mission_phase: "CY4-sample-chain",
      required_capabilities: ["sample-logistics", "manipulation"],
      base_steps: 9,
      min_steps: 5,
      max_steps: 32,
      terrain_sensitivity: 0.6,
      solar_sensitivity: 0.5,
      comm_sensitivity: 0.6,
      thermal_sensitivity: 0.8,
      battery_sensitivity: 0.8,
      terrain_duration_sensitivity: 0.3,
      risk_floor: 0.009,
    },
  },
};

function clamp01(value) {
  return Math.max(
    0,
    Math.min(1, Number.isFinite(Number(value)) ? Number(value) : 0),
  );
}

const TAU = Math.PI * 2;

function degToRad(value) {
  return (Number(value) * Math.PI) / 180;
}

function vectorLength(vec) {
  const x = Number(vec?.x) || 0;
  const y = Number(vec?.y) || 0;
  const z = Number(vec?.z) || 0;
  return Math.sqrt(x * x + y * y + z * z);
}

function normalizeVec(vec) {
  const length = vectorLength(vec);
  if (length <= 1e-9) return { x: 0, y: 0, z: 1 };
  return {
    x: vec.x / length,
    y: vec.y / length,
    z: vec.z / length,
  };
}

function dotVec(a, b) {
  return (
    (Number(a?.x) || 0) * (Number(b?.x) || 0) +
    (Number(a?.y) || 0) * (Number(b?.y) || 0) +
    (Number(a?.z) || 0) * (Number(b?.z) || 0)
  );
}

function latLonToLunarUnit(lat, lon) {
  const phi = degToRad(90 - Number(lat || 0));
  const theta = degToRad(Number(lon || 0) + 180);
  const sinPhi = Math.sin(phi);
  return {
    x: sinPhi * Math.cos(theta),
    y: Math.cos(phi),
    z: sinPhi * Math.sin(theta),
  };
}

class CelestialDynamics {
  constructor() {
    this.earthMoonDistanceKm = 384400;
    this.sunEarthDistanceKm = 149597870;
    this.earthOrbitPeriodDays = 365.256;
    this.moonOrbitPeriodDays = 27.321661;
    this.moonOrbitInclinationRad = degToRad(5.145);
    this.simDaysPerSecond = 1.2;
    this.phaseOffsetRad = -1.05;
    this.startEpochMs = LSOASTime.now();
  }

  getElapsedDays(nowMs = LSOASTime.now()) {
    return ((nowMs - this.startEpochMs) / 1000) * this.simDaysPerSecond;
  }

  computeFrame(nowMs = LSOASTime.now()) {
    const elapsedDays = this.getElapsedDays(nowMs);
    const earthOrbitAngle = TAU * (elapsedDays / this.earthOrbitPeriodDays);
    const moonOrbitAngle =
      TAU * (elapsedDays / this.moonOrbitPeriodDays) + this.phaseOffsetRad;

    const earthPosSun = {
      x: this.sunEarthDistanceKm * Math.cos(earthOrbitAngle),
      y: this.sunEarthDistanceKm * Math.sin(earthOrbitAngle),
      z: 0,
    };
    const moonGeo = {
      x: this.earthMoonDistanceKm * Math.cos(moonOrbitAngle),
      y:
        this.earthMoonDistanceKm *
        Math.sin(moonOrbitAngle) *
        Math.cos(this.moonOrbitInclinationRad),
      z:
        this.earthMoonDistanceKm *
        Math.sin(moonOrbitAngle) *
        Math.sin(this.moonOrbitInclinationRad),
    };
    const moonPosSun = {
      x: earthPosSun.x + moonGeo.x,
      y: earthPosSun.y + moonGeo.y,
      z: earthPosSun.z + moonGeo.z,
    };

    const earthFromMoon = {
      x: -moonGeo.x,
      y: -moonGeo.y,
      z: -moonGeo.z,
    };
    const sunFromMoon = {
      x: -moonPosSun.x,
      y: -moonPosSun.y,
      z: -moonPosSun.z,
    };

    const earthDistanceKm = vectorLength(earthFromMoon);
    const sunDistanceKm = vectorLength(sunFromMoon);

    return {
      ts: nowMs / 1000,
      sim_elapsed_days: Number(elapsedDays.toFixed(5)),
      earth_distance_km: Number(earthDistanceKm.toFixed(0)),
      sun_distance_km: Number(sunDistanceKm.toFixed(0)),
      distance_ratio_sun_to_earth: Number(
        (sunDistanceKm / Math.max(earthDistanceKm, 1)).toFixed(2),
      ),
      earth_from_moon_km: {
        x: Number(earthFromMoon.x.toFixed(3)),
        y: Number(earthFromMoon.y.toFixed(3)),
        z: Number(earthFromMoon.z.toFixed(3)),
      },
      sun_from_moon_km: {
        x: Number(sunFromMoon.x.toFixed(3)),
        y: Number(sunFromMoon.y.toFixed(3)),
        z: Number(sunFromMoon.z.toFixed(3)),
      },
      earth_dir_from_moon: normalizeVec(earthFromMoon),
      sun_dir_from_moon: normalizeVec(sunFromMoon),
    };
  }

  computeSolarExposureForLatLon(lat, lon, frame) {
    if (!frame?.sun_dir_from_moon) return 0.5;
    const normal = latLonToLunarUnit(lat, lon);
    const incidence = dotVec(normal, frame.sun_dir_from_moon);
    if (incidence <= 0) return 0;

    const horizonWeighted = Math.pow(incidence, 0.82);
    const distanceScale =
      this.sunEarthDistanceKm /
      Math.max(frame.sun_distance_km || this.sunEarthDistanceKm, 1);
    const inverseSquare = Math.pow(distanceScale, 2);
    return clamp01(horizonWeighted * inverseSquare);
  }
}

function getLunarTimeState(solarIntensity) {
  const solar = clamp01(solarIntensity);
  if (solar >= 0.55) return "DAYLIGHT";
  if (solar <= 0.3) return "NIGHT";
  return "TERMINATOR";
}

function getRoverCapabilities(roverId) {
  const match = String(roverId || "").match(/(\d+)$/);
  const roverIndex = match ? Number(match[1]) : 1;
  const mod = roverIndex % 3;
  if (mod === 1) return ["mobility", "imaging", "science"];
  if (mod === 2) return ["mobility", "science", "sample-logistics", "imaging"];
  return [
    "mobility",
    "excavation",
    "manipulation",
    "sample-logistics",
    "imaging",
  ];
}

function normalizeTaskRequest(taskCatalog, taskId, taskOptions = {}) {
  const taskType = String(taskOptions.task_type || "movement")
    .trim()
    .toLowerCase();
  const difficulty = String(taskOptions.difficulty_level || "L2")
    .trim()
    .toUpperCase();
  const safeTaskType = taskCatalog.task_types[taskType] ? taskType : "movement";
  const safeDifficulty = taskCatalog.difficulty_levels[difficulty]
    ? difficulty
    : "L2";
  const taskConfig = taskCatalog.task_types[safeTaskType];
  const requestedCapabilities = Array.isArray(taskOptions.required_capabilities)
    ? taskOptions.required_capabilities
        .map((item) =>
          String(item || "")
            .trim()
            .toLowerCase(),
        )
        .filter(Boolean)
    : [...(taskConfig.required_capabilities || [])];

  return {
    task_id: String(taskId || `${safeTaskType}-${safeDifficulty}`),
    task_type: safeTaskType,
    difficulty_level: safeDifficulty,
    mission_phase:
      taskOptions.mission_phase || taskConfig.mission_phase || "CY3-ops",
    required_capabilities: requestedCapabilities,
    target_site: taskOptions.target_site || null,
  };
}

function computeTaskDurationSteps(taskCatalog, taskRequest, context) {
  const taskCfg = taskCatalog.task_types[taskRequest.task_type];
  const diffCfg = taskCatalog.difficulty_levels[taskRequest.difficulty_level];
  const terrain = clamp01(context.terrain_difficulty ?? 0.3);
  const commQuality = clamp01(context.comm_quality ?? 0.8);
  const terrainFactor =
    1 + terrain * Number(taskCfg.terrain_duration_sensitivity ?? 0.3);
  const commFactor = 1 + (1 - commQuality) * 0.2;
  const raw = Math.round(
    Number(taskCfg.base_steps || 8) *
      Number(diffCfg.duration_multiplier || 1) *
      terrainFactor *
      commFactor,
  );
  return Math.max(
    Number(taskCfg.min_steps || 3),
    Math.min(Number(taskCfg.max_steps || 60), raw),
  );
}

function computeFaultProbability(taskCatalog, taskRequest, context) {
  const taskCfg = taskCatalog.task_types[taskRequest.task_type];
  const diffCfg = taskCatalog.difficulty_levels[taskRequest.difficulty_level];
  const baseRisk = Number(diffCfg.base_fault_rate ?? 0.03);

  const battery = clamp01(context.battery ?? 1);
  const solar = clamp01(
    context.solar_intensity ?? context.solar_exposure ?? 0.5,
  );
  const terrain = clamp01(context.terrain_difficulty ?? 0.3);
  const commQuality = clamp01(context.comm_quality ?? 0.8);
  const thermalStress = clamp01(context.thermal_stress ?? 0.3);
  const lunarState = String(
    context.lunar_time_state || getLunarTimeState(solar),
  ).toUpperCase();
  const capabilityMatch = Boolean(context.capability_match ?? true);

  const batteryPenalty =
    Math.max(0, 0.45 - battery) *
    Number(taskCfg.battery_sensitivity ?? 0.8) *
    0.22;
  const solarPenalty =
    Math.max(0, 0.4 - solar) * Number(taskCfg.solar_sensitivity ?? 0.6) * 0.18;
  const terrainPenalty =
    terrain * Number(taskCfg.terrain_sensitivity ?? 0.6) * 0.12;
  const commPenalty =
    (1 - commQuality) * Number(taskCfg.comm_sensitivity ?? 0.5) * 0.1;
  const thermalPenalty =
    thermalStress * Number(taskCfg.thermal_sensitivity ?? 0.6) * 0.1;
  const lunarPenalty =
    lunarState === "NIGHT" ? 0.06 : lunarState === "TERMINATOR" ? 0.02 : 0;
  const capabilityPenalty = capabilityMatch ? 0 : 0.12;
  const batteryBonus = Math.max(0, battery - 0.8) * 0.03;
  const solarBonus = Math.max(0, solar - 0.85) * 0.02;

  let risk =
    baseRisk +
    batteryPenalty +
    solarPenalty +
    terrainPenalty +
    commPenalty +
    thermalPenalty +
    lunarPenalty +
    capabilityPenalty -
    batteryBonus -
    solarBonus;
  risk = Math.max(Number(taskCfg.risk_floor ?? 0), risk);
  risk = Math.max(0, Math.min(0.6, risk));

  return {
    predicted_fault_probability: Number(risk.toFixed(4)),
    lunar_time_state: lunarState,
    breakdown: {
      base_risk: Number(baseRisk.toFixed(4)),
      battery_penalty: Number(batteryPenalty.toFixed(4)),
      solar_penalty: Number(solarPenalty.toFixed(4)),
      terrain_penalty: Number(terrainPenalty.toFixed(4)),
      comm_penalty: Number(commPenalty.toFixed(4)),
      thermal_penalty: Number(thermalPenalty.toFixed(4)),
      lunar_penalty: Number(lunarPenalty.toFixed(4)),
      capability_penalty: Number(capabilityPenalty.toFixed(4)),
      battery_bonus: Number(batteryBonus.toFixed(4)),
      solar_bonus: Number(solarBonus.toFixed(4)),
      lunar_time_state: lunarState,
    },
  };
}

function scoreRoverForTask(taskCatalog, roverId, roverStatus, taskRequest) {
  const state = String(roverStatus?.state || "UNKNOWN").toUpperCase();
  const battery = clamp01(roverStatus?.battery ?? 0);
  const solar = clamp01(
    roverStatus?.solar_exposure ?? roverStatus?.solar_intensity ?? 0.5,
  );
  const terrainDifficulty = clamp01(roverStatus?.terrain_difficulty ?? 0.3);
  const commQuality = clamp01(roverStatus?.comm_quality ?? 0.8);
  const thermalStress = clamp01(roverStatus?.thermal_stress ?? 0.3);
  const capabilities = getRoverCapabilities(roverId);
  const capabilityMatch = (taskRequest.required_capabilities || []).every(
    (cap) => capabilities.includes(cap),
  );

  const risk = computeFaultProbability(taskCatalog, taskRequest, {
    battery,
    solar_intensity: solar,
    terrain_difficulty: terrainDifficulty,
    comm_quality: commQuality,
    thermal_stress: thermalStress,
    capability_match: capabilityMatch,
  });

  const distanceMargin = 0.75;
  const accessibilityMargin = 1 - terrainDifficulty;
  const scoreBreakdown = {
    capability_match: capabilityMatch ? 1 : 0,
    battery_margin: battery,
    solar_margin: solar,
    thermal_margin: 1 - thermalStress,
    comm_margin: commQuality,
    distance_margin: distanceMargin,
    accessibility_margin: accessibilityMargin,
    risk_margin: 1 - risk.predicted_fault_probability / 0.6,
  };

  const score =
    scoreBreakdown.capability_match * 0.28 +
    scoreBreakdown.battery_margin * 0.2 +
    scoreBreakdown.solar_margin * 0.12 +
    scoreBreakdown.thermal_margin * 0.1 +
    scoreBreakdown.comm_margin * 0.1 +
    ((scoreBreakdown.distance_margin + scoreBreakdown.accessibility_margin) /
      2) *
      0.1 +
    scoreBreakdown.risk_margin * 0.1;

  const minBattery = Number(
    taskCatalog.difficulty_levels[taskRequest.difficulty_level]?.min_battery ??
      0,
  );
  const rejectReasons = [];
  if (state !== "IDLE") rejectReasons.push(`state=${state}`);
  if (battery < minBattery)
    rejectReasons.push(`battery<${minBattery.toFixed(2)}`);
  if (!capabilityMatch) rejectReasons.push("capability_mismatch");
  if (risk.predicted_fault_probability > 0.45)
    rejectReasons.push("predicted_risk_too_high");

  return {
    rover_id: roverId,
    score: Number(score.toFixed(4)),
    score_breakdown: Object.fromEntries(
      Object.entries(scoreBreakdown).map(([key, value]) => [
        key,
        Number(value.toFixed(4)),
      ]),
    ),
    capabilities,
    predicted_fault_probability: risk.predicted_fault_probability,
    lunar_time_state: risk.lunar_time_state,
    risk_breakdown: risk.breakdown,
    feasible: rejectReasons.length === 0,
    reject_reasons: rejectReasons,
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
    this.activeTaskType = null;
    this.activeTaskDifficulty = null;
    this.activeTaskMissionPhase = null;
    this.activeTaskRequiredCapabilities = [];
    this.assignmentScoreBreakdown = null;
    this.taskTotalSteps = 0;
    this.taskCounter = 0;
    this.batteryLevel = 1.0;
    this.lastFault = null;
    this.predictedFaultProbability = 0;
    this.commandTopic = roverTopic(this.roverId, "command");
    this.telemetryTopic = roverTopic(this.roverId, "telemetry");
    this.ackTopic = roverTopic(this.roverId, "ack");
    this.capabilities = getRoverCapabilities(this.roverId);
    this.celestialDynamics = this.config.celestialDynamics || null;
    this.latestCelestialFrame = this.config.latestCelestialFrame || null;

    // Initial position (Near crater Tycho, offset by rover)
    this.position = startPosition || { lat: -43.3, lon: -11.2 };
    this.solarExposure = this.celestialDynamics
      ? this.celestialDynamics.computeSolarExposureForLatLon(
          this.position.lat,
          this.position.lon,
          this.latestCelestialFrame ||
            this.celestialDynamics.computeFrame(LSOASTime.now()),
        )
      : clamp01(0.6 + Math.random() * 0.4);
    this.lunarTimeState = getLunarTimeState(this.solarExposure);
    this.terrainDifficulty = clamp01(0.2 + Math.random() * 0.45);
    this.commQuality = clamp01(0.72 + Math.random() * 0.22);
    this.thermalStress = clamp01(0.15 + Math.random() * 0.3);
    this._solarPhase = Math.random() * Math.PI * 2;

    this.commandListener = (data) => this.commandCallback(data);
    this.bus.on(this.commandTopic, this.commandListener);

    // Start telemetry publishing (every 2s)
    this.telemetryInterval = LSOASTime.setInterval(
      () => this.publishTelemetry(),
      2000,
    );
    // Start task execution tick (every 1s)
    this.taskInterval = LSOASTime.setInterval(
      () => this.executeTaskStep(),
      1000,
    );
  }

  commandCallback(cmdData) {
    const cmdId = cmdData.cmd_id || "unknown";
    const cmdType = cmdData.type;
    this.bus.emit("log", {
      tag: "cmd",
      text: `[${this.roverId}] received [${cmdId}]: ${cmdType}`,
    });

    let success, reason;

    switch (cmdType) {
      case "START_TASK":
        [success, reason] = this.handleStartTask(cmdData);
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

  _clearActiveTask() {
    this.currentTaskId = null;
    this.activeTaskType = null;
    this.activeTaskDifficulty = null;
    this.activeTaskMissionPhase = null;
    this.activeTaskRequiredCapabilities = [];
    this.assignmentScoreBreakdown = null;
    this.taskTotalSteps = 0;
    this.taskCounter = 0;
    this.predictedFaultProbability = 0;
  }

  _updateSolarExposure(nowMs = LSOASTime.now()) {
    if (this.celestialDynamics) {
      const frame =
        this.config.latestCelestialFrame ||
        this.celestialDynamics.computeFrame(nowMs);
      this.latestCelestialFrame = frame;
      const baseExposure = this.celestialDynamics.computeSolarExposureForLatLon(
        this.position.lat,
        this.position.lon,
        frame,
      );
      const microVariation =
        this.state === RoverNode.STATE_EXECUTING
          ? Math.random() * 0.012 - 0.006
          : Math.random() * 0.008 - 0.004;
      this.solarExposure = clamp01(baseExposure + microVariation);
      this.lunarTimeState = getLunarTimeState(this.solarExposure);
      return;
    }

    this._solarPhase += 0.02;
    const base = 0.5 + 0.5 * Math.sin(this._solarPhase);
    const noise = Math.random() * 0.1 - 0.05;
    this.solarExposure = clamp01(base + noise);
    this.lunarTimeState = getLunarTimeState(this.solarExposure);
  }

  _updateEnvironmentals() {
    this.commQuality = clamp01(
      this.commQuality + (Math.random() * 0.05 - 0.03),
    );
    this.terrainDifficulty = clamp01(
      this.terrainDifficulty + (Math.random() * 0.04 - 0.02),
    );
    const thermalDelta = this.lunarTimeState === "DAYLIGHT" ? 0.04 : -0.02;
    this.thermalStress = clamp01(
      this.thermalStress + thermalDelta + (Math.random() * 0.06 - 0.03),
    );
  }

  handleStartTask(cmdData) {
    const taskRequest = normalizeTaskRequest(
      this.config.taskCatalog || FALLBACK_TASK_CATALOG,
      cmdData.task_id,
      cmdData,
    );

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

    const missingCapabilities = (
      taskRequest.required_capabilities || []
    ).filter((cap) => !this.capabilities.includes(cap));
    if (missingCapabilities.length > 0) {
      return [
        false,
        `Cannot start task: Missing capabilities for ${taskRequest.task_type}`,
      ];
    }

    this.taskTotalSteps = computeTaskDurationSteps(
      this.config.taskCatalog || FALLBACK_TASK_CATALOG,
      taskRequest,
      {
        terrain_difficulty: this.terrainDifficulty,
        comm_quality: this.commQuality,
      },
    );
    const risk = computeFaultProbability(
      this.config.taskCatalog || FALLBACK_TASK_CATALOG,
      taskRequest,
      {
        battery: this.batteryLevel,
        solar_intensity: this.solarExposure,
        terrain_difficulty: this.terrainDifficulty,
        comm_quality: this.commQuality,
        thermal_stress: this.thermalStress,
        lunar_time_state: this.lunarTimeState,
        capability_match: true,
      },
    );

    this.state = RoverNode.STATE_EXECUTING;
    this.currentTaskId = taskRequest.task_id;
    this.activeTaskType = taskRequest.task_type;
    this.activeTaskDifficulty = taskRequest.difficulty_level;
    this.activeTaskMissionPhase = taskRequest.mission_phase;
    this.activeTaskRequiredCapabilities = [
      ...taskRequest.required_capabilities,
    ];
    this.assignmentScoreBreakdown = cmdData.assignment_score_breakdown || null;
    this.taskCounter = 0;
    this.predictedFaultProbability = Number.isFinite(
      Number(cmdData.predicted_fault_probability),
    )
      ? Number(cmdData.predicted_fault_probability)
      : risk.predicted_fault_probability;
    this.bus.emit("log", {
      tag: "task",
      text: `[${this.roverId}] Started ${this.currentTaskId} (${this.activeTaskType}/${this.activeTaskDifficulty}, steps=${this.taskTotalSteps}, risk=${this.predictedFaultProbability.toFixed(3)})`,
    });
    return [true, null];
  }

  handleAbort() {
    if (this.state === RoverNode.STATE_EXECUTING) {
      this.bus.emit("log", {
        tag: "system",
        text: `[${this.roverId}] Aborting task: ${this.currentTaskId}`,
      });
      this.state = RoverNode.STATE_IDLE;
      this._clearActiveTask();
    }
    return [true, null];
  }

  handleGoSafe() {
    this.state = RoverNode.STATE_SAFE_MODE;
    this._clearActiveTask();
    if (!this.lastFault) this.lastFault = "Commanded to SAFE_MODE";
    this.bus.emit("log", {
      tag: "system",
      text: `[${this.roverId}] Entering SAFE_MODE`,
    });
    return [true, null];
  }

  handleReset() {
    if (this.state === RoverNode.STATE_SAFE_MODE) {
      this.state = RoverNode.STATE_IDLE;
      this._clearActiveTask();
      this.lastFault = null;
      this.bus.emit("log", {
        tag: "system",
        text: `[${this.roverId}] RESET: SAFE_MODE -> IDLE`,
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
      ts: LSOASTime.now() / 1000,
    };
    this.bus.emit(this.ackTopic, ackData);
  }

  executeTaskStep() {
    if (this.state !== RoverNode.STATE_EXECUTING) return;

    this._updateSolarExposure();
    this._updateEnvironmentals();
    this.taskCounter++;
    const diffCfg =
      (this.config.taskCatalog || FALLBACK_TASK_CATALOG).difficulty_levels[
        this.activeTaskDifficulty || "L2"
      ] || {};
    const baseDrain = Number(diffCfg.energy_drain_per_step ?? 0.003);
    this.batteryLevel = Math.max(
      0.0,
      this.batteryLevel - baseDrain * (1 + this.terrainDifficulty * 0.4),
    );

    // Simulate movement
    this.position.lat += (Math.random() - 0.5) * 0.01;
    this.position.lon += (Math.random() - 0.5) * 0.01;

    const taskRequest = {
      task_type: this.activeTaskType || "movement",
      difficulty_level: this.activeTaskDifficulty || "L2",
    };
    const risk = computeFaultProbability(
      this.config.taskCatalog || FALLBACK_TASK_CATALOG,
      taskRequest,
      {
        battery: this.batteryLevel,
        solar_intensity: this.solarExposure,
        terrain_difficulty: this.terrainDifficulty,
        comm_quality: this.commQuality,
        thermal_stress: this.thermalStress,
        lunar_time_state: this.lunarTimeState,
        capability_match: true,
      },
    );
    const globalRiskBias = ((this.config.faultProbability ?? 0) / 100) * 0.1;
    const faultProb = Math.max(
      0,
      Math.min(0.6, risk.predicted_fault_probability + globalRiskBias),
    );
    this.predictedFaultProbability = Number(faultProb.toFixed(4));
    if (Math.random() < faultProb) {
      const faultMsg = `Fault during task execution (step ${this.taskCounter})`;
      this.bus.emit("log", {
        tag: "fault",
        text: `🚨 [${this.roverId}] FAULT DETECTED during task ${this.currentTaskId}!`,
      });
      this.state = RoverNode.STATE_SAFE_MODE;
      this.lastFault = faultMsg;
      this._clearActiveTask();
      return;
    }

    // Task completion is catalog-driven by type + difficulty.
    if (this.taskCounter >= Math.max(1, this.taskTotalSteps)) {
      this.bus.emit("log", {
        tag: "task",
        text: `[${this.roverId}] Task ${this.currentTaskId} completed`,
      });
      this.state = RoverNode.STATE_IDLE;
      this._clearActiveTask();
    } else {
      this.bus.emit("log", {
        tag: "task",
        text: `⚙️ [${this.roverId}] Executing ${this.currentTaskId}: step ${this.taskCounter}/${this.taskTotalSteps}`,
      });
    }
  }

  publishTelemetry() {
    this._updateSolarExposure(LSOASTime.now());
    const frame = this.latestCelestialFrame;
    const telemetry = {
      ts: LSOASTime.now() / 1000,
      rover_id: this.roverId,
      state: this.state,
      task_id: this.currentTaskId,
      active_task_type: this.activeTaskType,
      active_task_difficulty: this.activeTaskDifficulty,
      battery: Math.round(this.batteryLevel * 100) / 100,
      fault: this.lastFault,
      task_progress:
        this.state === RoverNode.STATE_EXECUTING ? this.taskCounter : null,
      task_total_steps:
        this.state === RoverNode.STATE_EXECUTING ? this.taskTotalSteps : null,
      predicted_fault_probability: this.predictedFaultProbability,
      assignment_score_breakdown: this.assignmentScoreBreakdown,
      lunar_time_state: this.lunarTimeState,
      solar_intensity: Number(this.solarExposure.toFixed(2)),
      solar_exposure: Number(this.solarExposure.toFixed(2)),
      terrain_difficulty: Number(this.terrainDifficulty.toFixed(2)),
      comm_quality: Number(this.commQuality.toFixed(2)),
      thermal_stress: Number(this.thermalStress.toFixed(2)),
      capabilities: [...this.capabilities],
      position: { lat: this.position.lat, lon: this.position.lon },
      celestial: frame
        ? {
            sim_elapsed_days: frame.sim_elapsed_days,
            earth_distance_km: frame.earth_distance_km,
            sun_distance_km: frame.sun_distance_km,
            distance_ratio_sun_to_earth: frame.distance_ratio_sun_to_earth,
            earth_from_moon_km: frame.earth_from_moon_km,
            sun_from_moon_km: frame.sun_from_moon_km,
            earth_dir_from_moon: frame.earth_dir_from_moon,
            sun_dir_from_moon: frame.sun_dir_from_moon,
          }
        : null,
    };
    this.bus.emit(this.telemetryTopic, telemetry);
  }

  destroy() {
    LSOASTime.clearInterval(this.telemetryInterval);
    LSOASTime.clearInterval(this.taskInterval);
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
    this.commandBuffer = new Map();
    this.losOutageActive = false;
    this.losIssueReason = "";

    // Uplink: earth:uplink_cmd → (delay) → <rover-id>:command
    this.bus.on("earth:uplink_cmd", (data) => this.relayCommand(data));
    this.bus.on("orbital:los-status", (status) => this.handleLosStatus(status));

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
    if (this.losOutageActive) {
      this.bufferCommand({ ...data, rover_id: roverId }, roverId);
      return;
    }
    this.relay(
      { ...data, rover_id: roverId },
      roverTopic(roverId, "command"),
      "UPLINK",
      roverId,
    );
  }

  handleLosStatus(status) {
    const hasIssue = Boolean(status?.hasIssue);
    const issueReason = String(status?.issueReason || "").trim();
    if (
      hasIssue === this.losOutageActive &&
      issueReason === this.losIssueReason
    )
      return;

    const wasOutage = this.losOutageActive;
    this.losOutageActive = hasIssue;
    this.losIssueReason = issueReason;

    if (!wasOutage && hasIssue) {
      this.bus.emit("log", {
        tag: "fault",
        text: `Uplink buffering enabled: ${issueReason || "LOS outage detected"}`,
      });
      return;
    }

    if (wasOutage && !hasIssue) {
      this.bus.emit("log", {
        tag: "system",
        text: "LOS restored. Releasing buffered uplink commands.",
      });
      this.flushBufferedCommands();
    }
  }

  bufferCommand(data, roverId) {
    const cmdId = String(data?.cmd_id || "");
    if (!cmdId) return;

    const existing = this.commandBuffer.get(cmdId);
    if (existing) {
      existing.data = { ...data };
      existing.roverId = roverId;
      this.bus.emit("spacelink:command-buffered", {
        cmdId,
        roverId,
        queueDepth: this.commandBuffer.size,
        issueReason: this.losIssueReason,
      });
      return;
    }

    this.commandBuffer.set(cmdId, {
      data: { ...data },
      roverId,
      bufferedAt: LSOASTime.now() / 1000,
    });
    this.bus.emit("spacelink:command-buffered", {
      cmdId,
      roverId,
      queueDepth: this.commandBuffer.size,
      issueReason: this.losIssueReason,
    });
    this.bus.emit("log", {
      tag: "relay",
      text: `Buffered ${cmdId} [${roverId}] until LOS recovery`,
    });
  }

  flushBufferedCommands() {
    if (this.commandBuffer.size === 0) return;
    const queuedEntries = Array.from(this.commandBuffer.values());
    this.commandBuffer.clear();

    queuedEntries.forEach((entry, index) => {
      LSOASTime.setTimeout(() => {
        if (this.losOutageActive) {
          this.bufferCommand(entry.data, entry.roverId);
          return;
        }

        this.bus.emit("spacelink:command-released", {
          cmdId: entry.data?.cmd_id || null,
          roverId: entry.roverId,
          bufferedFor: Math.max(
            0,
            LSOASTime.now() / 1000 - Number(entry.bufferedAt || 0),
          ),
          queueDepth: Math.max(0, queuedEntries.length - index - 1),
        });
        this.relay(
          { ...entry.data, rover_id: entry.roverId },
          roverTopic(entry.roverId, "command"),
          "UPLINK",
          entry.roverId,
        );
      }, index * 120);
    });
  }

  relay(data, targetEvent, direction, roverId = null) {
    this.stats.sent++;

    const dropRate = (this.config.dropRate ?? 5) / 100;
    if (Math.random() < dropRate) {
      this.stats.dropped++;
      this.bus.emit("log", {
        tag: "drop",
        text: `${direction}${roverId ? ` [${roverId}]` : ""} DROPPED`,
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
      text: `${direction}${roverId ? ` [${roverId}]` : ""} relay (${delay.toFixed(2)}s delay)`,
    });
    this.bus.emit("signal:active", { direction, delay, rover_id: roverId });

    LSOASTime.setTimeout(() => {
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
    this.bus.on("spacelink:command-buffered", (data) =>
      this.handleBufferedCommand(data),
    );
    this.bus.on("spacelink:command-released", (data) =>
      this.handleReleasedCommand(data),
    );

    // Check for timeouts every 1s
    this.timeoutInterval = LSOASTime.setInterval(
      () => this.checkTimeouts(),
      1000,
    );
    this.bus.emit("fleet:update", this.getFleetState());
  }

  buildDefaultFleetEntry(roverId) {
    return {
      rover_id: roverId,
      state: RoverNode.STATE_IDLE,
      battery: 1.0,
      task_id: null,
      active_task_type: null,
      active_task_difficulty: null,
      fault: null,
      task_progress: null,
      position: null,
      solar_exposure: 0.5,
      solar_intensity: 0.5,
      lunar_time_state: "UNKNOWN",
      terrain_difficulty: 0.3,
      comm_quality: 0.8,
      thermal_stress: 0.3,
      predicted_fault_probability: 0,
      assignment_score_breakdown: null,
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
      last_seen: LSOASTime.now() / 1000,
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
      text: `Selected rover set to [${roverId}]`,
    });
    return true;
  }

  selectBestRoverForTask(taskId, taskOptions = {}) {
    const taskRequest = normalizeTaskRequest(
      this.config.taskCatalog || FALLBACK_TASK_CATALOG,
      taskId,
      taskOptions,
    );
    const scored = Object.keys(this.fleetState).map((roverId) =>
      scoreRoverForTask(
        this.config.taskCatalog || FALLBACK_TASK_CATALOG,
        roverId,
        this.fleetState[roverId],
        taskRequest,
      ),
    );
    const feasible = scored.filter((item) => item.feasible);
    if (feasible.length === 0) {
      const sorted = [...scored].sort((a, b) => b.score - a.score);
      return {
        selected_rover: null,
        reject_reason:
          sorted[0]?.reject_reasons?.join(",") ||
          "no_rover_passed_feasibility_threshold",
        task_request: taskRequest,
        scored_candidates: sorted,
      };
    }
    const sorted = [...feasible].sort((a, b) => b.score - a.score);
    return {
      selected_rover: sorted[0].rover_id,
      reject_reason: null,
      task_request: taskRequest,
      scored_candidates: sorted,
    };
  }

  ackCallback(data) {
    const ackId = data.ack_id;
    const status = data.status;
    const reason = data.reason;
    const roverId = data.rover_id || "unknown";

    if (this.pendingCommands[ackId]) {
      const cmdInfo = this.pendingCommands[ackId];
      const rtt = LSOASTime.now() / 1000 - cmdInfo.sentAt;

      if (status === "ACCEPTED") {
        this.bus.emit("log", {
          tag: "ack",
          text: `ACK ${ackId} [${roverId}]: ACCEPTED (RTT: ${rtt.toFixed(2)}s)`,
        });
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `ACK ${ackId} [${roverId}]: REJECTED${reason ? ` (${reason})` : ""}`,
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
          text: `ACK ${ackId} [${roverId}]: ACCEPTED (late)`,
        });
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `ACK ${ackId} [${roverId}]: REJECTED${reason ? ` (${reason})` : ""}`,
        });
      }
    }

    this.bus.emit("pending:update", this.pendingCommands);
  }

  handleBufferedCommand(data) {
    const cmdId = data?.cmdId;
    if (!cmdId || !this.pendingCommands[cmdId]) return;

    const info = this.pendingCommands[cmdId];
    const roverId = data?.roverId || info.roverId;
    const transitioned = !info.buffered;
    info.buffered = true;
    info.bufferedAt = LSOASTime.now() / 1000;

    if (transitioned) {
      this.bus.emit("log", {
        tag: "system",
        text: `${cmdId} [${roverId}] queued at relay until LOS returns`,
      });
    }
    this.bus.emit("pending:update", this.pendingCommands);
  }

  handleReleasedCommand(data) {
    const cmdId = data?.cmdId;
    if (!cmdId || !this.pendingCommands[cmdId]) return;

    const info = this.pendingCommands[cmdId];
    if (!info.buffered) return;
    const roverId = data?.roverId || info.roverId;

    info.buffered = false;
    info.sentAt = LSOASTime.now() / 1000;
    info.bufferedAt = null;
    this.bus.emit("log", {
      tag: "system",
      text: `${cmdId} [${roverId}] released from relay buffer`,
    });
    this.bus.emit("pending:update", this.pendingCommands);
  }

  checkTimeouts() {
    const now = LSOASTime.now() / 1000;
    const timedOut = [];

    for (const [cmdId, info] of Object.entries(this.pendingCommands)) {
      if (info.buffered) continue;
      if (now - info.sentAt > this.ackTimeout) {
        timedOut.push(cmdId);
      }
    }

    for (const cmdId of timedOut) {
      const info = this.pendingCommands[cmdId];
      if (info.attempt < this.maxRetries) {
        info.attempt++;
        info.sentAt = LSOASTime.now() / 1000;
        this.bus.emit("log", {
          tag: "system",
          text: `No ACK for ${cmdId} [${info.roverId}], retrying (attempt ${info.attempt}/${this.maxRetries})`,
        });
        this.bus.emit("earth:uplink_cmd", info.cmdData);
      } else {
        this.bus.emit("log", {
          tag: "ack-fail",
          text: `Command ${cmdId} [${info.roverId}] failed after ${this.maxRetries} attempts`,
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

  sendCommand(cmdType, taskId = null, roverId = null, taskOptions = null) {
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
      ts: LSOASTime.now() / 1000,
    };

    if (taskId) cmdData.task_id = taskId;
    if (taskOptions) {
      cmdData.task_type = taskOptions.task_type;
      cmdData.difficulty_level = taskOptions.difficulty_level;
      cmdData.required_capabilities = taskOptions.required_capabilities;
      cmdData.mission_phase = taskOptions.mission_phase;
      cmdData.target_site = taskOptions.target_site || null;
      cmdData.assignment_score_breakdown =
        taskOptions.assignment_score_breakdown || null;
      cmdData.predicted_fault_probability =
        taskOptions.predicted_fault_probability;
      cmdData.selected_rover = taskOptions.selected_rover || targetRoverId;
      cmdData.reject_reason = taskOptions.reject_reason || null;
    }

    // Track pending
    this.pendingCommands[cmdId] = {
      sentAt: LSOASTime.now() / 1000,
      cmdType,
      attempt: 1,
      cmdData,
      roverId: targetRoverId,
      buffered: false,
      bufferedAt: null,
    };

    // Send to space link
    this.bus.emit("earth:uplink_cmd", cmdData);
    this.bus.emit("log", {
      tag: "cmd",
      text: `Sent ${cmdId} -> [${targetRoverId}]: ${cmdType}${taskId ? " " + taskId : ""}${taskOptions ? ` (${taskOptions.task_type}/${taskOptions.difficulty_level})` : ""}`,
    });
    this.bus.emit("cmd:sent", {
      cmdId,
      cmdType,
      taskId,
      roverId: targetRoverId,
      taskType: taskOptions?.task_type || null,
      difficultyLevel: taskOptions?.difficulty_level || null,
    });
    this.bus.emit("pending:update", this.pendingCommands);

    return cmdId;
  }

  destroy() {
    LSOASTime.clearInterval(this.timeoutInterval);
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
  constructor(options = {}) {
    const roverIds = resolveRoverIds();
    const runtimeOptions = resolveRuntimeOptions();
    const taskCatalog = options.taskCatalog || FALLBACK_TASK_CATALOG;
    this.celestialDynamics = new CelestialDynamics();
    this.bus = new EventBus();
    this.config = {
      baseLatency: runtimeOptions.baseLatency,
      jitter: runtimeOptions.jitter,
      dropRate: runtimeOptions.dropRate,
      faultProbability: runtimeOptions.faultProbability,
      roverIds,
      taskCatalog,
      celestialDynamics: this.celestialDynamics,
      latestCelestialFrame: this.celestialDynamics.computeFrame(
        LSOASTime.now(),
      ),
    };

    this.startTime = LSOASTime.now();
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
    this.publishCelestialFrame();
    this.celestialInterval = LSOASTime.setInterval(
      () => this.publishCelestialFrame(),
      1000,
    );
  }

  publishCelestialFrame(nowMs = LSOASTime.now()) {
    const frame = this.celestialDynamics.computeFrame(nowMs);
    this.config.latestCelestialFrame = frame;
    this.bus.emit("celestial:ephemeris", frame);
  }

  applyRoverRuntimeOverrides(runtimeOptions) {
    const roversById = new Map(
      this.rovers.map((rover) => [rover.roverId, rover]),
    );
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

        if (
          state === RoverNode.STATE_SAFE_MODE ||
          state === RoverNode.STATE_ERROR
        ) {
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

  sendCommand(type, taskId, roverId = null, taskOptions = null) {
    return this.earth.sendCommand(type, taskId, roverId, taskOptions);
  }

  selectBestRoverForTask(taskId, taskOptions = {}) {
    return this.earth.selectBestRoverForTask(taskId, taskOptions);
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
    return (LSOASTime.now() - this.startTime) / 1000;
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
    LSOASTime.clearInterval(this.celestialInterval);
  }
}

// Export for use by app.js
window.SimulationController = SimulationController;
