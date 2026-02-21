const fs = require("fs");

const TIME_CLASS = `/* ═══════════════════════════════════════════════════════
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
    if (typeof requestAnimationFrame !== 'undefined') {
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
    this.timers.set(id, { type: 'timeout', target: targetSimTime, fn });
    return id;
  }
  
  setInterval(fn, intervalRealMs) {
    const id = this.timerId++;
    this.timers.set(id, { type: 'interval', interval: intervalRealMs, next: this.now() + intervalRealMs, fn });
    return id;
  }
  
  clearTimeout(id) { this.timers.delete(id); }
  clearInterval(id) { this.timers.delete(id); }
  
  tick() {
    if (typeof requestAnimationFrame !== 'undefined') {
      requestAnimationFrame(this.tick);
    }
    const currentSim = this.now();
    
    for (const [id, timer] of Array.from(this.timers.entries())) {
      if (timer.type === 'timeout') {
        if (currentSim >= timer.target) {
          this.timers.delete(id);
          timer.fn();
        }
      } else if (timer.type === 'interval') {
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
`;

let content = fs.readFileSync("simulation.js", "utf8");

// Replace top banner
content = content.replace(
  /\/\* ═══════════════════════════════════════════════════════[\s\S]*?═══════════════════════════════════════════════════════ \*\//,
  TIME_CLASS,
);

// Replace Date.now() -> LSOASTime.now()
content = content.replace(/Date\.now\(\)/g, "LSOASTime.now()");

// Replace setInterval -> LSOASTime.setInterval (careful not to replace if it's already LSOASTime)
content = content.replace(
  /(?<!LSOASTime\.)setInterval\(/g,
  "LSOASTime.setInterval(",
);

// Replace setTimeout -> LSOASTime.setTimeout
content = content.replace(
  /(?<!LSOASTime\.)setTimeout\(/g,
  "LSOASTime.setTimeout(",
);

// Replace clearInterval -> LSOASTime.clearInterval
content = content.replace(
  /(?<!LSOASTime\.)clearInterval\(/g,
  "LSOASTime.clearInterval(",
);

fs.writeFileSync("simulation.js", content, "utf8");

console.log("Patched simulation.js");
