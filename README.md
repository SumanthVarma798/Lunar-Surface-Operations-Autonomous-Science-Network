<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Humble-blue?style=for-the-badge&logo=ros" alt="ROS 2 Humble" />
  <img src="https://img.shields.io/badge/Python-3.10+-green?style=for-the-badge&logo=python&logoColor=white" alt="Python" />
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge" alt="License" />
  <img src="https://img.shields.io/badge/Platform-Web%20%7C%20Docker-orange?style=for-the-badge&logo=docker" alt="Platform" />
</p>

<h1 align="center">🌙 LSOAS</h1>
<h3 align="center">Lunar Surface Operations Autonomous Science Network</h3>

<p align="center">
  <em>A simulated mission control system for autonomous lunar rover operations<br/>
  with realistic Earth↔Moon communication, fault-tolerant command protocols,<br/>
  and a NASA-inspired web dashboard.</em>
</p>

---

## 🖥️ Web Simulation Dashboard

A browser-based mission control interface that simulates all four ROS nodes in real-time — no Docker, no ROS setup needed. Designed with an Apple/Jony Ive aesthetic for a premium experience.

### Dashboard — Idle State

> Rover awaiting commands. Telemetry streams through the Space Link relay with realistic 1.3s latency. All systems nominal.

![Dashboard Idle](docs/screenshots/dashboard-idle.png)

### Dashboard — Task Execution

> `START_TASK SAMPLE-001` sent from Earth Station. The command traverses the Space Link with simulated delay, the rover acknowledges with an ACK (RTT: 2.87s), and begins executing the 10-step task sequence.

![Dashboard Executing](docs/screenshots/dashboard-executing.png)

### Dashboard — Safe Mode & Fault Detection

> A fault was detected during task execution (step 6). The rover entered `SAFE_MODE`, halting all operations. The `GO_SAFE` command is acknowledged. Command log shows full history with RTT measurements.

![Dashboard Safe Mode](docs/screenshots/dashboard-safe-mode.png)

### Dashboard — Light Theme

> Full light theme variant with the same telemetry fidelity. Toggle anytime via the ☀️ icon or system preference.

![Dashboard Light Theme](docs/screenshots/dashboard-light-theme.png)

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MISSION CONTROL                              │
│                                                                     │
│   ┌──────────┐     ┌────────────┐     ┌──────────┐                 │
│   │  EARTH   │────▶│ SPACE LINK │────▶│  ROVER   │                 │
│   │ STATION  │◀────│   (RELAY)  │◀────│  (MOON)  │                 │
│   └──────────┘     └────────────┘     └──────────┘                 │
│        │                                    │                       │
│        │           ┌────────────┐           │                       │
│        └──────────▶│ TELEMETRY  │◀──────────┘                       │
│                    │  MONITOR   │                                   │
│                    └────────────┘                                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Node Descriptions

| Node                     | Role                             | Key Behaviors                                                                                 |
| ------------------------ | -------------------------------- | --------------------------------------------------------------------------------------------- |
| **🌍 Earth Station**     | Ground control command interface | Sends JSON commands with unique IDs, tracks pending ACKs, retries on timeout (8s)             |
| **📡 Space Link**        | Moon↔Earth communication relay   | Simulates 1.3s base latency, ±0.2s jitter, 5% packet drop rate, message duplication           |
| **🤖 Lunar Rover**       | Autonomous rover on the Moon     | State machine (IDLE→EXECUTING→SAFE_MODE→ERROR), fault detection, battery drain, 10-step tasks |
| **📊 Telemetry Monitor** | Passive telemetry display        | Subscribes to all telemetry and ACK streams, formats data for display                         |

### Rover State Machine

```
            START_TASK
  ┌──────┐ ──────────▶ ┌───────────┐
  │ IDLE │              │ EXECUTING │
  └──┬───┘ ◀────────── └─────┬─────┘
     │       task done       │
     │                  fault detected
     │ RESET                 │
     │       ┌───────────┐   │
     └────── │ SAFE_MODE │ ◀─┘
             └─────┬─────┘
                   │ unrecoverable
             ┌─────▼─────┐
             │   ERROR   │
             └───────────┘
```

### Command Protocol

```
Earth ──[CMD: {cmd_id, type, ts}]──▶ Space Link ──[delay]──▶ Rover
Earth ◀──[ACK: {cmd_id, status}]──── Space Link ◀──[delay]──── Rover
```

- Commands are JSON-encoded with unique IDs (`c-00001`, `c-00002`, ...)
- ACKs return `ACCEPTED` or `REJECTED` with the original command ID
- Earth retries after 8 seconds if no ACK received
- Round-trip time (RTT) is measured and displayed

---

## 🚀 Quick Start

### Option 1: Web Simulation (Recommended)

No dependencies needed — just a browser and Python.

```bash
# Clone the repository
git clone https://github.com/SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network.git
cd Lunar-Surface-Operations-Autonomous-Science-Network

# Start the web server
cd web-sim && python3 -m http.server 8080
```

Open **http://localhost:8080** and start sending commands! 🎉

#### Keyboard Shortcuts

| Key         | Command    |
| ----------- | ---------- |
| `S`         | Start Task |
| `A`         | Abort      |
| `Shift + S` | Safe Mode  |
| `R`         | Reset      |

#### Configurable Parameters

Use the sliders on the right panel to adjust in real-time:

| Parameter         | Default | Range      | Description                         |
| ----------------- | ------- | ---------- | ----------------------------------- |
| Base Latency      | 1.3s    | 0.1 – 5.0s | One-way signal travel time          |
| Jitter            | ±0.2s   | 0 – 1.0s   | Random delay variation              |
| Drop Rate         | 5%      | 0 – 50%    | Probability of packet loss          |
| Fault Probability | 10%     | 0 – 50%    | Chance of rover fault per task step |

---

### Option 2: ROS 2 Simulation (Full Setup)

Requires Docker and ROS 2 Humble.

#### Prerequisites

- Docker Desktop
- Make

#### Build & Run

```bash
# Start the Docker container
make docker

# Build the ROS workspace (inside Docker)
make build

# Launch nodes in separate terminals:
make space-link     # Terminal 1: Start the Space Link relay
make telemetry      # Terminal 2: Start the Telemetry Monitor
make rover          # Terminal 3: Start the Rover
make earth          # Terminal 4: Start Earth Station (interactive)
```

#### Earth Station Commands (ROS)

Once the Earth Station is running, use the interactive prompt:

```
Commands: START_TASK | ABORT | GO_SAFE | RESET | quit
>> START_TASK
>> GO_SAFE
>> RESET
```

---

## 📁 Project Structure

```
Lunar-Surface-Operations-Autonomous-Science-Network/
│
├── web-sim/                          # 🌐 Browser-based simulation
│   ├── index.html                    #    Dashboard layout
│   ├── index.css                     #    Design system (dark/light themes)
│   ├── simulation.js                 #    Simulation engine (all 4 nodes in JS)
│   └── app.js                        #    UI controller & DOM bindings
│
├── lunar_ops/                        # 🤖 ROS 2 implementation
│   ├── docs/
│   │   ├── concept.md                #    Base concept document
│   │   ├── implementation_reference.md
│   │   └── test_results.md
│   ├── rover_ws/src/rover_core/
│   │   └── rover_core/
│   │       ├── rover_node.py         #    Rover state machine & telemetry
│   │       ├── earth_node.py         #    Earth command interface & ACK tracking
│   │       ├── space_link_node.py    #    Communication relay simulator
│   │       └── telemetry_monitor.py  #    Telemetry display node
│   └── README.md
│
├── scripts/
│   └── rosdev.sh                     #    Docker dev environment helper
│
├── docs/screenshots/                 #    Dashboard screenshots
├── Makefile                          #    Build & run automation
└── .gitignore
```

---

## 🔧 Development Workflow

This project uses a **branch-based workflow** with pull requests for all changes.

### Creating a New Feature

```bash
# 1. Start from latest main
git checkout main && git pull origin main

# 2. Create a feature branch
git checkout -b feature/your-feature-name

# 3. Make your changes, then commit
git add <files>
git commit -m "feat: description of your change"

# 4. Push and create a PR
git push -u origin feature/your-feature-name
```

Then open a Pull Request on GitHub to merge into `main`.

### Commit Convention

We use [Conventional Commits](https://www.conventionalcommits.org/):

| Prefix      | Usage                 |
| ----------- | --------------------- |
| `feat:`     | New feature           |
| `fix:`      | Bug fix               |
| `chore:`    | Maintenance, cleanup  |
| `docs:`     | Documentation changes |
| `refactor:` | Code restructuring    |

### Branch Naming

| Prefix     | Usage                  |
| ---------- | ---------------------- |
| `feature/` | New features           |
| `fix/`     | Bug fixes              |
| `chore/`   | Cleanup or maintenance |
| `docs/`    | Documentation updates  |

---

## 📡 Web Simulation — Technical Details

The web dashboard replaces ROS topics with a JavaScript **EventBus** pattern:

| ROS Concept                     | Web Equivalent                       |
| ------------------------------- | ------------------------------------ |
| ROS Topic (`/earth/uplink_cmd`) | `bus.emit('earth:uplink_cmd', data)` |
| ROS Subscriber                  | `bus.on('event', callback)`          |
| ROS Timer                       | `setInterval()`                      |
| ROS Node                        | JavaScript `class`                   |
| `std_msgs/String` (JSON)        | Native JavaScript objects            |

### Key Classes

- **`EventBus`** — Pub/sub message broker replacing ROS topics
- **`RoverNode`** — State machine, telemetry publisher, fault detection
- **`SpaceLinkNode`** — Bidirectional relay with configurable delay/drop
- **`EarthNode`** — Command sender with ACK tracking and retry
- **`TelemetryMonitor`** — Telemetry formatter and display
- **`SimulationEngine`** — Orchestrator that initializes and connects all nodes

---

## 🎨 Design Philosophy

The web dashboard follows an Apple/Jony Ive design language:

- **Dark-first** — Deep space-black background with high-contrast elements
- **Frosted glass** — `backdrop-filter: blur()` panels for depth
- **Typography** — SF Pro / Inter with carefully weighted hierarchy
- **Color semantics** — Green (nominal), Yellow (warning), Red (error), Blue (executing)
- **Micro-animations** — Smooth transitions on state changes and signal particles
- **Information density** — All critical data visible at a glance, no hidden menus

---

## 🧪 Testing Scenarios

Try these scenarios in the web simulation to explore the system:

| Scenario             | Steps                                   | What to Observe                                         |
| -------------------- | --------------------------------------- | ------------------------------------------------------- |
| **Happy path**       | Click START TASK → wait for completion  | 10 steps execute, battery drains, state returns to IDLE |
| **Mid-task abort**   | START TASK → wait 3s → ABORT            | Task interrupted, rover returns to IDLE                 |
| **Safe mode**        | START TASK → SAFE MODE during execution | Rover enters protected SAFE_MODE, ignores further tasks |
| **Recovery**         | SAFE MODE → RESET                       | Rover returns to IDLE, ready for new commands           |
| **High packet loss** | Set Drop Rate to 40% → START TASK       | Commands may need retries, telemetry gaps visible       |
| **High latency**     | Set Latency to 4s → START TASK          | Longer RTT times, delayed ACKs, visible relay delays    |

---

## 📄 License

MIT

---

<p align="center">
  <em>Built for learning. Inspired by real mission control systems.</em><br/>
  <strong>🌙 LSOAS — Lunar Surface Operations Autonomous Science Network</strong>
</p>
