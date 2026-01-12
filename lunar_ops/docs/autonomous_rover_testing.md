# Autonomous Rover State Machine - Testing Guide

## Overview

The rover now implements a **realistic autonomous state machine** instead of puppetry. It makes its own decisions about state transitions and can refuse invalid commands.

## States

| State | Description |
|-------|-------------|
| `IDLE` | Rover is idle and ready to accept tasks |
| `EXECUTING` | Rover is actively executing a task |
| `SAFE_MODE` | Rover detected a fault and is in safe mode (refuses new tasks) |
| `ERROR` | Error state (reserved for future use) |

## Commands from Earth

| Command | Format | Description |
|---------|--------|-------------|
| `START_TASK` | `START_TASK:<task_id>` | Start a new task with the given ID |
| `ABORT` | `ABORT` | Abort the currently executing task |
| `GO_SAFE` | `GO_SAFE` | Command rover to enter safe mode immediately |
| `RESET` | `RESET` | Reset rover from safe mode back to idle |

## State Transition Rules

### 1. START_TASK Command
- ✅ **IDLE → EXECUTING**: Task accepted and started
- ❌ **EXECUTING**: Refused (already executing a task)
- ❌ **SAFE_MODE**: Refused (must RESET first)

### 2. ABORT Command
- ✅ **EXECUTING → IDLE**: Task aborted successfully
- ℹ️ **IDLE/SAFE_MODE**: Acknowledged but no effect

### 3. GO_SAFE Command
- ✅ **Any state → SAFE_MODE**: Immediately enters safe mode

### 4. RESET Command
- ✅ **SAFE_MODE → IDLE**: Exits safe mode
- ℹ️ **Other states**: Acknowledged but no effect

### 5. Autonomous Fault Detection
- 🚨 **EXECUTING → SAFE_MODE**: Rover detects fault during task execution
- Fault probability: 10% per task step
- Task duration: ~10 seconds (10 steps at 1Hz)

## Test Scenarios

### Scenario 1: Normal Task Execution
```
Earth> start SAMPLE-001
```
**Expected behavior:**
1. Rover transitions IDLE → EXECUTING
2. Task executes for ~10 seconds (watch progress in rover terminal)
3. If no fault: EXECUTING → IDLE (task completed)
4. If fault detected: EXECUTING → SAFE_MODE

### Scenario 2: Abort During Execution
```
Earth> start DRILL-042
# Wait a few seconds
Earth> abort
```
**Expected behavior:**
1. Rover transitions IDLE → EXECUTING
2. Task starts executing
3. ABORT command received
4. Rover transitions EXECUTING → IDLE

### Scenario 3: Refusing Task in SAFE_MODE
```
Earth> safe
Earth> start ANALYSIS-123
```
**Expected behavior:**
1. Rover transitions to SAFE_MODE
2. START_TASK command is **refused** with warning message
3. Rover remains in SAFE_MODE

### Scenario 4: Recovery from SAFE_MODE
```
Earth> safe
Earth> reset
Earth> start PHOTO-999
```
**Expected behavior:**
1. Rover enters SAFE_MODE
2. RESET command accepted
3. Rover transitions SAFE_MODE → IDLE
4. New task accepted and started

### Scenario 5: Refusing Multiple Tasks
```
Earth> start TASK-001
# Immediately try another task
Earth> start TASK-002
```
**Expected behavior:**
1. First task accepted (IDLE → EXECUTING)
2. Second task **refused** (already executing TASK-001)

## How to Test

### Terminal 1: Rover Node
```bash
make test-rover
```
Watch for:
- 🤖 Rover initialization
- ⚙️ Task execution steps
- 🚨 Fault detection
- ✅ Task completion
- State transition messages

### Terminal 2: Earth Station
```bash
make test-earth
```
Use the interactive command interface:
- `start <task_id>` - Start a task
- `abort` - Abort current task
- `safe` - Enter safe mode
- `reset` - Reset from safe mode
- `help` - Show commands
- `quit` - Exit

## Example Test Session

```
Earth> start SAMPLE-001
[INFO] 📤 Sent command: START_TASK:SAMPLE-001

# Rover terminal shows:
# [INFO] 📡 Command received: START_TASK:SAMPLE-001
# [INFO] ✅ Started task: SAMPLE-001
# [INFO] ⚙️  Executing task SAMPLE-001: step 1/10
# [INFO] ⚙️  Executing task SAMPLE-001: step 2/10
# ... continues for ~10 seconds ...

# If lucky (90% chance):
# [INFO] ✅ Task SAMPLE-001 completed!

# If unlucky (10% chance):
# [ERROR] 🚨 FAULT DETECTED during task SAMPLE-001!
# Rover enters SAFE_MODE

Earth> reset
[INFO] 📤 Sent command: RESET

# Rover terminal shows:
# [INFO] 🔄 RESET: Leaving SAFE_MODE → IDLE
```

## What Changed

### Before (Puppetry)
```python
def command_callback(self, msg):
    self.state = msg.data  # Direct state control from Earth
```

### After (Autonomy)
```python
def command_callback(self, msg):
    # Parse command
    if command.startswith("START_TASK:"):
        task_id = command.split(":", 1)[1]
        self.handle_start_task(task_id)  # Rover decides if it can accept
    elif command == "ABORT":
        self.handle_abort()  # Rover decides if abort is valid
    # ... etc
```

## Key Features

✅ **Command Parsing**: Rover parses structured commands instead of accepting raw state  
✅ **State Validation**: Rover validates transitions and refuses invalid ones  
✅ **Autonomous Fault Detection**: Rover can detect faults and transition to SAFE_MODE autonomously  
✅ **Task Management**: Tracks current task ID and progress  
✅ **Rich Telemetry**: Reports state, task, and progress  
✅ **Safe Mode Protection**: Requires explicit RESET to leave safe mode  

## Success Criteria

- [x] Rover refuses tasks when in SAFE_MODE
- [x] Rover refuses multiple simultaneous tasks
- [x] ABORT command works during execution
- [x] Rover can autonomously detect faults and enter SAFE_MODE
- [x] RESET is the only way to leave SAFE_MODE
- [x] Task execution shows realistic progress
- [x] Unknown commands are rejected with warnings
