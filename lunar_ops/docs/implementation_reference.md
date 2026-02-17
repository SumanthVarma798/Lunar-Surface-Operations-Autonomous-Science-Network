# Rover Autonomy Implementation - Quick Reference

## Files Modified

### [`rover_node.py`](../rover_ws/src/rover_core/rover_core/rover_node.py)
**Complete rewrite** from puppetry to autonomy

### [`earth_node.py`](../rover_ws/src/rover_core/rover_core/earth_node.py)
**Added interactive command interface** for operator control

## State Machine Implementation

### State Constants
```python
STATE_IDLE = "IDLE"
STATE_EXECUTING = "EXECUTING"
STATE_SAFE_MODE = "SAFE_MODE"
STATE_ERROR = "ERROR"
```

### Command Handlers

#### `handle_start_task(task_id)`
```python
def handle_start_task(self, task_id):
    if self.state == self.STATE_SAFE_MODE:
        self.refuse()  # ❌ SAFE_MODE blocks tasks
        return
    
    if self.state == self.STATE_EXECUTING:
        self.refuse()  # ❌ Already busy
        return
    
    # ✅ Accept task
    self.state = self.STATE_EXECUTING
    self.current_task_id = task_id
    self.task_counter = 0
```

#### `handle_abort()`
```python
def handle_abort(self):
    if self.state == self.STATE_EXECUTING:
        # ✅ EXECUTING → IDLE
        self.state = self.STATE_IDLE
        self.current_task_id = None
        self.task_counter = 0
    else:
        # ℹ️ Acknowledge but no effect
        pass
```

#### `handle_go_safe()`
```python
def handle_go_safe(self):
    # ✅ Any state → SAFE_MODE
    old_state = self.state
    self.state = self.STATE_SAFE_MODE
    self.current_task_id = None
    self.task_counter = 0
```

#### `handle_reset()`
```python
def handle_reset(self):
    if self.state == self.STATE_SAFE_MODE:
        # ✅ SAFE_MODE → IDLE
        self.state = self.STATE_IDLE
        self.current_task_id = None
        self.task_counter = 0
    else:
        # ℹ️ Only works from SAFE_MODE
        pass
```

### Autonomous Execution

#### `execute_task_step()`
Runs at 1 Hz when `state == EXECUTING`

```python
def execute_task_step(self):
    if self.state != self.STATE_EXECUTING:
        return  # Only runs during task execution
    
    self.task_counter += 1
    
    # 🚨 Autonomous fault detection
    if random.random() < self.fault_probability:
        self.state = self.STATE_SAFE_MODE  # Autonomous decision!
        self.current_task_id = None
        self.task_counter = 0
        return
    
    # ✅ Task completion
    if self.task_counter >= 10:
        self.state = self.STATE_IDLE
        self.current_task_id = None
        self.task_counter = 0
```

## Command Protocol

### Format
```
START_TASK:<task_id>  # e.g., "START_TASK:SAMPLE-042"
ABORT                 # No parameters
GO_SAFE               # No parameters  
RESET                 # No parameters
```

### Parsing
```python
def command_callback(self, msg):
    command = msg.data.strip()
    
    if command.startswith("START_TASK:"):
        task_id = command.split(":", 1)[1]
        self.handle_start_task(task_id)
    elif command == "ABORT":
        self.handle_abort()
    elif command == "GO_SAFE":
        self.handle_go_safe()
    elif command == "RESET":
        self.handle_reset()
    else:
        self.get_logger().warn(f"Unknown command: {command}")
```

## Telemetry Protocol

### Format
```
STATE=<state>                                    # When idle/safe
STATE=<state> TASK=<id> PROGRESS=<n>/10         # During execution
```

### Examples
```
STATE=IDLE
STATE=EXECUTING TASK=SAMPLE-001 PROGRESS=3/10
STATE=SAFE_MODE
```

### Implementation
```python
def publish_telemetry(self):
    msg = String()
    if self.current_task_id:
        msg.data = f"STATE={self.state} TASK={self.current_task_id} PROGRESS={self.task_counter}/10"
    else:
        msg.data = f"STATE={self.state}"
    self.telemetry_pub.publish(msg)
```

## Configuration Parameters

```python
self.fault_probability = 0.1    # 10% fault chance per step
task_duration = 10              # 10 steps at 1Hz = ~10 seconds
telemetry_rate = 2.0            # Publish every 2 seconds
execution_rate = 1.0            # Execute step every 1 second
```

## Interactive Earth Station

### Command Loop
```python
def command_loop(node):
    while rclpy.ok():
        user_input = input("Earth> ").strip()
        parts = user_input.split(maxsplit=1)
        cmd = parts[0].lower()
        
        if cmd == "start":
            task_id = parts[1]
            node.send_command(f"START_TASK:{task_id}")
        elif cmd == "abort":
            node.send_command("ABORT")
        # ... etc
```

### Threading Model
```python
# ROS2 spin in background thread
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

# Command loop in main thread (for keyboard input)
command_loop(node)
```

## State Transition Matrix

| Current State | Command | New State | Notes |
|--------------|---------|-----------|-------|
| IDLE | START_TASK | EXECUTING | Task accepted |
| IDLE | ABORT | IDLE | No effect |
| IDLE | GO_SAFE | SAFE_MODE | Immediate transition |
| IDLE | RESET | IDLE | No effect |
| EXECUTING | START_TASK | EXECUTING | **Refused** (busy) |
| EXECUTING | ABORT | IDLE | Task aborted |
| EXECUTING | GO_SAFE | SAFE_MODE | Immediate transition |
| EXECUTING | - (fault) | SAFE_MODE | **Autonomous!** |
| EXECUTING | - (complete) | IDLE | **Autonomous!** |
| SAFE_MODE | START_TASK | SAFE_MODE | **Refused** (need RESET) |
| SAFE_MODE | ABORT | SAFE_MODE | No effect |
| SAFE_MODE | GO_SAFE | SAFE_MODE | Already safe |
| SAFE_MODE | RESET | IDLE | Recovery |

## Testing Commands

### Quick Test Sequence
```bash
# Terminal 1: Start rover
make test-rover

# Terminal 2: Start earth station
make test-earth

# In Earth terminal:
Earth> start SAMPLE-001    # Normal execution
Earth> start DRILL-042     # Should refuse (still executing)
# Wait for completion or fault

Earth> safe                # Force safe mode
Earth> start TEST-123      # Should refuse
Earth> reset               # Recovery
Earth> start PHOTO-999     # Now accepted
Earth> abort               # Abort task
```

### Using ROS2 CLI
```bash
# Start a task
ros2 topic pub --once /rover/command std_msgs/msg/String '{data: "START_TASK:DEMO-001"}'

# Abort
ros2 topic pub --once /rover/command std_msgs/msg/String '{data: "ABORT"}'

# Safe mode
ros2 topic pub --once /rover/command std_msgs/msg/String '{data: "GO_SAFE"}'

# Reset
ros2 topic pub --once /rover/command std_msgs/msg/String '{data: "RESET"}'

# Monitor telemetry
ros2 topic echo /rover/telemetry
```

## Key Differences from Puppetry

### Puppetry (Before)
```python
# Earth controls everything
def command_callback(self, msg):
    self.state = msg.data  # Blind obedience
```

### Autonomy (After)
```python
# Rover makes intelligent decisions
def command_callback(self, msg):
    command = self.parse(msg)
    
    # Validate based on current state
    if not self.can_accept(command):
        self.refuse_with_reason()
        return
    
    # Execute with autonomous monitoring
    self.execute_with_fault_detection()
```

## Success Metrics

✅ **Autonomous decision-making**: Rover validates all commands  
✅ **Fault detection**: Self-monitoring during execution  
✅ **State protection**: SAFE_MODE requires explicit recovery  
✅ **Task management**: Full lifecycle tracking  
✅ **Clear communication**: Rich logging and telemetry  
✅ **Safety first**: Refuses unsafe operations  

## Next Steps (Ideas for Extension)

1. **Multiple fault types**: Power, thermal, communication
2. **Task prioritization**: Abort low-priority for high-priority
3. **Resource management**: Battery, memory, disk
4. **Recovery strategies**: Auto-recovery for minor faults
5. **Task queue**: Accept multiple tasks for sequencing
6. **Health monitoring**: Periodic self-checks
7. **Graceful degradation**: Partial operation modes
