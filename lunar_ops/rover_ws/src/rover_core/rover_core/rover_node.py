import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json
import time
import math

class RoverNode(Node):
    """Autonomous rover with realistic state machine behavior.

    Supports multi-rover constellations via configurable rover_id parameter.
    Each instance uses namespaced topics: /rover/{id}/downlink_telemetry,
    /rover/{id}/command, /rover/{id}/ack.
    """
    
    # Valid states
    STATE_IDLE = "IDLE"
    STATE_EXECUTING = "EXECUTING"
    STATE_SAFE_MODE = "SAFE_MODE"
    STATE_ERROR = "ERROR"
    
    def __init__(self):
        # Declare rover_id parameter before super().__init__ so we can use it
        # to name the node dynamically.  We do a two-step init:
        #   1. Temporary super init with a placeholder name
        #   2. Read parameter and rebuild publishers/subscribers with namespaced topics
        super().__init__('rover_node')

        # --- Rover identity ---
        self.declare_parameter('rover_id', 'rover_1')
        self.rover_id = self.get_parameter('rover_id').value

        # State machine
        self.state = self.STATE_IDLE
        self.current_task_id = None
        self.task_counter = 0  # Simulates task progress
        self.fault_probability = 0.1  # 10% chance of fault during execution
        self.battery_level = 1.0  # Start at full charge
        self.last_fault = None  # Track last fault message

        # --- Simulated sensor data ---
        # Position: random starting lat/lon on the lunar surface
        self.position = {
            "lat": round(random.uniform(-45.0, 45.0), 6),
            "lon": round(random.uniform(-180.0, 180.0), 6),
        }
        # Solar exposure: 0.0 (night) to 1.0 (full sun) — driven by sim clock
        self.solar_exposure = round(random.uniform(0.6, 1.0), 2)
        self._solar_phase = random.uniform(0, 2 * math.pi)  # random start phase
        # Data buffer: bytes waiting to be downlinked
        self.data_buffer_size = 0

        # --- Namespaced topics ---
        topic_prefix = f'/rover/{self.rover_id}'

        self.telemetry_pub = self.create_publisher(
            String,
            f'{topic_prefix}/downlink_telemetry',
            10
        )

        self.ack_pub = self.create_publisher(
            String,
            f'{topic_prefix}/ack',
            10
        )

        self.command_sub = self.create_subscription(
            String,
            f'{topic_prefix}/command',
            self.command_callback,
            10
        )

        self.timer = self.create_timer(2.0, self.publish_telemetry)
        self.task_timer = self.create_timer(1.0, self.execute_task_step)
        
        self.get_logger().info(
            f"🤖 Rover [{self.rover_id}] initialized in {self.state} state\n"
            f"   Topics: {topic_prefix}/{{downlink_telemetry,command,ack}}"
        )

    def command_callback(self, msg):
        """Process JSON commands from Earth with autonomous decision-making"""
        try:
            # Parse JSON command
            cmd_data = json.loads(msg.data)
            cmd_id = cmd_data.get('cmd_id', 'unknown')
            cmd_type = cmd_data.get('type')
            task_id = cmd_data.get('task_id')
            
            self.get_logger().info(f"📡 [{self.rover_id}] Command received [{cmd_id}]: {cmd_type}")
            
            # Process command and get result
            if cmd_type == "START_TASK":
                success, reason = self.handle_start_task(task_id)
                self.send_ack(cmd_id, success, reason)
            elif cmd_type == "ABORT":
                success, reason = self.handle_abort()
                self.send_ack(cmd_id, success, reason)
            elif cmd_type == "GO_SAFE":
                success, reason = self.handle_go_safe()
                self.send_ack(cmd_id, success, reason)
            elif cmd_type == "RESET":
                success, reason = self.handle_reset()
                self.send_ack(cmd_id, success, reason)
            else:
                self.get_logger().warn(f"⚠️  Unknown command type: {cmd_type}")
                self.send_ack(cmd_id, False, f"Unknown command type: {cmd_type}")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse command JSON: {msg.data}")
            self.send_ack('unknown', False, f"Invalid JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            self.send_ack(cmd_data.get('cmd_id', 'unknown'), False, f"Error: {e}")

    def handle_start_task(self, task_id):
        """Start a new task if conditions allow"""
        if self.state == self.STATE_SAFE_MODE:
            reason = "Cannot start task: Rover in SAFE_MODE. Send RESET first."
            self.get_logger().warn(f"❌ {reason}")
            return False, reason
        
        if self.state == self.STATE_EXECUTING:
            reason = f"Cannot start task: Already executing task {self.current_task_id}"
            self.get_logger().warn(f"❌ {reason}")
            return False, reason
        
        # Accept the task
        self.state = self.STATE_EXECUTING
        self.current_task_id = task_id
        self.task_counter = 0
        self.get_logger().info(f"✅ [{self.rover_id}] Started task: {task_id}")
        return True, None

    def handle_abort(self):
        """Abort current task if executing"""
        if self.state == self.STATE_EXECUTING:
            self.get_logger().info(f"🛑 [{self.rover_id}] Aborting task: {self.current_task_id}")
            self.state = self.STATE_IDLE
            self.current_task_id = None
            self.task_counter = 0
            return True, None
        else:
            self.get_logger().info(f"ℹ️  ABORT received but rover is {self.state}, no task to abort")
            return True, None  # Still accepted, just no-op

    def handle_go_safe(self):
        """Enter safe mode immediately"""
        old_state = self.state
        self.state = self.STATE_SAFE_MODE
        self.current_task_id = None
        self.task_counter = 0
        if not self.last_fault:
            self.last_fault = "Commanded to SAFE_MODE"
        self.get_logger().warn(f"⚠️  [{self.rover_id}] Commanded to SAFE_MODE from {old_state}")
        return True, None

    def handle_reset(self):
        """Reset from safe mode to idle"""
        if self.state == self.STATE_SAFE_MODE:
            self.state = self.STATE_IDLE
            self.current_task_id = None
            self.task_counter = 0
            self.last_fault = None  # Clear fault on reset
            self.get_logger().info(f"🔄 [{self.rover_id}] RESET: Leaving SAFE_MODE → IDLE")
            return True, None
        else:
            self.get_logger().info(f"ℹ️  RESET received but rover is {self.state}, not in SAFE_MODE")
            return True, None  # Still accepted, just no-op

    def send_ack(self, cmd_id, success, reason=None):
        """Send command acknowledgment to Earth"""
        ack_data = {
            "ack_id": cmd_id,
            "rover_id": self.rover_id,
            "status": "ACCEPTED" if success else "REJECTED",
            "reason": reason,
            "ts": time.time()
        }
        
        msg = String()
        msg.data = json.dumps(ack_data)
        self.ack_pub.publish(msg)
        
        status_str = "✅ ACCEPTED" if success else f"❌ REJECTED ({reason})"
        self.get_logger().info(f"📤 [{self.rover_id}] ACK sent [{cmd_id}]: {status_str}")

    def execute_task_step(self):
        """Simulate autonomous task execution with fault detection"""
        if self.state != self.STATE_EXECUTING:
            return
        
        self.task_counter += 1
        
        # Drain battery during execution (0.5% per step)
        self.battery_level = max(0.0, self.battery_level - 0.005)
        
        # Accumulate data during task execution
        self.data_buffer_size += random.randint(128, 1024)

        # Simulate small position drift during movement
        self.position["lat"] += random.uniform(-0.0001, 0.0001)
        self.position["lon"] += random.uniform(-0.0001, 0.0001)
        self.position["lat"] = round(self.position["lat"], 6)
        self.position["lon"] = round(self.position["lon"], 6)
        
        # Simulate fault detection (realistic autonomy!)
        if random.random() < self.fault_probability:
            fault_msg = f"Fault during task execution (step {self.task_counter})"
            self.get_logger().error(f"🚨 [{self.rover_id}] FAULT DETECTED during task {self.current_task_id}!")
            self.state = self.STATE_SAFE_MODE
            self.last_fault = fault_msg
            self.current_task_id = None
            self.task_counter = 0
            return
        
        # Simulate task completion after 10 steps
        if self.task_counter >= 10:
            self.get_logger().info(f"✅ [{self.rover_id}] Task {self.current_task_id} completed!")
            self.state = self.STATE_IDLE
            self.current_task_id = None
            self.task_counter = 0
            # Flush data buffer on task completion
            self.data_buffer_size = 0
        else:
            self.get_logger().info(
                f"⚙️  [{self.rover_id}] Executing task {self.current_task_id}: "
                f"step {self.task_counter}/10"
            )

    def _update_solar_exposure(self):
        """Update simulated solar exposure using a slow sine wave + noise"""
        self._solar_phase += 0.02  # slow drift
        base = 0.5 + 0.5 * math.sin(self._solar_phase)
        noise = random.uniform(-0.05, 0.05)
        self.solar_exposure = round(max(0.0, min(1.0, base + noise)), 2)

    def publish_telemetry(self):
        """Publish current state as structured JSON with enriched fields"""
        self._update_solar_exposure()

        telemetry = {
            "ts": time.time(),
            "rover_id": self.rover_id,
            "state": self.state,
            "task_id": self.current_task_id,
            "battery": round(self.battery_level, 2),
            "position": {
                "lat": self.position["lat"],
                "lon": self.position["lon"],
            },
            "solar_exposure": self.solar_exposure,
            "data_buffer_size": self.data_buffer_size,
            "fault": self.last_fault,
        }
        
        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)

def main():
    rclpy.init()
    node = RoverNode()
    rclpy.spin(node)
    rclpy.shutdown()