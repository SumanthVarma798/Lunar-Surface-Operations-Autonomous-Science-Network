import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json
import time

class RoverNode(Node):
    """Autonomous rover with realistic state machine behavior"""
    
    # Valid states
    STATE_IDLE = "IDLE"
    STATE_EXECUTING = "EXECUTING"
    STATE_SAFE_MODE = "SAFE_MODE"
    STATE_ERROR = "ERROR"
    
    def __init__(self):
        super().__init__('rover_node')
        self.state = self.STATE_IDLE
        self.current_task_id = None
        self.task_counter = 0  # Simulates task progress
        self.fault_probability = 0.1  # 10% chance of fault during execution
        self.battery_level = 1.0  # Start at full charge
        self.last_fault = None  # Track last fault message

        self.telemetry_pub = self.create_publisher(
            String,
            '/rover/telemetry',
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/rover/command',
            self.command_callback,
            10
        )

        self.timer = self.create_timer(2.0, self.publish_telemetry)
        self.task_timer = self.create_timer(1.0, self.execute_task_step)
        
        self.get_logger().info(f"🤖 Rover initialized in {self.state} state")

    def command_callback(self, msg):
        """Process commands from Earth with autonomous decision-making"""
        command = msg.data.strip()
        self.get_logger().info(f"📡 Command received: {command}")
        
        # Parse command
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
            self.get_logger().warn(f"⚠️  Unknown command: {command}")

    def handle_start_task(self, task_id):
        """Start a new task if conditions allow"""
        if self.state == self.STATE_SAFE_MODE:
            self.get_logger().warn(f"❌ Cannot start task {task_id}: Rover in SAFE_MODE. Send RESET first.")
            return
        
        if self.state == self.STATE_EXECUTING:
            self.get_logger().warn(f"❌ Cannot start task {task_id}: Already executing task {self.current_task_id}")
            return
        
        # Accept the task
        self.state = self.STATE_EXECUTING
        self.current_task_id = task_id
        self.task_counter = 0
        self.get_logger().info(f"✅ Started task: {task_id}")

    def handle_abort(self):
        """Abort current task if executing"""
        if self.state == self.STATE_EXECUTING:
            self.get_logger().info(f"🛑 Aborting task: {self.current_task_id}")
            self.state = self.STATE_IDLE
            self.current_task_id = None
            self.task_counter = 0
        else:
            self.get_logger().info(f"ℹ️  ABORT received but rover is {self.state}, no task to abort")

    def handle_go_safe(self):
        """Enter safe mode immediately"""
        old_state = self.state
        self.state = self.STATE_SAFE_MODE
        self.current_task_id = None
        self.task_counter = 0
        if not self.last_fault:
            self.last_fault = "Commanded to SAFE_MODE"
        self.get_logger().warn(f"⚠️  Commanded to SAFE_MODE from {old_state}")

    def handle_reset(self):
        """Reset from safe mode to idle"""
        if self.state == self.STATE_SAFE_MODE:
            self.state = self.STATE_IDLE
            self.current_task_id = None
            self.task_counter = 0
            self.last_fault = None  # Clear fault on reset
            self.get_logger().info(f"🔄 RESET: Leaving SAFE_MODE → IDLE")
        else:
            self.get_logger().info(f"ℹ️  RESET received but rover is {self.state}, not in SAFE_MODE")

    def execute_task_step(self):
        """Simulate autonomous task execution with fault detection"""
        if self.state != self.STATE_EXECUTING:
            return
        
        self.task_counter += 1
        
        # Drain battery during execution (0.5% per step)
        self.battery_level = max(0.0, self.battery_level - 0.005)
        
        # Simulate fault detection (realistic autonomy!)
        if random.random() < self.fault_probability:
            fault_msg = f"Fault during task execution (step {self.task_counter})"
            self.get_logger().error(f"🚨 FAULT DETECTED during task {self.current_task_id}!")
            self.state = self.STATE_SAFE_MODE
            self.last_fault = fault_msg
            self.current_task_id = None
            self.task_counter = 0
            return
        
        # Simulate task completion after 10 steps
        if self.task_counter >= 10:
            self.get_logger().info(f"✅ Task {self.current_task_id} completed!")
            self.state = self.STATE_IDLE
            self.current_task_id = None
            self.task_counter = 0
        else:
            self.get_logger().info(f"⚙️  Executing task {self.current_task_id}: step {self.task_counter}/10")

    def publish_telemetry(self):
        """Publish current state as structured JSON"""
        telemetry = {
            "ts": time.time(),
            "state": self.state,
            "task_id": self.current_task_id,
            "battery": round(self.battery_level, 2),
            "fault": self.last_fault
        }
        
        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)

def main():
    rclpy.init()
    node = RoverNode()
    rclpy.spin(node)
    rclpy.shutdown()