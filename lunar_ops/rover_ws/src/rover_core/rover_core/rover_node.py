import json
import math
import random
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .task_model import compute_fault_probability
from .task_model import compute_task_duration_steps
from .task_model import get_lunar_time_state
from .task_model import get_rover_capabilities
from .task_model import load_catalog
from .task_model import normalize_task_request


class RoverNode(Node):
    """Autonomous rover with context-aware task execution model."""

    STATE_IDLE = 'IDLE'
    STATE_EXECUTING = 'EXECUTING'
    STATE_SAFE_MODE = 'SAFE_MODE'
    STATE_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('rover_node')

        self.declare_parameter('rover_id', 'rover_1')
        self.declare_parameter('task_catalog_path', '')

        self.rover_id = self.get_parameter('rover_id').value
        self.task_catalog = load_catalog(
            str(self.get_parameter('task_catalog_path').value).strip() or None
        )

        self.state = self.STATE_IDLE
        self.current_task_id = None
        self.active_task_type = None
        self.active_task_difficulty = None
        self.active_task_mission_phase = None
        self.active_task_required_capabilities = []
        self.assignment_score_breakdown = None
        self.task_total_steps = 0
        self.task_counter = 0
        self.predicted_fault_probability = 0.0
        self.battery_level = 1.0
        self.last_fault = None
        self.capabilities = sorted(get_rover_capabilities(self.rover_id))

        self.position = {
            'lat': round(random.uniform(-45.0, 45.0), 6),
            'lon': round(random.uniform(-180.0, 180.0), 6),
        }
        self.solar_exposure = round(random.uniform(0.6, 1.0), 2)
        self._solar_phase = random.uniform(0, 2 * math.pi)
        self.lunar_time_state = get_lunar_time_state(self.solar_exposure)
        self.terrain_difficulty = round(random.uniform(0.2, 0.65), 2)
        self.comm_quality = round(random.uniform(0.72, 0.94), 2)
        self.thermal_stress = round(random.uniform(0.15, 0.45), 2)
        self.data_buffer_size = 0

        topic_prefix = f'/rover/{self.rover_id}'
        self.telemetry_pub = self.create_publisher(
            String,
            f'{topic_prefix}/downlink_telemetry',
            10,
        )
        self.ack_pub = self.create_publisher(String, f'{topic_prefix}/ack', 10)
        self.command_sub = self.create_subscription(
            String,
            f'{topic_prefix}/command',
            self.command_callback,
            10,
        )

        self.timer = self.create_timer(2.0, self.publish_telemetry)
        self.task_timer = self.create_timer(1.0, self.execute_task_step)

        self.get_logger().info(
            f'Rover [{self.rover_id}] initialized in {self.state} state\n'
            f'Capabilities={self.capabilities}\n'
            f'Topics: {topic_prefix}/{{downlink_telemetry,command,ack}}'
        )

    def _clear_active_task(self):
        self.current_task_id = None
        self.active_task_type = None
        self.active_task_difficulty = None
        self.active_task_mission_phase = None
        self.active_task_required_capabilities = []
        self.assignment_score_breakdown = None
        self.task_total_steps = 0
        self.task_counter = 0
        self.predicted_fault_probability = 0.0

    def command_callback(self, msg):
        """Process JSON commands from Earth with autonomous decision-making."""
        cmd_data = {}
        try:
            cmd_data = json.loads(msg.data)
            cmd_id = cmd_data.get('cmd_id', 'unknown')
            cmd_type = cmd_data.get('type')

            self.get_logger().info(
                f'[{self.rover_id}] Command received [{cmd_id}]: {cmd_type}'
            )

            if cmd_type == 'START_TASK':
                success, reason = self.handle_start_task(
                    task_id=cmd_data.get('task_id'),
                    task_type=cmd_data.get('task_type'),
                    difficulty_level=cmd_data.get('difficulty_level'),
                    mission_phase=cmd_data.get('mission_phase'),
                    required_capabilities=cmd_data.get('required_capabilities'),
                    target_site=cmd_data.get('target_site'),
                    assignment_score_breakdown=cmd_data.get(
                        'assignment_score_breakdown'
                    ),
                    predicted_fault_probability=cmd_data.get(
                        'predicted_fault_probability'
                    ),
                )
                self.send_ack(cmd_id, success, reason)
            elif cmd_type == 'ABORT':
                success, reason = self.handle_abort()
                self.send_ack(cmd_id, success, reason)
            elif cmd_type == 'GO_SAFE':
                success, reason = self.handle_go_safe()
                self.send_ack(cmd_id, success, reason)
            elif cmd_type == 'RESET':
                success, reason = self.handle_reset()
                self.send_ack(cmd_id, success, reason)
            else:
                self.send_ack(cmd_id, False, f'Unknown command type: {cmd_type}')

        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Failed to parse command JSON: {msg.data}')
            self.send_ack('unknown', False, f'Invalid JSON: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Error processing command: {exc}')
            self.send_ack(cmd_data.get('cmd_id', 'unknown'), False, f'Error: {exc}')

    def handle_start_task(
        self,
        task_id,
        task_type=None,
        difficulty_level=None,
        mission_phase=None,
        required_capabilities=None,
        target_site=None,
        assignment_score_breakdown=None,
        predicted_fault_probability=None,
    ):
        """Start a new task if conditions allow."""
        if self.state == self.STATE_SAFE_MODE:
            reason = 'Cannot start task: Rover in SAFE_MODE. Send RESET first.'
            self.get_logger().warn(reason)
            return False, reason

        if self.state == self.STATE_EXECUTING:
            reason = (
                f'Cannot start task: Already executing task {self.current_task_id}'
            )
            self.get_logger().warn(reason)
            return False, reason

        task_request = normalize_task_request(
            self.task_catalog,
            task_id=task_id,
            task_type=task_type,
            difficulty_level=difficulty_level,
            mission_phase=mission_phase,
            required_capabilities=required_capabilities,
            target_site=target_site,
        )

        required_caps = set(task_request['required_capabilities'])
        capability_set = set(self.capabilities)
        if not required_caps.issubset(capability_set):
            reason = (
                f'Cannot start task: Missing capabilities for '
                f'{task_request["task_type"]}'
            )
            self.get_logger().warn(reason)
            return False, reason

        duration_context = {
            'terrain_difficulty': self.terrain_difficulty,
            'comm_quality': self.comm_quality,
        }
        self.task_total_steps = compute_task_duration_steps(
            self.task_catalog,
            task_request['task_type'],
            task_request['difficulty_level'],
            duration_context,
        )

        risk_context = {
            'battery': self.battery_level,
            'solar_intensity': self.solar_exposure,
            'terrain_difficulty': self.terrain_difficulty,
            'comm_quality': self.comm_quality,
            'thermal_stress': self.thermal_stress,
            'lunar_time_state': self.lunar_time_state,
            'capability_match': True,
        }
        computed_risk, _ = compute_fault_probability(
            self.task_catalog,
            task_request['task_type'],
            task_request['difficulty_level'],
            risk_context,
        )

        self.state = self.STATE_EXECUTING
        self.current_task_id = task_request['task_id']
        self.active_task_type = task_request['task_type']
        self.active_task_difficulty = task_request['difficulty_level']
        self.active_task_mission_phase = task_request['mission_phase']
        self.active_task_required_capabilities = list(
            task_request['required_capabilities']
        )
        self.assignment_score_breakdown = assignment_score_breakdown
        self.task_counter = 0
        self.predicted_fault_probability = (
            float(predicted_fault_probability)
            if predicted_fault_probability is not None
            else computed_risk
        )

        self.get_logger().info(
            f'[{self.rover_id}] Started task: {self.current_task_id} '
            f'({self.active_task_type}/{self.active_task_difficulty}, '
            f'steps={self.task_total_steps}, '
            f'risk={self.predicted_fault_probability:.3f})'
        )
        return True, None

    def handle_abort(self):
        """Abort current task if executing."""
        if self.state == self.STATE_EXECUTING:
            self.get_logger().info(
                f'[{self.rover_id}] Aborting task: {self.current_task_id}'
            )
            self.state = self.STATE_IDLE
            self._clear_active_task()
        return True, None

    def handle_go_safe(self):
        """Enter safe mode immediately."""
        old_state = self.state
        self.state = self.STATE_SAFE_MODE
        self._clear_active_task()
        if not self.last_fault:
            self.last_fault = 'Commanded to SAFE_MODE'
        self.get_logger().warn(
            f'[{self.rover_id}] Commanded to SAFE_MODE from {old_state}'
        )
        return True, None

    def handle_reset(self):
        """Reset from safe mode to idle."""
        if self.state == self.STATE_SAFE_MODE:
            self.state = self.STATE_IDLE
            self._clear_active_task()
            self.last_fault = None
            self.get_logger().info(
                f'[{self.rover_id}] RESET: Leaving SAFE_MODE -> IDLE'
            )
        return True, None

    def send_ack(self, cmd_id, success, reason=None):
        """Send command acknowledgment to Earth."""
        ack_data = {
            'ack_id': cmd_id,
            'rover_id': self.rover_id,
            'status': 'ACCEPTED' if success else 'REJECTED',
            'reason': reason,
            'ts': time.time(),
        }

        msg = String()
        msg.data = json.dumps(ack_data)
        self.ack_pub.publish(msg)

    def _update_solar_exposure(self):
        self._solar_phase += 0.02
        base = 0.5 + 0.5 * math.sin(self._solar_phase)
        noise = random.uniform(-0.05, 0.05)
        self.solar_exposure = round(max(0.0, min(1.0, base + noise)), 2)
        self.lunar_time_state = get_lunar_time_state(self.solar_exposure)

    def _update_environmentals(self):
        self.comm_quality = round(
            max(0.3, min(1.0, self.comm_quality + random.uniform(-0.03, 0.02))),
            2,
        )
        self.terrain_difficulty = round(
            max(
                0.1,
                min(0.95, self.terrain_difficulty + random.uniform(-0.02, 0.02)),
            ),
            2,
        )
        thermal_delta = 0.04 if self.lunar_time_state == 'DAYLIGHT' else -0.02
        self.thermal_stress = round(
            max(
                0.05,
                min(
                    0.98,
                    self.thermal_stress
                    + thermal_delta
                    + random.uniform(-0.03, 0.03),
                ),
            ),
            2,
        )

    def execute_task_step(self):
        """Simulate autonomous task execution with dynamic fault modeling."""
        if self.state != self.STATE_EXECUTING:
            return

        self._update_solar_exposure()
        self._update_environmentals()
        self.task_counter += 1

        difficulty_cfg = self.task_catalog['difficulty_levels'].get(
            self.active_task_difficulty or 'L2',
            {},
        )
        base_drain = float(difficulty_cfg.get('energy_drain_per_step', 0.003))
        drain_multiplier = 1.0 + (self.terrain_difficulty * 0.4)
        self.battery_level = max(
            0.0,
            self.battery_level - (base_drain * drain_multiplier),
        )

        self.data_buffer_size += random.randint(128, 1024)
        self.position['lat'] += random.uniform(-0.0001, 0.0001)
        self.position['lon'] += random.uniform(-0.0001, 0.0001)
        self.position['lat'] = round(self.position['lat'], 6)
        self.position['lon'] = round(self.position['lon'], 6)

        risk_context = {
            'battery': self.battery_level,
            'solar_intensity': self.solar_exposure,
            'terrain_difficulty': self.terrain_difficulty,
            'comm_quality': self.comm_quality,
            'thermal_stress': self.thermal_stress,
            'lunar_time_state': self.lunar_time_state,
            'capability_match': True,
        }
        fault_probability, _ = compute_fault_probability(
            self.task_catalog,
            self.active_task_type or 'movement',
            self.active_task_difficulty or 'L2',
            risk_context,
        )
        self.predicted_fault_probability = fault_probability

        if random.random() < fault_probability:
            fault_msg = f'Fault during task execution (step {self.task_counter})'
            self.get_logger().error(
                f'[{self.rover_id}] FAULT DETECTED during task {self.current_task_id}'
            )
            self.state = self.STATE_SAFE_MODE
            self.last_fault = fault_msg
            self._clear_active_task()
            return

        if self.task_counter >= max(1, self.task_total_steps):
            self.get_logger().info(
                f'[{self.rover_id}] Task {self.current_task_id} completed'
            )
            self.state = self.STATE_IDLE
            self._clear_active_task()
            self.data_buffer_size = 0
        else:
            self.get_logger().info(
                f'[{self.rover_id}] Executing task {self.current_task_id}: '
                f'step {self.task_counter}/{self.task_total_steps}'
            )

    def publish_telemetry(self):
        """Publish current state as structured JSON with enriched fields."""
        self._update_solar_exposure()

        telemetry = {
            'ts': time.time(),
            'rover_id': self.rover_id,
            'state': self.state,
            'task_id': self.current_task_id,
            'active_task_type': self.active_task_type,
            'active_task_difficulty': self.active_task_difficulty,
            'task_total_steps': (
                self.task_total_steps if self.state == self.STATE_EXECUTING else None
            ),
            'battery': round(self.battery_level, 2),
            'position': {
                'lat': self.position['lat'],
                'lon': self.position['lon'],
            },
            'solar_exposure': self.solar_exposure,
            'solar_intensity': self.solar_exposure,
            'lunar_time_state': self.lunar_time_state,
            'terrain_difficulty': self.terrain_difficulty,
            'comm_quality': self.comm_quality,
            'thermal_stress': self.thermal_stress,
            'predicted_fault_probability': round(
                self.predicted_fault_probability, 4
            ),
            'assignment_score_breakdown': self.assignment_score_breakdown,
            'data_buffer_size': self.data_buffer_size,
            'fault': self.last_fault,
            'capabilities': self.capabilities,
        }

        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)


def main():
    rclpy.init()
    node = RoverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
