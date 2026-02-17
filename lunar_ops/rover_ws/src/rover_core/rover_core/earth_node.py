import json
import re
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .task_model import load_catalog
from .task_model import normalize_task_request
from .task_model import select_best_rover


class EarthNode(Node):
    """Earth station with fleet registry and task assignment logic."""

    def __init__(self):
        super().__init__('earth_node')

        # Runtime configuration
        self.declare_parameter('rover_ids', 'rover_1')
        self.declare_parameter('ack_timeout', 5.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('enable_topic_discovery', True)
        self.declare_parameter('task_catalog_path', '')

        self.ack_timeout = float(self.get_parameter('ack_timeout').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.enable_topic_discovery = bool(
            self.get_parameter('enable_topic_discovery').value
        )
        self.task_catalog = load_catalog(
            str(self.get_parameter('task_catalog_path').value).strip() or None
        )

        # Command tracking
        self.cmd_counter = 0
        self.pending_commands = {}

        # Fleet state registry (rover_id -> state snapshot)
        self.fleet_registry = {}

        # Dynamic interfaces
        self.command_publishers = {}
        self.telemetry_subs = {}
        self.ack_subs = {}

        self._lock = threading.Lock()
        self._telemetry_re = re.compile(r'^/earth/telemetry/([^/]+)$')
        self._ack_re = re.compile(r'^/earth/ack/([^/]+)$')

        # Seed known rover IDs from parameter
        rover_ids_raw = self.get_parameter('rover_ids').value
        for rover_id in self._parse_rover_ids(rover_ids_raw):
            self._ensure_rover_interfaces(rover_id, source='config')

        # Timers
        self.retry_timer = self.create_timer(1.0, self.check_timeouts)
        if self.enable_topic_discovery:
            self.discovery_timer = self.create_timer(
                2.0, self.discover_rovers
            )

        self.get_logger().info('Earth station online (Fleet Command)')
        self.print_help()

    def _parse_rover_ids(self, rover_ids_raw):
        """Parse comma-separated rover IDs from parameter value."""
        if not rover_ids_raw:
            return []
        rover_ids = []
        for rover_id in str(rover_ids_raw).split(','):
            rover_id = rover_id.strip()
            if rover_id:
                rover_ids.append(rover_id)
        return rover_ids

    def _ensure_rover_interfaces(self, rover_id, source='discovery'):
        """Create pub/sub interfaces for a rover if missing."""
        created = False

        if rover_id not in self.command_publishers:
            self.command_publishers[rover_id] = self.create_publisher(
                String, f'/earth/uplink_cmd/{rover_id}', 10
            )
            created = True

        if rover_id not in self.telemetry_subs:
            self.telemetry_subs[rover_id] = self.create_subscription(
                String,
                f'/earth/telemetry/{rover_id}',
                lambda msg, rid=rover_id: self.telemetry_callback(msg, rid),
                10,
            )
            created = True

        if rover_id not in self.ack_subs:
            self.ack_subs[rover_id] = self.create_subscription(
                String,
                f'/earth/ack/{rover_id}',
                lambda msg, rid=rover_id: self.ack_callback(msg, rid),
                10,
            )
            created = True

        if rover_id not in self.fleet_registry:
            self.fleet_registry[rover_id] = {
                'state': 'UNKNOWN',
                'battery': 0.0,
                'solar_exposure': 0.0,
                'task_id': None,
                'active_task_type': None,
                'active_task_difficulty': None,
                'task_total_steps': None,
                'position': None,
                'data_buffer_size': 0,
                'fault': None,
                'last_seen': None,
                'predicted_fault_probability': 0.0,
                'assignment_score_breakdown': None,
                'lunar_time_state': 'UNKNOWN',
                'solar_intensity': 0.0,
                'terrain_difficulty': 0.3,
                'comm_quality': 0.8,
                'thermal_stress': 0.3,
            }
            created = True

        if created:
            self.get_logger().info(
                f'Fleet interface ready for [{rover_id}] via {source}'
            )

    def discover_rovers(self):
        """Discover rover IDs from active telemetry/ack topics."""
        try:
            topic_names_and_types = self.get_topic_names_and_types()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Topic discovery failed: {exc}')
            return

        discovered = set()
        for topic_name, _topic_types in topic_names_and_types:
            telemetry_match = self._telemetry_re.match(topic_name)
            if telemetry_match:
                discovered.add(telemetry_match.group(1))

            ack_match = self._ack_re.match(topic_name)
            if ack_match:
                discovered.add(ack_match.group(1))

        for rover_id in sorted(discovered):
            self._ensure_rover_interfaces(rover_id, source='topic-scan')

    def telemetry_callback(self, msg, subscribed_rover_id):
        """Update fleet registry from rover telemetry."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'Failed to parse telemetry JSON from '
                f'[{subscribed_rover_id}]: {msg.data}'
            )
            return

        rover_id = data.get('rover_id', subscribed_rover_id)
        if rover_id != subscribed_rover_id:
            self._ensure_rover_interfaces(rover_id, source='telemetry')

        with self._lock:
            if rover_id not in self.fleet_registry:
                self.fleet_registry[rover_id] = {
                    'state': 'UNKNOWN',
                    'battery': 0.0,
                    'solar_exposure': 0.0,
                    'task_id': None,
                    'active_task_type': None,
                    'active_task_difficulty': None,
                    'task_total_steps': None,
                    'position': None,
                    'data_buffer_size': 0,
                    'fault': None,
                    'last_seen': None,
                    'predicted_fault_probability': 0.0,
                    'assignment_score_breakdown': None,
                    'lunar_time_state': 'UNKNOWN',
                    'solar_intensity': 0.0,
                    'terrain_difficulty': 0.3,
                    'comm_quality': 0.8,
                    'thermal_stress': 0.3,
                }

            self.fleet_registry[rover_id].update({
                'state': data.get('state', 'UNKNOWN'),
                'battery': float(data.get('battery', 0.0)),
                'solar_exposure': float(data.get('solar_exposure', 0.0)),
                'task_id': data.get('task_id'),
                'active_task_type': (
                    data.get('active_task_type') or data.get('task_type')
                ),
                'active_task_difficulty': (
                    data.get('active_task_difficulty')
                    or data.get('difficulty_level')
                ),
                'task_total_steps': data.get('task_total_steps'),
                'position': data.get('position'),
                'data_buffer_size': int(data.get('data_buffer_size', 0)),
                'fault': data.get('fault'),
                'last_seen': time.time(),
                'predicted_fault_probability': float(
                    data.get('predicted_fault_probability', 0.0)
                ),
                'assignment_score_breakdown': data.get(
                    'assignment_score_breakdown'
                ),
                'lunar_time_state': data.get('lunar_time_state', 'UNKNOWN'),
                'solar_intensity': float(
                    data.get('solar_intensity', data.get('solar_exposure', 0.0))
                ),
                'terrain_difficulty': float(data.get('terrain_difficulty', 0.3)),
                'comm_quality': float(data.get('comm_quality', 0.8)),
                'thermal_stress': float(data.get('thermal_stress', 0.3)),
            })

    def ack_callback(self, msg, subscribed_rover_id):
        """Handle ACK responses from rovers."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'Failed to parse ACK JSON from '
                f'[{subscribed_rover_id}]: {msg.data}'
            )
            return

        ack_id = data.get('ack_id')
        status = data.get('status')
        reason = data.get('reason')
        ack_rover_id = data.get('rover_id', subscribed_rover_id)

        with self._lock:
            cmd_info = self.pending_commands.pop(ack_id, None)

        if cmd_info is None:
            reason_suffix = f' ({reason})' if reason else ''
            print(
                f'ACK {ack_id} from [{ack_rover_id}]: '
                f'{status}{reason_suffix} (untracked)'
            )
            return

        rtt = time.time() - cmd_info['sent_at']
        if status == 'ACCEPTED':
            print(
                f'ACK {ack_id} [{ack_rover_id}] ACCEPTED '
                f'(RTT: {rtt:.2f}s)'
            )
        else:
            reason_suffix = f' ({reason})' if reason else ''
            print(
                f'ACK {ack_id} [{ack_rover_id}] REJECTED'
                f'{reason_suffix} (RTT: {rtt:.2f}s)'
            )

    def check_timeouts(self):
        """Retry or fail commands that exceeded ACK timeout."""
        current_time = time.time()

        with self._lock:
            timed_out = [
                (cmd_id, cmd_info)
                for cmd_id, cmd_info in self.pending_commands.items()
                if current_time - cmd_info['sent_at'] > self.ack_timeout
            ]

        for cmd_id, cmd_info in timed_out:
            rover_id = cmd_info['rover_id']
            attempt = cmd_info['attempt']

            if attempt < self.max_retries:
                next_attempt = attempt + 1
                print(
                    f'No ACK for {cmd_id} [{rover_id}], retrying '
                    f'({next_attempt}/{self.max_retries})'
                )

                publisher = self.command_publishers.get(rover_id)
                if publisher is None:
                    self._ensure_rover_interfaces(rover_id, source='retry')
                    publisher = self.command_publishers.get(rover_id)

                if publisher is None:
                    self.get_logger().error(
                        f'Cannot retry {cmd_id}: publisher missing '
                        f'for [{rover_id}]'
                    )
                    continue

                msg = String()
                msg.data = cmd_info['cmd_json']
                publisher.publish(msg)

                with self._lock:
                    if cmd_id in self.pending_commands:
                        self.pending_commands[cmd_id]['sent_at'] = time.time()
                        self.pending_commands[cmd_id]['attempt'] = next_attempt
            else:
                print(
                    f'Command {cmd_id} [{rover_id}] failed after '
                    f'{self.max_retries} attempts'
                )
                with self._lock:
                    self.pending_commands.pop(cmd_id, None)

    def select_best_rover(self, available_rovers, task_request):
        """Pick best rover with explainable context-aware assignment scoring."""
        selection = select_best_rover(
            self.task_catalog, available_rovers, task_request
        )
        return selection

    def auto_assign_task(self, task_id, task_type=None, difficulty_level='L2',
                         mission_phase=None, required_capabilities=None,
                         target_site=None):
        """Auto-assign a task to the best available rover."""
        task_request = normalize_task_request(
            self.task_catalog,
            task_id=task_id,
            task_type=task_type,
            difficulty_level=difficulty_level,
            mission_phase=mission_phase,
            required_capabilities=required_capabilities,
            target_site=target_site,
        )

        with self._lock:
            available = {
                rover_id: status.copy()
                for rover_id, status in self.fleet_registry.items()
            }

        selection = self.select_best_rover(available, task_request)
        best_rover = selection.get('selected_rover')
        if best_rover is None:
            print(
                f'REJECTED: no suitable rover for task {task_request["task_id"]} '
                f'({selection.get("reject_reason")})'
            )
            return False

        selected_candidate = selection['scored_candidates'][0]
        score_text = json.dumps(selected_candidate['score_breakdown'])
        print(
            f'Auto-assigned task {task_request["task_id"]} '
            f'({task_request["task_type"]}/{task_request["difficulty_level"]}) '
            f'-> [{best_rover}] score={selected_candidate["score"]:.3f} '
            f'risk={selected_candidate["predicted_fault_probability"]:.3f}'
        )
        print(f'Assignment breakdown [{best_rover}]: {score_text}')
        self.send_command(
            best_rover,
            'START_TASK',
            task_id=task_request['task_id'],
            task_request=task_request,
            assignment_context={
                'selected_rover': best_rover,
                'score_breakdown': selected_candidate['score_breakdown'],
                'predicted_fault_probability': selected_candidate[
                    'predicted_fault_probability'
                ],
                'reject_reason': None,
            },
        )
        return True

    def assign_task(self, rover_id, task_id, task_type=None,
                    difficulty_level='L2', mission_phase=None,
                    required_capabilities=None, target_site=None):
        """Manual assignment command for a specific rover."""
        self._ensure_rover_interfaces(rover_id, source='manual-assign')
        task_request = normalize_task_request(
            self.task_catalog,
            task_id=task_id,
            task_type=task_type,
            difficulty_level=difficulty_level,
            mission_phase=mission_phase,
            required_capabilities=required_capabilities,
            target_site=target_site,
        )
        print(
            f'Manual assignment: {task_request["task_id"]} '
            f'({task_request["task_type"]}/{task_request["difficulty_level"]}) '
            f'-> [{rover_id}]'
        )
        self.send_command(
            rover_id,
            'START_TASK',
            task_id=task_request['task_id'],
            task_request=task_request,
        )

    def send_command(self, rover_id, cmd_type, task_id=None, task_request=None,
                     assignment_context=None):
        """Send JSON command to a target rover and track ACK state."""
        self._ensure_rover_interfaces(rover_id, source='send-command')
        publisher = self.command_publishers.get(rover_id)
        if publisher is None:
            self.get_logger().error(
                f'Cannot send {cmd_type}: publisher unavailable '
                f'for [{rover_id}]'
            )
            return

        with self._lock:
            self.cmd_counter += 1
            cmd_id = f'c-{self.cmd_counter:05d}'

        cmd_data = {
            'cmd_id': cmd_id,
            'type': cmd_type,
            'ts': time.time(),
        }
        if task_id:
            cmd_data['task_id'] = task_id
        if task_request:
            cmd_data.update({
                'task_type': task_request['task_type'],
                'difficulty_level': task_request['difficulty_level'],
                'required_capabilities': task_request['required_capabilities'],
                'mission_phase': task_request['mission_phase'],
                'target_site': task_request.get('target_site'),
            })
        if assignment_context:
            cmd_data['assignment_score_breakdown'] = assignment_context.get(
                'score_breakdown'
            )
            cmd_data['predicted_fault_probability'] = assignment_context.get(
                'predicted_fault_probability'
            )
            cmd_data['selected_rover'] = assignment_context.get('selected_rover')
            cmd_data['reject_reason'] = assignment_context.get('reject_reason')

        cmd_json = json.dumps(cmd_data)
        msg = String()
        msg.data = cmd_json
        publisher.publish(msg)

        with self._lock:
            self.pending_commands[cmd_id] = {
                'sent_at': time.time(),
                'cmd_type': cmd_type,
                'attempt': 1,
                'cmd_json': cmd_json,
                'rover_id': rover_id,
                'task_id': task_id,
                'task_request': task_request,
                'assignment_context': assignment_context,
            }

        task_suffix = f' {task_id}' if task_id else ''
        print(f'Sent {cmd_id} -> [{rover_id}] {cmd_type}{task_suffix}')

    def get_default_rover(self, action_name):
        """Pick a default rover when there is exactly one known rover."""
        with self._lock:
            known_rovers = sorted(self.command_publishers.keys())

        if len(known_rovers) == 1:
            return known_rovers[0]

        print(
            f'Usage: {action_name.lower()} <rover_id>. '
            f'Known rovers: {", ".join(known_rovers) or "none"}'
        )
        return None

    def print_fleet(self):
        """Print current fleet registry snapshot."""
        with self._lock:
            if not self.fleet_registry:
                print('Fleet registry is empty')
                return

            print('\nFleet registry:')
            for rover_id in sorted(self.fleet_registry):
                status = self.fleet_registry[rover_id]
                state = status.get('state', 'UNKNOWN')
                battery = status.get('battery', 0.0)
                solar = status.get('solar_exposure', 0.0)
                task_id = status.get('task_id')
                print(
                    f'  {rover_id}: state={state}, '
                    f'battery={battery:.2f}, solar={solar:.2f}, '
                    f'task={task_id}'
                )
            print('')

    def print_help(self):
        """Print available Earth station commands."""
        print('\n' + '=' * 66)
        print('EARTH STATION FLEET COMMAND INTERFACE')
        print('=' * 66)
        print('Commands:')
        print('  start <task_id> [task_type] [difficulty]')
        print('      Auto-select best rover using capability/risk scoring')
        print('  assign_task <rover_id> <task_id> [task_type] [difficulty]')
        print('      Manual START_TASK to specific rover')
        print('  abort <rover_id>')
        print('  safe <rover_id>')
        print('  reset <rover_id>')
        print('  fleet')
        print('      Show tracked rover states')
        print('  help')
        print('  quit')
        print('=' * 66 + '\n')


def command_loop(node):
    """Interactive Earth station command loop."""
    while rclpy.ok():
        try:
            user_input = input('Earth> ').strip()
            if not user_input:
                continue

            parts = user_input.split()
            cmd = parts[0].upper()

            if cmd in {'QUIT', 'EXIT'}:
                print('Shutting down Earth station...')
                rclpy.shutdown()
                break

            if cmd == 'HELP':
                node.print_help()
                continue

            if cmd == 'FLEET':
                node.print_fleet()
                continue

            if cmd == 'START':
                split_parts = user_input.split(maxsplit=3)
                if len(split_parts) < 2:
                    print('Usage: start <task_id> [task_type] [difficulty]')
                    continue
                task_id = split_parts[1]
                task_type = split_parts[2] if len(split_parts) >= 3 else None
                difficulty = split_parts[3] if len(split_parts) >= 4 else 'L2'
                node.auto_assign_task(task_id, task_type, difficulty)
                continue

            if cmd in {'ASSIGN_TASK', 'ASSIGN'}:
                split_parts = user_input.split(maxsplit=4)
                if len(split_parts) < 3:
                    print(
                        'Usage: assign_task <rover_id> <task_id> '
                        '[task_type] [difficulty]'
                    )
                    continue
                rover_id = split_parts[1]
                task_id = split_parts[2]
                task_type = split_parts[3] if len(split_parts) >= 4 else None
                difficulty = split_parts[4] if len(split_parts) >= 5 else 'L2'
                node.assign_task(rover_id, task_id, task_type, difficulty)
                continue

            if cmd in {'ABORT', 'SAFE', 'RESET'}:
                rover_id = None
                if len(parts) >= 2:
                    rover_id = parts[1]
                else:
                    rover_id = node.get_default_rover(cmd)

                if rover_id is None:
                    continue

                rover_cmd = {
                    'ABORT': 'ABORT',
                    'SAFE': 'GO_SAFE',
                    'RESET': 'RESET',
                }[cmd]
                node.send_command(rover_id, rover_cmd)
                continue

            print(f'Unknown command: {cmd}. Type help for options.')

        except EOFError:
            print('\nShutting down Earth station...')
            rclpy.shutdown()
            break
        except KeyboardInterrupt:
            print('\nShutting down Earth station...')
            rclpy.shutdown()
            break


def main():
    """Entry point for the earth node."""
    rclpy.init()
    node = EarthNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    command_loop(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
