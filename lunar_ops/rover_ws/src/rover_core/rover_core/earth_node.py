import json
import re
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EarthNode(Node):
    """Earth station with fleet registry and task assignment logic."""

    def __init__(self):
        super().__init__('earth_node')

        # Runtime configuration
        self.declare_parameter('rover_ids', 'rover_1')
        self.declare_parameter('ack_timeout', 5.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('enable_topic_discovery', True)

        self.ack_timeout = float(self.get_parameter('ack_timeout').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.enable_topic_discovery = bool(
            self.get_parameter('enable_topic_discovery').value
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
                'position': None,
                'data_buffer_size': 0,
                'fault': None,
                'last_seen': None,
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
                    'position': None,
                    'data_buffer_size': 0,
                    'fault': None,
                    'last_seen': None,
                }

            self.fleet_registry[rover_id].update({
                'state': data.get('state', 'UNKNOWN'),
                'battery': float(data.get('battery', 0.0)),
                'solar_exposure': float(data.get('solar_exposure', 0.0)),
                'task_id': data.get('task_id'),
                'position': data.get('position'),
                'data_buffer_size': int(data.get('data_buffer_size', 0)),
                'fault': data.get('fault'),
                'last_seen': time.time(),
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

    def select_best_rover(self, available_rovers):
        """Pick best rover using IDLE + battery/solar scoring."""
        scores = {}
        for rover_id, status in available_rovers.items():
            if status.get('state') != 'IDLE':
                continue

            battery = float(status.get('battery', 0.0))
            solar_exposure = float(status.get('solar_exposure', 0.0))
            solar_bonus = 1.0 if solar_exposure >= 0.5 else 0.3
            score = battery * 0.5 + solar_bonus
            scores[rover_id] = score

        if not scores:
            return None
        return max(scores, key=scores.get)

    def auto_assign_task(self, task_id):
        """Auto-assign a task to the best available rover."""
        with self._lock:
            available = {
                rover_id: status.copy()
                for rover_id, status in self.fleet_registry.items()
                if status.get('state') == 'IDLE'
            }

        best_rover = self.select_best_rover(available)
        if best_rover is None:
            print(
                f'REJECTED: no suitable rover for task {task_id} '
                '(need IDLE rover with telemetry)'
            )
            return False

        print(f'Auto-assigned task {task_id} -> [{best_rover}]')
        self.send_command(best_rover, 'START_TASK', task_id)
        return True

    def assign_task(self, rover_id, task_id):
        """Manual assignment command for a specific rover."""
        self._ensure_rover_interfaces(rover_id, source='manual-assign')
        print(f'Manual assignment: {task_id} -> [{rover_id}]')
        self.send_command(rover_id, 'START_TASK', task_id)

    def send_command(self, rover_id, cmd_type, task_id=None):
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
        print('  start <task_id>')
        print('      Auto-select best rover (IDLE + battery/solar score)')
        print('  assign_task <rover_id> <task_id>')
        print('      Manual ASSIGN_TASK to specific rover')
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
                split_parts = user_input.split(maxsplit=1)
                if len(split_parts) < 2:
                    print('Usage: start <task_id>')
                    continue
                task_id = split_parts[1]
                node.auto_assign_task(task_id)
                continue

            if cmd in {'ASSIGN_TASK', 'ASSIGN'}:
                split_parts = user_input.split(maxsplit=2)
                if len(split_parts) < 3:
                    print('Usage: assign_task <rover_id> <task_id>')
                    continue
                rover_id = split_parts[1]
                task_id = split_parts[2]
                node.assign_task(rover_id, task_id)
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
