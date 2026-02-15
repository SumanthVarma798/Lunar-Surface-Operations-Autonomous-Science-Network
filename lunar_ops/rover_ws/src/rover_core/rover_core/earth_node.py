import json
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class EarthNode(Node):
    """Earth station - command interface with ACK tracking and retry logic."""

    def __init__(self):
        super().__init__('earth_node')

        # Command tracking
        self.cmd_counter = 0
        self.pending_commands = {}

        # Configuration
        self.ack_timeout = 5.0
        self.max_retries = 3

        # Publishers
        self.command_pub = self.create_publisher(
            String,
            '/earth/uplink_cmd',
            10
        )

        # Subscribers
        self.ack_sub = self.create_subscription(
            String,
            '/earth/ack',
            self.ack_callback,
            10
        )

        # Timer for checking ACK timeouts
        self.retry_timer = self.create_timer(1.0, self.check_timeouts)

        self.get_logger().info('🌍 Earth station online (Command Interface)')
        self.print_help()

    def ack_callback(self, msg):
        """Handle ACK responses from rover."""
        try:
            data = json.loads(msg.data)
            ack_id = data['ack_id']
            status = data['status']
            reason = data.get('reason')

            if ack_id in self.pending_commands:
                cmd_info = self.pending_commands[ack_id]
                rtt = time.time() - cmd_info['sent_at']

                if status == 'ACCEPTED':
                    print(f'✅ ACK {ack_id}: ACCEPTED (RTT: {rtt:.2f}s)')
                else:
                    reason_str = f' ({reason})' if reason else ''
                    print(f'❌ ACK {ack_id}: REJECTED{reason_str}')

                del self.pending_commands[ack_id]
            else:
                if status == 'ACCEPTED':
                    print(f'✅ ACK {ack_id}: ACCEPTED')
                else:
                    reason_str = f' ({reason})' if reason else ''
                    print(f'❌ ACK {ack_id}: REJECTED{reason_str}')

        except json.JSONDecodeError:
            self.get_logger().warn(
                f'Failed to parse ACK JSON: {msg.data}'
            )
        except Exception as e:
            self.get_logger().error(f'Error processing ACK: {e}')

    def check_timeouts(self):
        """Check for commands that have not received ACKs and retry."""
        current_time = time.time()
        timed_out = []

        for cmd_id, cmd_info in self.pending_commands.items():
            elapsed = current_time - cmd_info['sent_at']

            if elapsed > self.ack_timeout:
                timed_out.append(cmd_id)

        for cmd_id in timed_out:
            cmd_info = self.pending_commands[cmd_id]

            if cmd_info['attempt'] < self.max_retries:
                attempt = cmd_info['attempt'] + 1
                print(
                    f'⏰ No ACK for {cmd_id}, retrying '
                    f'(attempt {attempt}/{self.max_retries})'
                )

                msg = String()
                msg.data = cmd_info['cmd_json']
                self.command_pub.publish(msg)

                cmd_info['sent_at'] = time.time()
                cmd_info['attempt'] = attempt
            else:
                print(
                    f'❌ Command {cmd_id} failed after '
                    f'{self.max_retries} attempts'
                )
                del self.pending_commands[cmd_id]

    def send_command(self, cmd_type, task_id=None):
        """Send command to rover with JSON format and tracking."""
        self.cmd_counter += 1
        cmd_id = f'c-{self.cmd_counter:05d}'

        cmd_data = {
            'cmd_id': cmd_id,
            'type': cmd_type,
            'ts': time.time()
        }

        if task_id:
            cmd_data['task_id'] = task_id

        cmd_json = json.dumps(cmd_data)

        msg = String()
        msg.data = cmd_json
        self.command_pub.publish(msg)

        self.pending_commands[cmd_id] = {
            'sent_at': time.time(),
            'cmd_type': cmd_type,
            'attempt': 1,
            'cmd_json': cmd_json
        }

        if task_id:
            print(f'📤 Sent {cmd_id}: {cmd_type} {task_id}')
        else:
            print(f'📤 Sent {cmd_id}: {cmd_type}')

    def print_help(self):
        """Print available commands."""
        print('\n' + '=' * 60)
        print('🌍 EARTH STATION COMMAND INTERFACE')
        print('=' * 60)
        print('Available commands:')
        print('  start <task_id>  - Start a new task '
              '(e.g., start SAMPLE-001)')
        print('  abort            - Abort current task')
        print('  safe             - Put rover in safe mode')
        print('  reset            - Reset from safe mode to idle')
        print('  help             - Show this help')
        print('  quit             - Exit')
        print('=' * 60)
        print('Note: Telemetry display is in a separate terminal window')
        print('      Run: make test-telemetry to view telemetry stream')
        print('=' * 60 + '\n')


def command_loop(node):
    """Interactive command loop."""
    while rclpy.ok():
        try:
            user_input = input('Earth> ').strip()

            if not user_input:
                continue

            parts = user_input.split(maxsplit=1)
            cmd = parts[0].lower()

            if cmd == 'quit' or cmd == 'exit':
                print('Shutting down Earth station...')
                rclpy.shutdown()
                break
            elif cmd == 'help':
                node.print_help()
            elif cmd == 'start':
                if len(parts) < 2:
                    print('❌ Usage: start <task_id>')
                else:
                    task_id = parts[1]
                    node.send_command('START_TASK', task_id)
            elif cmd == 'abort':
                node.send_command('ABORT')
            elif cmd == 'safe':
                node.send_command('GO_SAFE')
            elif cmd == 'reset':
                node.send_command('RESET')
            else:
                print(
                    f'❌ Unknown command: {cmd}. '
                    'Type help for options.'
                )

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

    # Run ROS2 spinning in a separate thread
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    # Run command loop in main thread
    command_loop(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
