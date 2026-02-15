import json
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TelemetryMonitor(Node):
    """Telemetry Monitor - dedicated display for rover telemetry and ACKs.

    Runs in a separate terminal from the command interface.
    """

    def __init__(self):
        super().__init__('telemetry_monitor')

        # Subscribe to telemetry stream
        self.telemetry_sub = self.create_subscription(
            String,
            '/earth/telemetry',
            self.telemetry_callback,
            10
        )

        # Subscribe to ACKs
        self.ack_sub = self.create_subscription(
            String,
            '/earth/ack',
            self.ack_callback,
            10
        )

        self.print_header()

    def print_header(self):
        """Print monitor header."""
        print('\n' + '━' * 70)
        print('🌍 TELEMETRY MONITOR - Earth Station')
        print('━' * 70)
        print('Listening on /earth/telemetry and /earth/ack')
        print('━' * 70 + '\n')

    def telemetry_callback(self, msg):
        """Display telemetry from rover."""
        try:
            data = json.loads(msg.data)

            # Format timestamp
            ts = datetime.fromtimestamp(data['ts']).strftime('%H:%M:%S')

            # Color-code state
            state = data['state']
            state_icons = {
                'IDLE': '🟢',
                'EXECUTING': '🔵',
                'SAFE_MODE': '🟡',
                'ERROR': '🔴'
            }
            icon = state_icons.get(state, '⚪')

            # Format battery
            battery = data['battery']
            battery_pct = f'{int(battery * 100)}%'
            if battery < 0.2:
                battery_display = f'🔋❗{battery_pct}'
            elif battery < 0.5:
                battery_display = f'🔋⚠️ {battery_pct}'
            else:
                battery_display = f'🔋 {battery_pct}'

            # Build one-liner
            parts = [f'{ts}', f'{icon} {state}', battery_display]

            if data['task_id']:
                parts.append(f'📋 {data["task_id"]}')

            if data['fault']:
                parts.append(f'⚠️  {data["fault"]}')

            telemetry_line = ' | '.join(parts)
            print(f'📡 {telemetry_line}')

        except json.JSONDecodeError:
            self.get_logger().warn(
                f'Failed to parse telemetry JSON: {msg.data}'
            )
        except Exception as e:
            self.get_logger().error(
                f'Error processing telemetry: {e}'
            )

    def ack_callback(self, msg):
        """Display command acknowledgments."""
        try:
            data = json.loads(msg.data)

            ack_id = data['ack_id']
            status = data['status']
            reason = data.get('reason')

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


def main():
    """Entry point for the telemetry monitor."""
    rclpy.init()
    node = TelemetryMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n' + '━' * 70)
        print('Telemetry Monitor shutting down')
        print('━' * 70)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
