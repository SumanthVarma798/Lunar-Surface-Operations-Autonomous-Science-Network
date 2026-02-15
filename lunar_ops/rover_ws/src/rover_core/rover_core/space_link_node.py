import random
import re
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SpaceLinkNode(Node):
    """Space Link relay node — simulates realistic Moon-Earth comms.

    Multi-rover behavior:
    - Dynamically discovers rovers via `/rover/*` topic patterns.
    - Relays Earth uplink commands to all discovered rover command topics.
    - Relays rover telemetry/ACKs back to shared Earth topics.
    - Tracks per-rover relay statistics.
    """

    COMMAND_TOPIC_PATTERN = re.compile(r'^/rover/([^/]+)/command$')
    TELEMETRY_TOPIC_PATTERN = re.compile(
        r'^/rover/([^/]+)/downlink_telemetry$'
    )
    ACK_TOPIC_PATTERN = re.compile(r'^/rover/([^/]+)/ack$')

    def __init__(self):
        super().__init__('space_link_node')

        # Link simulation parameters
        self.declare_parameter('base_latency', 1.3)
        self.declare_parameter('jitter', 0.2)
        self.declare_parameter('drop_rate', 0.05)
        self.declare_parameter('duplication_rate', 0.0)
        self.declare_parameter('discovery_interval', 2.0)

        self.base_latency = self.get_parameter('base_latency').value
        self.jitter = self.get_parameter('jitter').value
        self.drop_rate = self.get_parameter('drop_rate').value
        self.duplication_rate = self.get_parameter('duplication_rate').value
        self.discovery_interval = self.get_parameter(
            'discovery_interval'
        ).value

        # Shared Earth downlink topics
        self.earth_tlm_pub = self.create_publisher(
            String, '/earth/telemetry', 10
        )
        self.earth_ack_pub = self.create_publisher(
            String, '/earth/ack', 10
        )

        # Earth uplink input (fan-out to all discovered rovers)
        self.earth_uplink_sub = self.create_subscription(
            String,
            '/earth/uplink_cmd',
            self.uplink_callback,
            10
        )

        # Dynamic relay resources
        self._rover_command_pubs = {}
        self._rover_tlm_subs = {}
        self._rover_ack_subs = {}

        # Per-rover statistics with thread-safe updates
        self._stats = {}
        self._stats_lock = threading.Lock()

        self.discovery_timer = self.create_timer(
            float(self.discovery_interval), self.discover_rovers
        )
        self.stats_timer = self.create_timer(15.0, self.log_stats)

        # Initial discovery on startup
        self.discover_rovers()

        self.get_logger().info(
            '🛰️  Space Link relay initialized (multi-rover dynamic mode)\n'
            '   Uplink: /earth/uplink_cmd -> /rover/<id>/command\n'
            '   Downlink telemetry: /rover/<id>/downlink_telemetry '
            '-> /earth/telemetry\n'
            '   Downlink ACK: /rover/<id>/ack -> /earth/ack\n'
            f'   Latency: {self.base_latency}s ± {self.jitter}s\n'
            f'   Drop rate: {self.drop_rate * 100:.1f}%\n'
            f'   Duplication rate: {self.duplication_rate * 100:.1f}%\n'
            f'   Discovery interval: {self.discovery_interval:.1f}s'
        )

    def discover_rovers(self):
        """Discover rover IDs from ROS graph topic patterns."""
        topic_names = self.get_topic_names_and_types()
        discovered_rovers = set()

        for topic_name, _ in topic_names:
            command_match = self.COMMAND_TOPIC_PATTERN.match(topic_name)
            telemetry_match = self.TELEMETRY_TOPIC_PATTERN.match(topic_name)
            ack_match = self.ACK_TOPIC_PATTERN.match(topic_name)

            if command_match:
                discovered_rovers.add(command_match.group(1))
            if telemetry_match:
                discovered_rovers.add(telemetry_match.group(1))
            if ack_match:
                discovered_rovers.add(ack_match.group(1))

        for rover_id in sorted(discovered_rovers):
            self.setup_rover_relay(rover_id)

    def _init_stats(self, rover_id):
        """Initialize per-rover relay statistics if missing."""
        with self._stats_lock:
            if rover_id in self._stats:
                return
            self._stats[rover_id] = {
                'commands_received': 0,
                'commands_relayed': 0,
                'commands_dropped': 0,
                'commands_duplicated': 0,
                'commands_publish_errors': 0,
                'telemetry_received': 0,
                'telemetry_relayed': 0,
                'telemetry_dropped': 0,
                'telemetry_duplicated': 0,
                'telemetry_publish_errors': 0,
                'acks_received': 0,
                'acks_relayed': 0,
                'acks_dropped': 0,
                'acks_duplicated': 0,
                'acks_publish_errors': 0,
            }

    def _increment_stat(self, rover_id, stat_key, amount=1):
        """Thread-safe stat increment helper."""
        self._init_stats(rover_id)
        with self._stats_lock:
            self._stats[rover_id][stat_key] += amount

    def setup_rover_relay(self, rover_id):
        """Create relay pub/sub pairs for a rover if not already created."""
        self._init_stats(rover_id)
        created_any = False

        if rover_id not in self._rover_command_pubs:
            command_topic = f'/rover/{rover_id}/command'
            self._rover_command_pubs[rover_id] = self.create_publisher(
                String, command_topic, 10
            )
            created_any = True

        if rover_id not in self._rover_tlm_subs:
            telemetry_topic = f'/rover/{rover_id}/downlink_telemetry'
            self._rover_tlm_subs[rover_id] = self.create_subscription(
                String,
                telemetry_topic,
                lambda msg, rid=rover_id: self.telemetry_callback(msg, rid),
                10
            )
            created_any = True

        if rover_id not in self._rover_ack_subs:
            ack_topic = f'/rover/{rover_id}/ack'
            self._rover_ack_subs[rover_id] = self.create_subscription(
                String,
                ack_topic,
                lambda msg, rid=rover_id: self.ack_callback(msg, rid),
                10
            )
            created_any = True

        if created_any:
            self.get_logger().info(
                f'   📡 Relay active for [{rover_id}] — '
                f'command: /rover/{rover_id}/command, '
                f'telemetry: /rover/{rover_id}/downlink_telemetry, '
                f'ack: /rover/{rover_id}/ack'
            )

    def uplink_callback(self, msg):
        """Relay Earth uplink command to all discovered rovers."""
        if not self._rover_command_pubs:
            self.get_logger().warn(
                '⚠️  No rover command topics discovered; '
                'dropping uplink command'
            )
            return

        for rover_id in sorted(self._rover_command_pubs.keys()):
            publisher = self._rover_command_pubs[rover_id]
            self._increment_stat(rover_id, 'commands_received')
            self.relay_message(
                msg,
                publisher,
                f'UPLINK-{rover_id}',
                rover_id,
                'commands'
            )

    def telemetry_callback(self, msg, rover_id):
        """Relay rover telemetry to Earth shared telemetry topic."""
        self._increment_stat(rover_id, 'telemetry_received')
        self.relay_message(
            msg,
            self.earth_tlm_pub,
            f'DOWNLINK-TLM-{rover_id}',
            rover_id,
            'telemetry'
        )

    def ack_callback(self, msg, rover_id):
        """Relay rover ACK to Earth shared ACK topic."""
        self._increment_stat(rover_id, 'acks_received')
        self.relay_message(
            msg,
            self.earth_ack_pub,
            f'DOWNLINK-ACK-{rover_id}',
            rover_id,
            'acks'
        )

    def log_stats(self):
        """Periodically log per-rover relay statistics."""
        with self._stats_lock:
            if not self._stats:
                return
            stats_snapshot = {
                rover_id: values.copy()
                for rover_id, values in self._stats.items()
            }

        lines = ['🛰️  Relay stats per rover:']
        for rover_id in sorted(stats_snapshot.keys()):
            s = stats_snapshot[rover_id]
            lines.append(
                f'   [{rover_id}] '
                f'cmd rx/ok/drop/dup/err='
                f'{s["commands_received"]}/{s["commands_relayed"]}/'
                f'{s["commands_dropped"]}/{s["commands_duplicated"]}/'
                f'{s["commands_publish_errors"]}, '
                f'tlm rx/ok/drop/dup/err='
                f'{s["telemetry_received"]}/{s["telemetry_relayed"]}/'
                f'{s["telemetry_dropped"]}/{s["telemetry_duplicated"]}/'
                f'{s["telemetry_publish_errors"]}, '
                f'ack rx/ok/drop/dup/err='
                f'{s["acks_received"]}/{s["acks_relayed"]}/'
                f'{s["acks_dropped"]}/{s["acks_duplicated"]}/'
                f'{s["acks_publish_errors"]}'
            )

        self.get_logger().info('\n'.join(lines))

    def simulate_delay(self):
        """Calculate realistic delay with jitter."""
        jitter_value = random.uniform(-self.jitter, self.jitter)
        delay = self.base_latency + jitter_value
        return max(0.0, delay)

    def should_drop(self):
        """Determine if packet should be dropped."""
        return random.random() < self.drop_rate

    def should_duplicate(self):
        """Determine if packet should be duplicated."""
        return random.random() < self.duplication_rate

    def relay_message(self, msg, publisher, direction, rover_id, channel):
        """Relay a message with simulated space link conditions.

        Args:
            msg: The ROS message to relay
            publisher: The ROS publisher to use
            direction: Label for logging (e.g. 'UPLINK-rover_1')
            rover_id: Rover identity for per-rover stats
            channel: One of 'commands', 'telemetry', 'acks'
        """
        if self.should_drop():
            self._increment_stat(rover_id, f'{channel}_dropped')
            self.get_logger().warn(
                f'❌ {direction} DROPPED: {msg.data[:60]}...'
            )
            return

        delay = self.simulate_delay()

        self.get_logger().info(
            f'📡 {direction} relay (delay: {delay:.2f}s): '
            f'{msg.data[:60]}...'
        )

        timer = threading.Timer(
            delay,
            lambda: self._publish_message(
                publisher, msg, direction, rover_id, channel
            )
        )
        timer.daemon = True
        timer.start()
        self._increment_stat(rover_id, f'{channel}_relayed')

        if self.should_duplicate():
            self._increment_stat(rover_id, f'{channel}_duplicated')
            dup_delay = self.simulate_delay()
            self.get_logger().info(
                f'📡 {direction} DUPLICATE (delay: {dup_delay:.2f}s): '
                f'{msg.data[:60]}...'
            )
            dup_timer = threading.Timer(
                dup_delay,
                lambda: self._publish_message(
                    publisher,
                    msg,
                    direction,
                    rover_id,
                    channel,
                    is_duplicate=True
                )
            )
            dup_timer.daemon = True
            dup_timer.start()

    def _publish_message(self, publisher, msg, direction, rover_id, channel,
                         is_duplicate=False):
        """Publish message after delay."""
        try:
            # Rebuild String message to avoid sharing mutable objects across
            # timer threads.
            output_msg = String()
            output_msg.data = msg.data
            publisher.publish(output_msg)
            if is_duplicate:
                self.get_logger().debug(
                    f'✅ {direction} DUPLICATE delivered'
                )
        except Exception as e:
            self._increment_stat(rover_id, f'{channel}_publish_errors')
            self.get_logger().error(
                f'Error publishing {direction}: {e}'
            )


def main():
    """Entry point for the space link node."""
    rclpy.init()
    node = SpaceLinkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('🛰️  Space Link relay shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
