import random
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SpaceLinkNode(Node):
    """Space Link relay node — simulates realistic Moon-Earth comms.

    Supports multi-rover constellations: accepts a 'rover_ids' parameter
    (comma-separated list) and creates relay pairs for each rover.

    Introduces:
    - Latency (fixed + jitter)
    - Packet drops
    - Optional duplication
    - Optional out-of-order delivery
    """

    def __init__(self):
        super().__init__('space_link_node')

        # Declare parameters with defaults
        self.declare_parameter('base_latency', 1.3)
        self.declare_parameter('jitter', 0.2)
        self.declare_parameter('drop_rate', 0.05)
        self.declare_parameter('duplication_rate', 0.0)
        self.declare_parameter('rover_ids', 'rover_1')

        # Get parameter values
        self.base_latency = self.get_parameter('base_latency').value
        self.jitter = self.get_parameter('jitter').value
        self.drop_rate = self.get_parameter('drop_rate').value
        self.duplication_rate = self.get_parameter('duplication_rate').value

        # Parse rover IDs
        rover_ids_str = self.get_parameter('rover_ids').value
        self.rover_ids = [rid.strip() for rid in rover_ids_str.split(',')]

        # Storage for dynamic subscribers/publishers
        self._subs = []
        self._pubs = {}

        # Create relay pairs for each rover
        for rover_id in self.rover_ids:
            self._setup_rover_relay(rover_id)

        self.get_logger().info(
            f'🛰️  Space Link relay initialized\n'
            f'   Rovers: {", ".join(self.rover_ids)}\n'
            f'   Latency: {self.base_latency}s ± {self.jitter}s\n'
            f'   Drop rate: {self.drop_rate * 100:.1f}%\n'
            f'   Duplication rate: {self.duplication_rate * 100:.1f}%'
        )

    def _setup_rover_relay(self, rover_id):
        """Create uplink/downlink relay pairs for a single rover."""
        # --- Uplink: Earth -> Rover ---
        uplink_pub_topic = f'/rover/{rover_id}/command'
        uplink_pub = self.create_publisher(String, uplink_pub_topic, 10)
        self._pubs[f'uplink_{rover_id}'] = uplink_pub

        uplink_sub_topic = f'/earth/uplink_cmd/{rover_id}'
        uplink_sub = self.create_subscription(
            String,
            uplink_sub_topic,
            lambda msg, pub=uplink_pub, rid=rover_id:
                self.relay_message(msg, pub, f'UPLINK-{rid}'),
            10
        )
        self._subs.append(uplink_sub)

        # --- Downlink: Rover -> Earth (telemetry) ---
        dl_tlm_pub_topic = f'/earth/telemetry/{rover_id}'
        dl_tlm_pub = self.create_publisher(String, dl_tlm_pub_topic, 10)
        self._pubs[f'dl_tlm_{rover_id}'] = dl_tlm_pub

        dl_tlm_sub_topic = f'/rover/{rover_id}/downlink_telemetry'
        dl_tlm_sub = self.create_subscription(
            String,
            dl_tlm_sub_topic,
            lambda msg, pub=dl_tlm_pub, rid=rover_id:
                self.relay_message(msg, pub, f'DOWNLINK-TLM-{rid}'),
            10
        )
        self._subs.append(dl_tlm_sub)

        # --- Downlink: Rover -> Earth (ACKs) ---
        dl_ack_pub_topic = f'/earth/ack/{rover_id}'
        dl_ack_pub = self.create_publisher(String, dl_ack_pub_topic, 10)
        self._pubs[f'dl_ack_{rover_id}'] = dl_ack_pub

        dl_ack_sub_topic = f'/rover/{rover_id}/ack'
        dl_ack_sub = self.create_subscription(
            String,
            dl_ack_sub_topic,
            lambda msg, pub=dl_ack_pub, rid=rover_id:
                self.relay_message(msg, pub, f'DOWNLINK-ACK-{rid}'),
            10
        )
        self._subs.append(dl_ack_sub)

        self.get_logger().info(
            f'   📡 Relay for [{rover_id}] — '
            f'uplink: {uplink_sub_topic} → {uplink_pub_topic}, '
            f'downlink-tlm: {dl_tlm_sub_topic} → {dl_tlm_pub_topic}, '
            f'downlink-ack: {dl_ack_sub_topic} → {dl_ack_pub_topic}'
        )

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

    def relay_message(self, msg, publisher, direction):
        """Relay a message with simulated space link conditions.

        Args:
            msg: The ROS message to relay
            publisher: The ROS publisher to use
            direction: Label for logging (e.g. 'UPLINK-rover_1')
        """
        if self.should_drop():
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
            lambda: self._publish_message(publisher, msg, direction)
        )
        timer.daemon = True
        timer.start()

        if self.should_duplicate():
            dup_delay = self.simulate_delay()
            self.get_logger().info(
                f'📡 {direction} DUPLICATE (delay: {dup_delay:.2f}s): '
                f'{msg.data[:60]}...'
            )
            dup_timer = threading.Timer(
                dup_delay,
                lambda: self._publish_message(
                    publisher, msg, direction, is_duplicate=True
                )
            )
            dup_timer.daemon = True
            dup_timer.start()

    def _publish_message(self, publisher, msg, direction,
                         is_duplicate=False):
        """Publish message after delay."""
        try:
            publisher.publish(msg)
            if is_duplicate:
                self.get_logger().debug(
                    f'✅ {direction} DUPLICATE delivered'
                )
        except Exception as e:
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
