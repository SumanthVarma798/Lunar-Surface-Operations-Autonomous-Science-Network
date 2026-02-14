import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import threading
import time

class SpaceLinkNode(Node):
    """
    Space Link relay node - simulates realistic Moon↔Earth communication conditions.
    
    Introduces:
    - Latency (fixed + jitter)
    - Packet drops
    - Optional duplication
    - Optional out-of-order delivery
    """
    
    def __init__(self):
        super().__init__('space_link_node')
        
        # Declare parameters with defaults
        self.declare_parameter('base_latency', 1.3)  # seconds, one-way Moon-Earth light time
        self.declare_parameter('jitter', 0.2)  # ± seconds
        self.declare_parameter('drop_rate', 0.05)  # 5% packet loss
        self.declare_parameter('duplication_rate', 0.0)  # disabled by default
        
        # Get parameter values
        self.base_latency = self.get_parameter('base_latency').value
        self.jitter = self.get_parameter('jitter').value
        self.drop_rate = self.get_parameter('drop_rate').value
        self.duplication_rate = self.get_parameter('duplication_rate').value
        
        # Uplink: Earth → Rover
        self.uplink_sub = self.create_subscription(
            String,
            '/earth/uplink_cmd',
            self.uplink_callback,
            10
        )
        self.uplink_pub = self.create_publisher(
            String,
            '/rover/command',
            10
        )
        
        # Downlink: Rover → Earth (telemetry)
        self.downlink_telemetry_sub = self.create_subscription(
            String,
            '/rover/downlink_telemetry',
            self.downlink_telemetry_callback,
            10
        )
        self.downlink_telemetry_pub = self.create_publisher(
            String,
            '/earth/telemetry',
            10
        )
        
        # Downlink: Rover → Earth (ACKs)
        self.downlink_ack_sub = self.create_subscription(
            String,
            '/rover/ack',
            self.downlink_ack_callback,
            10
        )
        self.downlink_ack_pub = self.create_publisher(
            String,
            '/earth/ack',
            10
        )
        
        self.get_logger().info(
            f"🛰️  Space Link relay initialized\n"
            f"   Latency: {self.base_latency}s ± {self.jitter}s\n"
            f"   Drop rate: {self.drop_rate*100:.1f}%\n"
            f"   Duplication rate: {self.duplication_rate*100:.1f}%"
        )
    
    def simulate_delay(self):
        """Calculate realistic delay with jitter"""
        jitter_value = random.uniform(-self.jitter, self.jitter)
        delay = self.base_latency + jitter_value
        return max(0.0, delay)  # Ensure non-negative
    
    def should_drop(self):
        """Determine if packet should be dropped"""
        return random.random() < self.drop_rate
    
    def should_duplicate(self):
        """Determine if packet should be duplicated"""
        return random.random() < self.duplication_rate
    
    def relay_message(self, msg, publisher, direction):
        """
        Relay a message with simulated space link conditions
        
        Args:
            msg: The ROS message to relay
            publisher: The ROS publisher to use
            direction: "UPLINK" or "DOWNLINK" for logging
        """
        # Check if dropped
        if self.should_drop():
            self.get_logger().warn(f"❌ {direction} DROPPED: {msg.data[:60]}...")
            return
        
        # Calculate delay
        delay = self.simulate_delay()
        
        # Log the relay
        self.get_logger().info(
            f"📡 {direction} relay (delay: {delay:.2f}s): {msg.data[:60]}..."
        )
        
        # Schedule delayed delivery
        timer = threading.Timer(
            delay,
            lambda: self._publish_message(publisher, msg, direction)
        )
        timer.daemon = True
        timer.start()
        
        # Handle duplication if enabled
        if self.should_duplicate():
            dup_delay = self.simulate_delay()
            self.get_logger().info(
                f"📡 {direction} DUPLICATE (delay: {dup_delay:.2f}s): {msg.data[:60]}..."
            )
            dup_timer = threading.Timer(
                dup_delay,
                lambda: self._publish_message(publisher, msg, direction, is_duplicate=True)
            )
            dup_timer.daemon = True
            dup_timer.start()
    
    def _publish_message(self, publisher, msg, direction, is_duplicate=False):
        """Publish message after delay"""
        try:
            publisher.publish(msg)
            if is_duplicate:
                self.get_logger().debug(f"✅ {direction} DUPLICATE delivered")
        except Exception as e:
            self.get_logger().error(f"Error publishing {direction}: {e}")
    
    def uplink_callback(self, msg):
        """Handle uplink: Earth → Rover"""
        self.relay_message(msg, self.uplink_pub, "UPLINK")
    
    def downlink_telemetry_callback(self, msg):
        """Handle downlink telemetry: Rover → Earth"""
        self.relay_message(msg, self.downlink_telemetry_pub, "DOWNLINK-TLM")
    
    def downlink_ack_callback(self, msg):
        """Handle downlink ACKs: Rover → Earth"""
        self.relay_message(msg, self.downlink_ack_pub, "DOWNLINK-ACK")

def main():
    rclpy.init()
    node = SpaceLinkNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🛰️  Space Link relay shutting down")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
