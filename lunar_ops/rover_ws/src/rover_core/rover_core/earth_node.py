import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json

class EarthNode(Node):
    """Earth station for sending commands to the rover"""
    
    def __init__(self):
        super().__init__('earth_node')

        self.command_pub = self.create_publisher(
            String,
            '/rover/command',
            10
        )

        self.telemetry_sub = self.create_subscription(
            String,
            '/rover/telemetry',
            self.telemetry_callback,
            10
        )
        
        self.get_logger().info("🌍 Earth station online")
        self.print_help()

    def telemetry_callback(self, msg):
        """Display telemetry from rover"""
        try:
            data = json.loads(msg.data)
            
            # Format timestamp
            from datetime import datetime
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
            battery_pct = f"{int(battery * 100)}%"
            if battery < 0.2:
                battery_display = f"🔋❗{battery_pct}"
            elif battery < 0.5:
                battery_display = f"🔋⚠️ {battery_pct}"
            else:
                battery_display = f"🔋 {battery_pct}"
            
            # Build one-liner
            parts = [f"{ts}", f"{icon} {state}", battery_display]
            
            if data['task_id']:
                parts.append(f"📋 {data['task_id']}")
            
            if data['fault']:
                parts.append(f"⚠️  {data['fault']}")
            
            telemetry_line = " | ".join(parts)
            print(f"📡 {telemetry_line}")
            
        except json.JSONDecodeError:
            self.get_logger().warn(f"Failed to parse telemetry JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing telemetry: {e}")
    
    def send_command(self, command):
        """Send command to rover"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"📤 Sent command: {command}")
    
    def print_help(self):
        """Print available commands"""
        print("\n" + "="*60)
        print("🌍 EARTH STATION COMMAND INTERFACE")
        print("="*60)
        print("Available commands:")
        print("  start <task_id>  - Start a new task (e.g., 'start SAMPLE-001')")
        print("  abort            - Abort current task")
        print("  safe             - Put rover in safe mode")
        print("  reset            - Reset from safe mode to idle")
        print("  help             - Show this help")
        print("  quit             - Exit")
        print("="*60 + "\n")

def command_loop(node):
    """Interactive command loop"""
    while rclpy.ok():
        try:
            user_input = input("Earth> ").strip()
            
            if not user_input:
                continue
            
            parts = user_input.split(maxsplit=1)
            cmd = parts[0].lower()
            
            if cmd == "quit" or cmd == "exit":
                print("Shutting down Earth station...")
                rclpy.shutdown()
                break
            elif cmd == "help":
                node.print_help()
            elif cmd == "start":
                if len(parts) < 2:
                    print("❌ Usage: start <task_id>")
                else:
                    task_id = parts[1]
                    node.send_command(f"START_TASK:{task_id}")
            elif cmd == "abort":
                node.send_command("ABORT")
            elif cmd == "safe":
                node.send_command("GO_SAFE")
            elif cmd == "reset":
                node.send_command("RESET")
            else:
                print(f"❌ Unknown command: {cmd}. Type 'help' for options.")
        
        except EOFError:
            print("\nShutting down Earth station...")
            rclpy.shutdown()
            break
        except KeyboardInterrupt:
            print("\nShutting down Earth station...")
            rclpy.shutdown()
            break

def main():
    rclpy.init()
    node = EarthNode()
    
    # Run ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Run command loop in main thread
    command_loop(node)
    
    rclpy.shutdown()