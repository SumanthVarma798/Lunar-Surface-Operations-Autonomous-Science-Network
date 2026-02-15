import json
import re
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FleetManager(Node):
    """Centralized fleet orchestration node.

    Responsibilities:
    - Discover and subscribe to rover telemetry streams
    - Aggregate fleet status metrics
    - Publish fleet status at 1 Hz on /fleet/status
    - Optionally auto-recover rovers stuck in SAFE_MODE
    """

    TELEMETRY_TOPIC_PATTERN = re.compile(
        r'^/rover/([^/]+)/downlink_telemetry$'
    )

    def __init__(self):
        super().__init__('fleet_manager')

        # --- Runtime parameters ---
        self.declare_parameter('auto_recovery_enabled', False)
        self.declare_parameter('safe_mode_timeout_sec', 300.0)
        self.declare_parameter('reset_cooldown_sec', 60.0)
        self.declare_parameter('discovery_interval_sec', 2.0)

        self.auto_recovery_enabled = bool(
            self.get_parameter('auto_recovery_enabled').value
        )
        self.safe_mode_timeout_sec = float(
            self.get_parameter('safe_mode_timeout_sec').value
        )
        self.reset_cooldown_sec = float(
            self.get_parameter('reset_cooldown_sec').value
        )
        self.discovery_interval_sec = float(
            self.get_parameter('discovery_interval_sec').value
        )

        # --- Dynamic rover interfaces ---
        self._telemetry_subs = {}
        self._command_pubs = {}

        # rover_id -> state snapshot
        self._fleet = {}
        self._lock = threading.Lock()

        self.fleet_status_pub = self.create_publisher(
            String,
            '/fleet/status',
            10
        )

        # 1 Hz fleet status publishing
        self.status_timer = self.create_timer(1.0, self.publish_fleet_status)

        # Dynamic topic discovery
        self.discovery_timer = self.create_timer(
            self.discovery_interval_sec,
            self.discover_rovers
        )

        # Auto-recovery check loop at 1 Hz
        self.recovery_timer = self.create_timer(1.0, self.check_auto_recovery)

        self.discover_rovers()

        self.get_logger().info(
            'Fleet manager online\n'
            '  Telemetry source: /rover/<id>/downlink_telemetry\n'
            '  Fleet status topic: /fleet/status (1 Hz)\n'
            f'  Auto-recovery enabled: {self.auto_recovery_enabled}\n'
            f'  SAFE_MODE timeout: {self.safe_mode_timeout_sec:.1f}s\n'
            f'  Reset cooldown: {self.reset_cooldown_sec:.1f}s'
        )

    def discover_rovers(self):
        """Discover rover IDs by scanning telemetry topics in ROS graph."""
        try:
            topic_names_and_types = self.get_topic_names_and_types()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Rover discovery failed: {exc}')
            return

        discovered = set()
        for topic_name, _types in topic_names_and_types:
            match = self.TELEMETRY_TOPIC_PATTERN.match(topic_name)
            if match:
                discovered.add(match.group(1))

        for rover_id in sorted(discovered):
            self.ensure_rover_interfaces(rover_id, source='topic-scan')

    def ensure_rover_interfaces(self, rover_id, source='discovery'):
        """Create telemetry subscriber and command publisher for rover."""
        created = False

        if rover_id not in self._telemetry_subs:
            telemetry_topic = f'/rover/{rover_id}/downlink_telemetry'
            self._telemetry_subs[rover_id] = self.create_subscription(
                String,
                telemetry_topic,
                lambda msg, rid=rover_id: self.telemetry_callback(msg, rid),
                10,
            )
            created = True

        if rover_id not in self._command_pubs:
            self._command_pubs[rover_id] = self.create_publisher(
                String,
                f'/rover/{rover_id}/command',
                10,
            )
            created = True

        with self._lock:
            if rover_id not in self._fleet:
                self._fleet[rover_id] = {
                    'state': 'UNKNOWN',
                    'battery': 0.0,
                    'task_id': None,
                    'fault': None,
                    'last_seen': None,
                    'safe_mode_since': None,
                    'last_reset_sent_at': None,
                }
                created = True

        if created:
            self.get_logger().info(
                f'Fleet interface ready for [{rover_id}] via {source}'
            )

    def telemetry_callback(self, msg, subscribed_rover_id):
        """Track per-rover telemetry and state transitions in real-time."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'Invalid telemetry JSON from [{subscribed_rover_id}]'
            )
            return

        rover_id = data.get('rover_id', subscribed_rover_id)
        if rover_id != subscribed_rover_id:
            self.ensure_rover_interfaces(rover_id, source='telemetry')

        state = data.get('state', 'UNKNOWN')

        try:
            battery = float(data.get('battery', 0.0))
        except (TypeError, ValueError):
            battery = 0.0

        now = time.time()
        with self._lock:
            if rover_id not in self._fleet:
                self._fleet[rover_id] = {
                    'state': 'UNKNOWN',
                    'battery': 0.0,
                    'task_id': None,
                    'fault': None,
                    'last_seen': None,
                    'safe_mode_since': None,
                    'last_reset_sent_at': None,
                }

            rover = self._fleet[rover_id]
            old_state = rover['state']

            rover.update({
                'state': state,
                'battery': battery,
                'task_id': data.get('task_id'),
                'fault': data.get('fault'),
                'last_seen': now,
            })

            if state == 'SAFE_MODE':
                if rover['safe_mode_since'] is None:
                    rover['safe_mode_since'] = now
            else:
                rover['safe_mode_since'] = None

        if old_state != state:
            self.get_logger().info(
                f'[{rover_id}] state transition: {old_state} -> {state}'
            )

            if state == 'SAFE_MODE':
                self.get_logger().warn(
                    f'[{rover_id}] entered SAFE_MODE; fault={data.get("fault")}'
                )

    def publish_fleet_status(self):
        """Aggregate and publish fleet status to /fleet/status at 1 Hz."""
        with self._lock:
            snapshot = {
                rover_id: values.copy()
                for rover_id, values in self._fleet.items()
            }

        state_distribution = {}
        batteries = []
        rovers_out = {}

        for rover_id in sorted(snapshot.keys()):
            rover = snapshot[rover_id]
            state = rover.get('state', 'UNKNOWN')
            state_distribution[state] = state_distribution.get(state, 0) + 1

            if rover.get('last_seen') is not None:
                batteries.append(float(rover.get('battery', 0.0)))

            rovers_out[rover_id] = {
                'state': state,
                'battery': float(rover.get('battery', 0.0)),
                'task_id': rover.get('task_id'),
                'fault': rover.get('fault'),
                'last_seen': rover.get('last_seen'),
            }

        avg_battery = (
            round(sum(batteries) / len(batteries), 3)
            if batteries else 0.0
        )

        payload = {
            'ts': time.time(),
            'total_rovers': len(snapshot),
            'state_distribution': state_distribution,
            'avg_battery': avg_battery,
            'rovers': rovers_out,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.fleet_status_pub.publish(msg)

    def check_auto_recovery(self):
        """Optionally send RESET to rovers stuck in SAFE_MODE too long."""
        if not self.auto_recovery_enabled:
            return

        now = time.time()

        with self._lock:
            snapshot = {
                rover_id: values.copy()
                for rover_id, values in self._fleet.items()
            }

        for rover_id, rover in snapshot.items():
            if rover.get('state') != 'SAFE_MODE':
                continue

            safe_mode_since = rover.get('safe_mode_since')
            if safe_mode_since is None:
                continue

            elapsed = now - float(safe_mode_since)
            if elapsed < self.safe_mode_timeout_sec:
                continue

            last_reset_sent_at = rover.get('last_reset_sent_at')
            if last_reset_sent_at is not None:
                cooldown_elapsed = now - float(last_reset_sent_at)
                if cooldown_elapsed < self.reset_cooldown_sec:
                    continue

            publisher = self._command_pubs.get(rover_id)
            if publisher is None:
                self.ensure_rover_interfaces(rover_id, source='auto-recovery')
                publisher = self._command_pubs.get(rover_id)

            if publisher is None:
                self.get_logger().error(
                    f'Auto-recovery failed for [{rover_id}]: '
                    'command publisher unavailable'
                )
                continue

            cmd = {
                'cmd_id': f'fleet-reset-{rover_id}-{int(now)}',
                'type': 'RESET',
                'task_id': None,
                'ts': now,
                'source': 'fleet_manager',
            }

            msg = String()
            msg.data = json.dumps(cmd)
            publisher.publish(msg)

            with self._lock:
                if rover_id in self._fleet:
                    self._fleet[rover_id]['last_reset_sent_at'] = now

            self.get_logger().warn(
                f'Auto-recovery: sent RESET to [{rover_id}] after '
                f'{elapsed:.1f}s in SAFE_MODE'
            )


def main():
    """Entry point for fleet manager node."""
    rclpy.init()
    node = FleetManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
