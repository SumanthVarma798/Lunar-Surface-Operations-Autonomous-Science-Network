import json
import re
import time

from _helpers import FakeBus, FakeString, load_module


class SimulatedSpaceLink:
    """Routes per-rover uplink/downlink topics like a space-link relay."""

    UPLINK_RE = re.compile(r'^/earth/uplink_cmd/([^/]+)$')
    TLM_RE = re.compile(r'^/rover/([^/]+)/downlink_telemetry$')
    ACK_RE = re.compile(r'^/rover/([^/]+)/ack$')

    def __init__(self, bus):
        self._bus = bus
        self._bus.add_alias_handler(self.route_topic)

    def route_topic(self, topic, _msg):
        uplink_match = self.UPLINK_RE.match(topic)
        if uplink_match:
            rover_id = uplink_match.group(1)
            return f'/rover/{rover_id}/command'

        telemetry_match = self.TLM_RE.match(topic)
        if telemetry_match:
            rover_id = telemetry_match.group(1)
            return f'/earth/telemetry/{rover_id}'

        ack_match = self.ACK_RE.match(topic)
        if ack_match:
            rover_id = ack_match.group(1)
            return f'/earth/ack/{rover_id}'

        return None


class FakeRover:
    def __init__(self, bus, rover_id, battery, solar_exposure):
        self._bus = bus
        self.rover_id = rover_id
        self.state = 'IDLE'
        self.task_id = None
        self.battery = battery
        self.solar_exposure = solar_exposure
        self._bus.subscribe(
            f'/rover/{rover_id}/command',
            self.command_callback,
        )

    def publish_telemetry(self):
        payload = {
            'ts': time.time(),
            'rover_id': self.rover_id,
            'state': self.state,
            'task_id': self.task_id,
            'battery': self.battery,
            'solar_exposure': self.solar_exposure,
            'data_buffer_size': 0,
            'fault': None,
        }
        self._bus.publish(
            f'/rover/{self.rover_id}/downlink_telemetry',
            FakeString(json.dumps(payload)),
        )

    def command_callback(self, msg):
        cmd = json.loads(msg.data)
        cmd_type = cmd.get('type')

        success = False
        reason = None
        if cmd_type == 'START_TASK' and self.state == 'IDLE':
            self.state = 'EXECUTING'
            self.task_id = cmd.get('task_id')
            success = True
        elif cmd_type == 'START_TASK':
            reason = f'Cannot start task in state {self.state}'
        else:
            success = True

        ack = {
            'ack_id': cmd.get('cmd_id'),
            'rover_id': self.rover_id,
            'status': 'ACCEPTED' if success else 'REJECTED',
            'reason': reason,
            'ts': time.time(),
        }
        self._bus.publish(
            f'/rover/{self.rover_id}/ack',
            FakeString(json.dumps(ack)),
        )
        self.publish_telemetry()


def _ack_pending_command(earth, bus, rover_id):
    pending_ids = list(earth.pending_commands.keys())
    assert len(pending_ids) == 1

    cmd_id = pending_ids[0]
    ack = {
        'ack_id': cmd_id,
        'rover_id': rover_id,
        'status': 'ACCEPTED',
        'reason': None,
        'ts': time.time(),
    }
    bus.publish(
        f'/rover/{rover_id}/ack',
        FakeString(json.dumps(ack)),
    )


def test_multi_rover_command_flow_auto_assignment_and_ack_routing(monkeypatch):
    bus = FakeBus()
    _space_link = SimulatedSpaceLink(bus)

    earth_module = load_module(
        monkeypatch,
        'rover_core.earth_node',
        bus,
        parameter_overrides={
            'earth_node': {
                'rover_ids': 'rover_1,rover_2,rover_3',
                'enable_topic_discovery': False,
            }
        },
    )
    earth = earth_module.EarthNode()

    rovers = {
        'rover_1': FakeRover(bus, 'rover_1', battery=0.95, solar_exposure=0.8),
        'rover_2': FakeRover(bus, 'rover_2', battery=0.88, solar_exposure=0.7),
        'rover_3': FakeRover(bus, 'rover_3', battery=0.81, solar_exposure=0.6),
    }

    for rover in rovers.values():
        rover.publish_telemetry()

    assert len(rovers) == 3
    assert set(earth.fleet_registry.keys()) >= {'rover_1', 'rover_2', 'rover_3'}

    initial_executing = [
        rid
        for rid, status in earth.fleet_registry.items()
        if status.get('state') == 'EXECUTING'
    ]
    assert initial_executing == []

    first_start = len(bus.published)
    assert earth.auto_assign_task('task-alpha') is True
    first_events = bus.published[first_start:]

    first_targets = [
        topic.split('/')[-1]
        for topic, _payload in first_events
        if topic.startswith('/earth/uplink_cmd/')
    ]
    assert len(first_targets) == 1
    first_target = first_targets[0]

    executing_after_first = sorted(
        rid
        for rid, status in earth.fleet_registry.items()
        if status.get('state') == 'EXECUTING'
    )
    assert executing_after_first == [first_target]

    ack_payloads = [
        json.loads(payload)
        for topic, payload in first_events
        if topic == f'/earth/ack/{first_target}'
    ]
    assert ack_payloads
    assert ack_payloads[0]['rover_id'] == first_target
    assert ack_payloads[0]['status'] == 'ACCEPTED'
    _ack_pending_command(earth, bus, first_target)
    assert earth.pending_commands == {}

    second_start = len(bus.published)
    assert earth.auto_assign_task('task-beta') is True
    second_events = bus.published[second_start:]

    second_targets = [
        topic.split('/')[-1]
        for topic, _payload in second_events
        if topic.startswith('/earth/uplink_cmd/')
    ]
    assert len(second_targets) == 1
    second_target = second_targets[0]

    assert second_target != first_target

    executing_after_second = sorted(
        rid
        for rid, status in earth.fleet_registry.items()
        if status.get('state') == 'EXECUTING'
    )
    assert sorted([first_target, second_target]) == executing_after_second
    _ack_pending_command(earth, bus, second_target)
    assert earth.pending_commands == {}
