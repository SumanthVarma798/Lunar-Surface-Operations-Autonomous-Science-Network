import importlib
import sys
import types
from pathlib import Path


class FakeString:
    def __init__(self, data=''):
        self.data = data


class FakeLogger:
    def info(self, *_args, **_kwargs):
        return None

    def warn(self, *_args, **_kwargs):
        return None

    def error(self, *_args, **_kwargs):
        return None


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeBus:
    """In-process pub/sub bus with optional topic alias routing."""

    def __init__(self):
        self.subscribers = {}
        self.alias_handlers = []
        self.published = []

    def subscribe(self, topic, callback):
        self.subscribers.setdefault(topic, []).append(callback)

    def add_alias_handler(self, handler):
        self.alias_handlers.append(handler)

    def topic_names_and_types(self):
        topics = set(self.subscribers.keys())
        topics.update(topic for topic, _payload in self.published)
        return sorted((topic, ['std_msgs/msg/String']) for topic in topics)

    def publish(self, topic, msg):
        queue = [topic]
        seen = set()

        while queue:
            current = queue.pop(0)
            if current in seen:
                continue
            seen.add(current)

            self.published.append((current, msg.data))
            for callback in list(self.subscribers.get(current, [])):
                callback(msg)

            for handler in self.alias_handlers:
                routed = handler(current, msg)
                if not routed:
                    continue
                if isinstance(routed, str):
                    queue.append(routed)
                else:
                    queue.extend(routed)


class FakePublisher:
    def __init__(self, bus, topic):
        self._bus = bus
        self.topic = topic

    def publish(self, msg):
        self._bus.publish(self.topic, msg)


class FakeNode:
    _bus = None
    _parameter_overrides = {}

    @classmethod
    def configure(cls, bus, parameter_overrides=None):
        cls._bus = bus
        cls._parameter_overrides = parameter_overrides or {}

    def __init__(self, node_name):
        self._name = node_name
        self._logger = FakeLogger()
        self._timers = []
        self._params = dict(self._parameter_overrides.get(node_name, {}))

    def declare_parameter(self, name, default_value):
        self._params.setdefault(name, default_value)

    def get_parameter(self, name):
        return FakeParameter(self._params[name])

    def create_publisher(self, _msg_type, topic, _qos):
        return FakePublisher(self._bus, topic)

    def create_subscription(self, _msg_type, topic, callback, _qos):
        self._bus.subscribe(topic, callback)
        return types.SimpleNamespace(topic=topic, callback=callback)

    def create_timer(self, period, callback):
        timer = types.SimpleNamespace(period=period, callback=callback)
        self._timers.append(timer)
        return timer

    def get_topic_names_and_types(self):
        return self._bus.topic_names_and_types()

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        return None


def install_fake_ros(monkeypatch, bus, parameter_overrides=None):
    FakeNode.configure(bus, parameter_overrides=parameter_overrides)

    rclpy_module = types.ModuleType('rclpy')
    rclpy_node_module = types.ModuleType('rclpy.node')
    std_msgs_module = types.ModuleType('std_msgs')
    std_msgs_msg_module = types.ModuleType('std_msgs.msg')

    rclpy_node_module.Node = FakeNode
    rclpy_module.node = rclpy_node_module
    rclpy_module.init = lambda *_args, **_kwargs: None
    rclpy_module.shutdown = lambda *_args, **_kwargs: None
    rclpy_module.spin = lambda *_args, **_kwargs: None
    rclpy_module.ok = lambda: True

    std_msgs_msg_module.String = FakeString
    std_msgs_module.msg = std_msgs_msg_module

    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_module)
    monkeypatch.setitem(sys.modules, 'rclpy.node', rclpy_node_module)
    monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs_module)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', std_msgs_msg_module)


def load_module(monkeypatch, module_name, bus, parameter_overrides=None):
    install_fake_ros(
        monkeypatch,
        bus,
        parameter_overrides=parameter_overrides,
    )

    src_root = Path(__file__).resolve().parents[1]
    src_root_str = str(src_root)
    if src_root_str not in sys.path:
        sys.path.insert(0, src_root_str)

    if module_name in sys.modules:
        del sys.modules[module_name]

    return importlib.import_module(module_name)
