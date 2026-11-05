"""
pymoveit2 for the browser demo.

Pyodide imports this module as `pymoveit2`. It has the public names of the real
package, but every call goes to the demo's JavaScript stand-in for pymoveit2
and MoveIt 2 (`web/src/api`), which checks arguments with the real signatures
and drives the simulated robot. Only `connect()` is supported: building
`MoveIt2` and friends from your own ROS 2 node needs real ROS 2.
"""

import enum
import json
import math
import re

import _pymoveit2_bridge as _bridge

__all__ = [
    "GripperInterface",
    "MoveIt2",
    "MoveIt2Gripper",
    "MoveIt2Servo",
    "MoveIt2State",
    "RobotDescription",
    "RobotSession",
    "connect",
]

_PACKAGES = {
    "Header": "std_msgs.msg",
    "Time": "builtin_interfaces.msg",
    "Duration": "builtin_interfaces.msg",
    "PoseStamped": "geometry_msgs.msg",
    "Pose": "geometry_msgs.msg",
    "Point": "geometry_msgs.msg",
    "Quaternion": "geometry_msgs.msg",
    "JointState": "sensor_msgs.msg",
    "JointTrajectory": "trajectory_msgs.msg",
    "JointTrajectoryPoint": "trajectory_msgs.msg",
    "PlanningScene": "moveit_msgs.msg",
}
# Message types of nested fields, for their reprs.
_FIELD_TYPES = {
    "header": "Header",
    "stamp": "Time",
    "pose": "Pose",
    "position": "Point",
    "orientation": "Quaternion",
    "time_from_start": "Duration",
}
# Message fields that hold floats; JSON writes a whole float such as 0.0 as 0.
_FLOAT_FIELDS = frozenset(
    ("x", "y", "z", "w", "position", "positions", "velocity", "velocities")
    + ("accelerations", "effort")
)


class MoveIt2State(enum.Enum):
    """State of a MoveIt2 or gripper interface."""

    IDLE = 0
    REQUESTING = 1
    EXECUTING = 2


class Message:
    """A ROS 2 message returned by the demo, with attribute access to its fields."""

    def __init__(self, type_name, fields):
        object.__setattr__(self, "_type", type_name)
        object.__setattr__(self, "_fields", dict(fields))

    def __getattr__(self, name):
        try:
            return self._fields[name]
        except KeyError:
            raise AttributeError(
                f"'{self._type}' object has no attribute '{name}'"
            ) from None

    def __setattr__(self, name, value):
        if name not in self._fields:
            raise AttributeError(f"'{self._type}' object has no attribute '{name}'")
        self._fields[name] = value

    def __eq__(self, other):
        return (
            isinstance(other, Message)
            and self._type == other._type
            and self._fields == other._fields
        )

    def __repr__(self):
        package = _PACKAGES.get(self._type)
        name = f"{package}.{self._type}" if package else self._type
        inner = ", ".join(f"{key}={value!r}" for key, value in self._fields.items())
        return f"{name}({inner})"


class Future:
    """An rclpy Future that the demo completes immediately."""

    def __init__(self, result=None):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


def _float_hint(hint):
    """Whether a pymoveit2 annotation, such as `List[float]`, holds only floats."""
    rest = re.sub(
        r"float|list|tuple|optional|union|none|[\[\]|,. ]", "", (hint or "").lower()
    )
    return "float" in (hint or "") and not rest


def _floats(value):
    if isinstance(value, list):
        return [_floats(item) for item in value]
    if isinstance(value, int) and not isinstance(value, bool):
        return float(value)
    return value


def _from_js(value, field=None):
    if isinstance(value, list):
        return [_from_js(item, field) for item in value]
    if not isinstance(value, dict):
        return _floats(value) if field in _FLOAT_FIELDS else value
    type_name = value.get("__class__")
    if type_name == "MoveIt2State":
        return MoveIt2State[value["name"]]
    if type_name == "Future":
        return Future(_from_js(value.get("result")))
    fields = {k: _from_js(v, k) for k, v in value.items() if k != "__class__"}
    if type_name is None:
        type_name = _FIELD_TYPES.get(field)
        if type_name is None and field == "points":
            type_name = "JointTrajectoryPoint"
        if type_name is None:
            return fields
    return Message(type_name, fields)


def _to_js(value):
    if isinstance(value, float) and not math.isfinite(value):
        # JSON has no NaN or infinity; the bridge decodes this marker so that
        # pymoveit2's own validation rejects the value with its ValueError.
        return {"__float__": repr(value)}
    if isinstance(value, Message):
        # Tagged like the console's messages, so a `Pose` is told from a dict.
        fields = {k: _to_js(v) for k, v in value._fields.items()}
        fields["__class__"] = value._type
        return fields
    if isinstance(value, MoveIt2State):
        return {"__class__": "MoveIt2State", "name": value.name, "value": value.value}
    if isinstance(value, (list, tuple)):
        return [_to_js(item) for item in value]
    if isinstance(value, dict):
        return {str(k): _to_js(v) for k, v in value.items()}
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


_ERRORS = {
    "TypeError": TypeError,
    "ValueError": ValueError,
    "AttributeError": AttributeError,
    "RuntimeError": RuntimeError,
    "KeyError": KeyError,
    "IndexError": IndexError,
    "NotImplementedError": NotImplementedError,
    "TimeoutError": TimeoutError,
}


def _reply(raw, hint=None):
    reply = json.loads(raw)
    if "error" in reply:
        error = reply["error"]
        raise _ERRORS.get(error["type"], RuntimeError)(error["message"])
    value = _from_js(reply.get("value"))
    return _floats(value) if _float_hint(hint) else value


def _invoke(path, args, kwargs, hint=None):
    return _reply(
        _bridge.invoke(
            path, json.dumps(_to_js(list(args))), json.dumps(_to_js(dict(kwargs)))
        ),
        hint,
    )


class _Interface:
    """One pymoveit2 object of the demo, such as `robot.arm`."""

    _CLASS = "object"

    def __init__(self, path):
        object.__setattr__(self, "_path", path)
        members = json.loads(_bridge.members(path))
        object.__setattr__(self, "_members", {m["name"]: m for m in members})

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        path = f"{self._path}.{name}"
        if path in _OBJECTS:
            return _object(path)
        member = self._member(name)
        if member is not None and member.get("kind") == "method":
            return _method(path, member)
        return _reply(_bridge.getattr(path), (member or {}).get("type"))

    def _member(self, name):
        return self._members.get(name)

    def __setattr__(self, name, value):
        _reply(_bridge.setattr(f"{self._path}.{name}", json.dumps(_to_js(value))))

    def __dir__(self):
        return sorted(self._members)

    def __repr__(self):
        return f"<pymoveit2.{self._CLASS} object ({self._path})>"


class RobotSession(_Interface):
    """A connected robot: `arm`, `gripper`, `servo` and `description`."""

    _CLASS = "RobotSession"

    def _member(self, name):
        member = self._members.get(name)
        if member is None:
            # RobotSession.__getattr__ forwards what it lacks to the arm.
            return _object("robot.arm")._members.get(name)
        return member

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()
        return False


class MoveIt2(_Interface):
    """MoveIt 2 interface of the arm: planning, execution and the planning scene."""

    _CLASS = "MoveIt2"

    def __init__(self, *args, **kwargs):
        if args or kwargs:
            _not_in_browser("MoveIt2")
        super().__init__("robot.arm")


class GripperInterface(_Interface):
    """Gripper interface over the robot's MoveIt 2 gripper backend."""

    _CLASS = "GripperInterface"

    def __call__(self):
        return _invoke(self._path, (), {})


class MoveIt2Servo(_Interface):
    """MoveIt 2 Servo interface for real-time twist and joint commands."""

    _CLASS = "MoveIt2Servo"

    def __call__(self, *args, **kwargs):
        return _invoke(self._path, args, kwargs)


class RobotDescription(_Interface):
    """The robot's URDF and SRDF as discovered from `move_group`."""

    _CLASS = "RobotDescription"


MoveIt2Gripper = GripperInterface

_OBJECTS = {
    "robot": RobotSession,
    "robot.arm": MoveIt2,
    "robot.gripper": GripperInterface,
    "robot.servo": MoveIt2Servo,
    "robot.description": RobotDescription,
}
_INSTANCES = {}


def _object(path):
    if path not in _INSTANCES:
        instance = object.__new__(_OBJECTS[path])
        _Interface.__init__(instance, path)
        _INSTANCES[path] = instance
    return _INSTANCES[path]


def _method(path, member):
    def method(*args, **kwargs):
        return _invoke(path, args, kwargs, member.get("returns"))

    method.__name__ = path.rsplit(".", 1)[-1]
    method.__qualname__ = method.__name__
    signature = member.get("python_signature") or f"{method.__name__}(...)"
    method.__doc__ = f"{signature}\n\n{member.get('doc') or ''}".rstrip()
    return method


def _not_in_browser(name):
    raise NotImplementedError(
        f"The browser demo supports `connect()` only; building `{name}` from your "
        "own ROS 2 node needs real ROS 2. Use `with connect() as robot:`."
    )


def connect(*args, **kwargs):
    """Connect to the robot and return a `RobotSession` (a context manager)."""
    _reply(_bridge.connect(json.dumps(_to_js(list(args))), json.dumps(_to_js(kwargs))))
    _INSTANCES.clear()
    return _object("robot")
