"""One sentence for a failed goal of what went wrong, and what to try next."""

from typing import Any, Dict, Optional

from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes

from pymoveit2.utils import enum_to_str

ERROR_FIXES: Dict[str, str] = {
    "PLANNING_FAILED": "The planner found no collision-free path to the goal. Move the goal closer, clear what stands between start and goal, or raise `allowed_planning_time`.",
    "INVALID_MOTION_PLAN": "The planner produced a trajectory the robot cannot execute. Check the joint limits of the group.",
    "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE": "The planning scene changed after the plan was made. Plan again.",
    "CONTROL_FAILED": "The controller did not follow the trajectory. Check that a controller is active and claims the joints of the group: `ros2 control list_controllers`.",
    "UNABLE_TO_AQUIRE_SENSOR_DATA": "MoveIt did not receive the sensor data it needs. Check the depth sensor and the octomap updater of `move_group`.",
    "TIMED_OUT": "The request ran out of time. Raise `allowed_planning_time`, or the `timeout_sec` of the call.",
    "PREEMPTED": "The goal was cancelled, by `cancel_execution()` or by a newer goal.",
    "START_STATE_IN_COLLISION": "The robot is in collision where it currently stands. Move it clear by hand, or remove the object it touches from the scene.",
    "START_STATE_VIOLATES_PATH_CONSTRAINTS": "The current pose already breaks a path constraint. Drop it with `clear_path_constraints()`, or start from elsewhere.",
    "START_STATE_INVALID": "The start state is not valid for the group. Check that `joint_states` carries every joint of the group.",
    "GOAL_IN_COLLISION": "The goal pose is inside an obstacle. Pick another goal, or remove the object with `remove_collision_object()`.",
    "GOAL_VIOLATES_PATH_CONSTRAINTS": "The goal breaks a path constraint that is set. Relax the constraint, or pick a goal that satisfies it.",
    "GOAL_CONSTRAINTS_VIOLATED": "The robot stopped outside the goal tolerance. Raise the tolerance of the call, or check the controller tuning.",
    "GOAL_STATE_INVALID": "The goal state is not valid for the group. Check the joint count and the limits against `joint_names`.",
    "UNRECOGNIZED_GOAL_TYPE": "MoveIt did not recognise the kind of goal that was sent. Build the goal through `set_pose_goal()` or `set_joint_goal()`.",
    "INVALID_GROUP_NAME": "`move_group` has no planning group of that name. Check `group_name` against the groups in the SRDF.",
    "INVALID_GOAL_CONSTRAINTS": "The goal constraints are malformed. Build the goal through `set_pose_goal()` or `set_joint_goal()`.",
    "INVALID_ROBOT_STATE": "The robot state sent with the request is not valid. Check that `joint_states` is being published.",
    "INVALID_LINK_NAME": "The URDF has no link of that name. Check `end_effector_name` and `base_link_name`.",
    "INVALID_OBJECT_NAME": "The planning scene holds no object of that name. List what is there with `planning_scene`.",
    "FRAME_TRANSFORM_FAILURE": "TF could not transform between the frames of the request. Check that the TF tree is complete and current: `ros2 run tf2_tools view_frames`.",
    "COLLISION_CHECKING_UNAVAILABLE": "MoveIt cannot check collisions. Check the planning scene monitor of `move_group`.",
    "ROBOT_STATE_STALE": "The robot state is too old. Check that `joint_states` is published continuously.",
    "SENSOR_INFO_STALE": "The sensor data is too old. Check the sensor and its updater.",
    "COMMUNICATION_FAILURE": "The request did not reach `move_group`. Check that it runs, and that it shares a namespace with this node.",
    "CRASH": "MoveIt crashed while handling the request. Check the `move_group` log.",
    "ABORT": "`move_group` aborted the request. Check the `move_group` log for the reason.",
    "NO_IK_SOLUTION": "The kinematics solver found no solution for that pose. Move the pose into the workspace of the arm, or change its orientation.",
    "FAILURE": "The request failed without a specific reason. Check the `move_group` log.",
}

STATUS_FIXES: Dict[str, str] = {
    "STATUS_ABORTED": "The action server aborted the goal. Check the log of the server.",
    "STATUS_CANCELED": "The goal was cancelled before it finished.",
    "STATUS_UNKNOWN": "The action server reported no status for the goal. Check that it is still running.",
}


NON_FAILURE_VALUES = frozenset(
    getattr(MoveItErrorCodes, name)
    for name in ("SUCCESS", "UNDEFINED")
    if isinstance(getattr(MoveItErrorCodes, name, None), int)
)


def _error_value(error_code: Any) -> Optional[int]:
    if error_code is None:
        return None
    value = getattr(error_code, "val", error_code)
    return value if isinstance(value, int) else None


def describe_error_code(error_code: Any) -> Optional[str]:
    value = _error_value(error_code)
    if value is None or value in NON_FAILURE_VALUES:
        return None
    name = enum_to_str(MoveItErrorCodes, value)
    described = ERROR_FIXES.get(name)
    text = f"{name}: {described}" if described else f"{name}."
    detail = str(getattr(error_code, "message", "") or "").strip()
    source = str(getattr(error_code, "source", "") or "").strip()
    if detail:
        text = f"{text} {detail.rstrip('.')}."
    if source:
        text = f"{text} Reported by '{source}'."
    return text


def _sentence(text: str) -> str:
    stripped = text.strip().rstrip(".")
    if not stripped:
        return ""
    return f"{stripped[0].upper()}{stripped[1:]}."


def describe_status(status: Optional[int]) -> Optional[str]:
    if status is None or status == GoalStatus.STATUS_SUCCEEDED:
        return None
    name = enum_to_str(GoalStatus, status)
    described = STATUS_FIXES.get(name)
    return f"{name}: {described}" if described else f"{name}."


def describe_failure(
    status: Optional[int] = None,
    result: Any = None,
    reason: Optional[str] = None,
) -> str:
    parts = []
    described_code = describe_error_code(getattr(result, "error_code", None))
    if described_code is not None:
        parts.append(described_code)
    if reason:
        parts.append(_sentence(reason))
    if described_code is None:
        described_status = describe_status(status)
        if described_status is not None:
            parts.append(described_status)
    if not parts:
        return "The goal did not succeed and the server gave no reason. Check the `move_group` log."
    return " ".join(parts)
