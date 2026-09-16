"""Helpers for ROS planning request construction."""

import copy
from typing import Any, Optional

from geometry_msgs.msg import Pose
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan


def validate_goal_constraints(request: MotionPlanRequest) -> None:
    if not any(
        goal.joint_constraints
        or goal.position_constraints
        or goal.orientation_constraints
        or goal.visibility_constraints
        for goal in request.goal_constraints
    ):
        raise ValueError("Planning requires a goal constraint.")


def motion_plan_request(
    request: MotionPlanRequest, stamp: Any
) -> GetMotionPlan.Request:
    request = copy.deepcopy(request)
    request.workspace_parameters.header.stamp = stamp
    for constraints in request.goal_constraints:
        for constraint in constraints.position_constraints:
            constraint.header.stamp = stamp
        for constraint in constraints.orientation_constraints:
            constraint.header.stamp = stamp
    return GetMotionPlan.Request(motion_plan_request=request)


def cartesian_request(
    request: MotionPlanRequest,
    settings: GetCartesianPath.Request,
    max_step: float,
    frame_id: Optional[str],
    target_link: Optional[str],
    base_link: str,
    end_effector: str,
    stamp: Any,
) -> GetCartesianPath.Request:
    request = copy.deepcopy(request)
    goal_constraints = request.goal_constraints[-1]
    if (
        not goal_constraints.position_constraints
        or not goal_constraints.orientation_constraints
    ):
        raise ValueError(
            "Cartesian planning requires a pose goal (position and orientation); "
            "joint-space goals cannot be planned with `cartesian=True`."
        )
    position_constraint = goal_constraints.position_constraints[-1]
    orientation_constraint = goal_constraints.orientation_constraints[-1]

    cartesian_request = copy.deepcopy(settings)
    cartesian_request.start_state = request.start_state

    if hasattr(cartesian_request, "max_velocity_scaling_factor"):
        cartesian_request.max_velocity_scaling_factor = (
            request.max_velocity_scaling_factor
        )
    if hasattr(cartesian_request, "max_acceleration_scaling_factor"):
        cartesian_request.max_acceleration_scaling_factor = (
            request.max_acceleration_scaling_factor
        )

    cartesian_request.group_name = request.group_name
    cartesian_request.link_name = (
        target_link
        if target_link is not None
        else (position_constraint.link_name or end_effector)
    )
    cartesian_request.max_step = float(max_step)

    stored_frame = position_constraint.header.frame_id or base_link
    orientation_frame = orientation_constraint.header.frame_id or base_link
    stored_link = position_constraint.link_name or end_effector
    orientation_link = orientation_constraint.link_name or end_effector
    if stored_frame != orientation_frame or (
        frame_id is not None and frame_id != stored_frame
    ):
        raise ValueError("Cartesian position and orientation must use the same frame.")
    if stored_link != orientation_link or (
        target_link is not None and target_link != stored_link
    ):
        raise ValueError(
            "Cartesian position and orientation must use the same target link."
        )
    if not position_constraint.constraint_region.primitive_poses:
        raise ValueError("Cartesian position constraint must contain a pose goal.")
    cartesian_request.header.frame_id = stored_frame

    cartesian_request.header.stamp = stamp

    cartesian_request.path_constraints = request.path_constraints
    for constraint in cartesian_request.path_constraints.position_constraints:
        constraint.header.stamp = stamp
    for constraint in cartesian_request.path_constraints.orientation_constraints:
        constraint.header.stamp = stamp

    target_pose = Pose()
    target_pose.position = position_constraint.constraint_region.primitive_poses[
        0
    ].position
    target_pose.orientation = orientation_constraint.orientation

    cartesian_request.waypoints = [target_pose]
    return cartesian_request
