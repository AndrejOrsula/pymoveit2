"""
Derivation of the interface configuration from the robot description.
The presets under `pymoveit2.robots` hard-code the joint names, links and gripper positions of a specific robot.
`RobotDescription` derives the same information from the URDF and SRDF that `move_group` is already configured with.
"""

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple
from xml.etree import ElementTree

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

__all__ = [
    "MoveGroupDescription",
    "RobotDescription",
]

DEFAULT_DESCRIPTION_NODE_NAME: str = "move_group"
DEFAULT_URDF_PARAMETER: str = "robot_description"
DEFAULT_SRDF_PARAMETER: str = "robot_description_semantic"

OPEN_STATE_NAMES: Tuple[str, ...] = ("open", "opened", "open_gripper", "gripper_open")
CLOSED_STATE_NAMES: Tuple[str, ...] = (
    "close",
    "closed",
    "close_gripper",
    "gripper_close",
    "gripper_closed",
)
GRIPPER_NAME_HINTS: Tuple[str, ...] = ("gripper", "hand", "eef", "end_effector")
ARM_STATE_NAMES: Tuple[str, ...] = (
    "ready",
    "home",
    "default",
    "start",
    "rest",
    "up",
    "zero",
)


@dataclass(frozen=True)
class MoveGroupDescription:
    name: str
    joint_names: List[str] = field(default_factory=list)
    passive_joint_names: List[str] = field(default_factory=list)
    link_names: List[str] = field(default_factory=list)
    base_link_name: str = ""
    end_effector_name: str = ""
    subgroup_names: List[str] = field(default_factory=list)
    named_states: Dict[str, Dict[str, float]] = field(default_factory=dict)

    @property
    def movable_joint_names(self) -> List[str]:
        ordered = getattr(self, "_ordered_movable_joint_names", None)
        if ordered:
            return list(ordered)
        return [*self.joint_names, *self.passive_joint_names]

    def state_positions(
        self, state_name: str, joint_names: Optional[Sequence[str]] = None
    ) -> List[float]:
        if state_name not in self.named_states:
            raise ValueError(
                f"Group '{self.name}' has no state '{state_name}'"
                f" (known: {sorted(self.named_states)})."
            )
        state = self.named_states[state_name]
        names = self.joint_names if joint_names is None else joint_names
        return [float(state.get(joint_name, 0.0)) for joint_name in names]


class RobotDescription:
    """
    Robot configuration parsed from a URDF and its SRDF.

    ```python
    description = RobotDescription.from_node(node)
    moveit2 = MoveIt2(node=node, callback_group=callback_group, **description.moveit2_kwargs())
    ```
    """

    def __init__(self, urdf_xml: str, srdf_xml: str):
        self._urdf = _UrdfModel(urdf_xml)
        srdf = _parse_xml(srdf_xml, "SRDF")

        self._name: str = srdf.get("name", self._urdf.name)
        self._elements = self._parse_groups(srdf)

        self._passive_joints = {
            joint.get("name")
            for joint in srdf.iter("passive_joint")
            if joint.get("name")
        }
        self._virtual_joints = self._parse_virtual_joints(srdf)
        self._states = self._parse_states(srdf)
        self._end_effectors = self._parse_end_effectors(srdf)

        group_order = self._validate_group_graph()
        self._groups: Dict[str, MoveGroupDescription] = {}
        for group_name in group_order:
            self._groups[group_name] = self._describe_group(group_name, ())

    @classmethod
    def from_node(
        cls,
        node: Node,
        remote_node_name: str = DEFAULT_DESCRIPTION_NODE_NAME,
        timeout_sec: Optional[float] = 10.0,
        callback_group: Optional[CallbackGroup] = None,
        urdf_parameter: str = DEFAULT_URDF_PARAMETER,
        srdf_parameter: str = DEFAULT_SRDF_PARAMETER,
    ) -> "RobotDescription":
        """
        Fetch the URDF and SRDF from the parameters of `remote_node_name` (relative to the namespace of `node` unless it is absolute)
        ."""

        deadline = _Deadline(timeout_sec)
        service_name = f"{remote_node_name.rstrip('/')}/get_parameters"
        response = _fetch_parameter_response(
            node,
            service_name,
            urdf_parameter,
            srdf_parameter,
            callback_group,
            deadline,
        )

        try:
            values = response.values
            if response is None or len(values) != 2:
                raise ValueError
        except (AttributeError, TypeError, ValueError) as ex:
            raise RuntimeError(f"Invalid response from '{service_name}'.") from ex
        descriptions = []
        for parameter, value in zip(
            (urdf_parameter, srdf_parameter), values, strict=False
        ):
            try:
                valid = value.type == ParameterType.PARAMETER_STRING and bool(
                    value.string_value
                )
            except AttributeError:
                valid = False
            if not valid:
                raise RuntimeError(
                    f"Parameter '{parameter}' of '{remote_node_name}' is not set."
                )
            descriptions.append(value.string_value)
        return cls(*descriptions)

    @property
    def name(self) -> str:
        return self._name

    @property
    def group_names(self) -> List[str]:
        return list(self._groups)

    @property
    def groups(self) -> Dict[str, MoveGroupDescription]:
        return {
            group_name: _copy_group_description(group)
            for group_name, group in self._groups.items()
        }

    def group(self, group_name: str) -> MoveGroupDescription:
        if group_name not in self._groups:
            raise ValueError(
                f"Unknown group '{group_name}' (known: {sorted(self._groups)})."
            )
        return _copy_group_description(self._groups[group_name])

    @property
    def gripper_group_name(self) -> Optional[str]:
        end_effector_groups = list(
            dict.fromkeys(
                group_name
                for group_name, _, _ in self._end_effectors
                if group_name in self._groups
            )
        )
        if len(end_effector_groups) > 1:
            raise ValueError(
                "Multiple SRDF end-effector groups are available "
                f"({', '.join(end_effector_groups)}); pass group_name explicitly."
            )
        if end_effector_groups:
            return end_effector_groups[0]

        hinted_groups = [
            group_name
            for group_name, group in self._groups.items()
            if group.joint_names
            and any(hint in group_name.lower() for hint in GRIPPER_NAME_HINTS)
        ]
        if len(hinted_groups) > 1:
            raise ValueError(
                "Multiple gripper groups are available "
                f"({', '.join(hinted_groups)}); pass group_name explicitly."
            )
        if hinted_groups:
            return hinted_groups[0]
        return None

    @property
    def arm_group_name(self) -> Optional[str]:
        parent_groups = list(
            dict.fromkeys(
                parent_group_name
                for _, parent_group_name, _ in self._end_effectors
                if parent_group_name in self._groups
            )
        )
        if len(parent_groups) == 1:
            return parent_groups[0]
        if len(parent_groups) > 1:
            raise ValueError(
                "Multiple end-effector parent groups are available "
                f"({', '.join(parent_groups)}); pass group_name explicitly."
            )

        gripper_group_name = self.gripper_group_name
        gripper_joints = (
            set(self._groups[gripper_group_name].movable_joint_names)
            if gripper_group_name is not None
            else set()
        )
        candidates = [
            group
            for group in self._groups.values()
            if group.joint_names
            and gripper_joints.isdisjoint(group.movable_joint_names)
        ]
        if not candidates:
            return None
        largest_size = max(len(group.joint_names) for group in candidates)
        largest = [
            group for group in candidates if len(group.joint_names) == largest_size
        ]
        if len(largest) > 1:
            raise ValueError(
                "Multiple arm groups have the largest joint set "
                f"({', '.join(group.name for group in largest)}); "
                "pass group_name explicitly."
            )
        return largest[0].name

    def joint_positions(
        self,
        state_name: Optional[str] = None,
        group_name: Optional[str] = None,
    ) -> List[float]:
        group = self.group(self._arm_group_name(group_name))
        if state_name is None and len(group.named_states) == 1:
            state_name = next(iter(group.named_states))
        return group.state_positions(
            _select_state(group, state_name, ARM_STATE_NAMES, "default")
        )

    def moveit2_kwargs(self, group_name: Optional[str] = None) -> Dict[str, object]:
        """
        Keyword arguments that configure `MoveIt2` for the arm group.
        """

        group = self.group(self._arm_group_name(group_name))
        return {
            "joint_names": list(group.joint_names),
            "base_link_name": group.base_link_name,
            "end_effector_name": group.end_effector_name,
            "group_name": group.name,
        }

    def _arm_group_name(self, group_name: Optional[str]) -> str:
        if group_name is not None:
            return group_name
        group_name = self.arm_group_name
        if group_name is None:
            raise ValueError(
                "Unable to determine the arm group, pass `group_name` explicitly."
            )
        return group_name

    def moveit2_gripper_kwargs(
        self,
        group_name: Optional[str] = None,
        open_state: Optional[str] = None,
        closed_state: Optional[str] = None,
    ) -> Dict[str, object]:
        """
        Keyword arguments that configure `MoveIt2Gripper` or the planning backend of `GripperInterface` from the SRDF group states of the gripper group.
        """

        if group_name is None:
            group_name = self.gripper_group_name
            if group_name is None:
                raise ValueError(
                    "Unable to determine the gripper group, pass `group_name` explicitly."
                )
        group = self.group(group_name)
        open_state = _select_state(group, open_state, OPEN_STATE_NAMES, "open")
        closed_state = _select_state(group, closed_state, CLOSED_STATE_NAMES, "closed")

        mentioned = set(group.named_states[open_state]).intersection(
            group.named_states[closed_state]
        )
        joint_names = [
            joint_name
            for joint_name in group.movable_joint_names
            if joint_name in mentioned
        ] or list(group.joint_names)
        return {
            "gripper_joint_names": joint_names,
            "open_gripper_joint_positions": group.state_positions(
                open_state, joint_names
            ),
            "closed_gripper_joint_positions": group.state_positions(
                closed_state, joint_names
            ),
            "gripper_group_name": group.name,
        }

    def _parse_groups(
        self, srdf: ElementTree.Element
    ) -> Dict[str, ElementTree.Element]:
        elements: Dict[str, ElementTree.Element] = {}
        for group in srdf.findall("group"):
            group_name = group.get("name")
            if not group_name:
                raise ValueError("The SRDF contains a planning group without a name.")
            if group_name in elements:
                raise ValueError(
                    f"The SRDF defines duplicate planning group '{group_name}'."
                )
            elements[group_name] = group
        if not elements:
            raise ValueError("The SRDF does not define any planning group.")
        return elements

    def _parse_virtual_joints(self, srdf: ElementTree.Element) -> Dict[str, str]:
        virtual_joints: Dict[str, str] = {}
        for joint in srdf.iter("virtual_joint"):
            joint_name = joint.get("name")
            child_link = joint.get("child_link", "")
            if not joint_name:
                raise ValueError("The SRDF contains a virtual joint without a name.")
            if joint_name in virtual_joints:
                raise ValueError(
                    f"The SRDF defines duplicate virtual joint '{joint_name}'."
                )
            if not self._urdf.has_link(child_link):
                raise ValueError(
                    f"The SRDF virtual joint '{joint_name}' refers to unknown link"
                    f" '{child_link}'."
                )
            virtual_joints[joint_name] = child_link
        return virtual_joints

    def _parse_states(
        self, srdf: ElementTree.Element
    ) -> Dict[str, Dict[str, Dict[str, float]]]:
        states: Dict[str, Dict[str, Dict[str, float]]] = {}
        for state in srdf.findall("group_state"):
            group_name, state_name = state.get("group"), state.get("name")
            if not group_name or not state_name:
                raise ValueError(
                    "The SRDF contains a group state without a group or name."
                )
            if group_name not in self._elements:
                raise ValueError(
                    f"The SRDF group state '{state_name}' refers to unknown group"
                    f" '{group_name}'."
                )
            if state_name in states.get(group_name, {}):
                raise ValueError(
                    f"The SRDF defines duplicate state '{state_name}' for group"
                    f" '{group_name}'."
                )
            positions: Dict[str, float] = {}
            for joint in state.findall("joint"):
                joint_name = joint.get("name")
                if not joint_name:
                    raise ValueError(
                        f"The SRDF state '{state_name}' contains a joint without a name."
                    )
                if joint_name in positions:
                    raise ValueError(
                        f"The SRDF state '{state_name}' repeats joint '{joint_name}'."
                    )
                position = _finite_state_value(
                    joint.get("value", 0.0), state_name, joint_name
                )
                if (
                    not self._urdf.has_joint(joint_name)
                    and joint_name not in self._virtual_joints
                ):
                    raise ValueError(
                        f"The SRDF state '{state_name}' refers to unknown joint"
                        f" '{joint_name}'."
                    )
                positions[joint_name] = position
            states.setdefault(group_name, {})[state_name] = positions
        return states

    def _parse_end_effectors(
        self, srdf: ElementTree.Element
    ) -> List[Tuple[str, str, str]]:
        end_effectors = [
            (
                end_effector.get("group", ""),
                end_effector.get("parent_group", ""),
                end_effector.get("parent_link", ""),
            )
            for end_effector in srdf.findall("end_effector")
        ]
        for effector_group, parent_group, parent_link in end_effectors:
            if effector_group not in self._elements:
                raise ValueError(
                    f"The SRDF end effector refers to unknown group '{effector_group}'."
                )
            if parent_group and parent_group not in self._elements:
                raise ValueError(
                    f"The SRDF end effector refers to unknown parent group"
                    f" '{parent_group}'."
                )
            if parent_link and not self._urdf.has_link(parent_link):
                raise ValueError(
                    f"The SRDF end effector refers to unknown parent link"
                    f" '{parent_link}'."
                )
        return end_effectors

    def _validate_group_graph(self) -> List[str]:
        adjacency: Dict[str, List[str]] = {}
        for group_name, element in self._elements.items():
            children: List[str] = []
            for child in element:
                if child.tag != "group":
                    continue
                subgroup_name = child.get("name")
                if not subgroup_name:
                    raise ValueError(
                        f"The SRDF group '{group_name}' contains a subgroup without a name."
                    )
                if subgroup_name not in self._elements:
                    raise ValueError(
                        f"The SRDF group '{group_name}' refers to unknown subgroup"
                        f" '{subgroup_name}'."
                    )
                if subgroup_name in children:
                    raise ValueError(
                        f"The SRDF group '{group_name}' repeats subgroup"
                        f" '{subgroup_name}'."
                    )
                children.append(subgroup_name)
            adjacency[group_name] = children

        state: Dict[str, int] = {group_name: 0 for group_name in self._elements}
        order: List[str] = []
        for root in self._elements:
            if state[root] != 0:
                continue
            state[root] = 1
            path: List[str] = [root]
            stack: List[Tuple[str, int]] = [(root, 0)]
            while stack:
                group_name, child_index = stack[-1]
                children = adjacency[group_name]
                if child_index == len(children):
                    state[group_name] = 2
                    stack.pop()
                    path.pop()
                    order.append(group_name)
                    continue
                subgroup_name = children[child_index]
                stack[-1] = (group_name, child_index + 1)
                if state[subgroup_name] == 2:
                    continue
                if state[subgroup_name] == 1:
                    try:
                        cycle_start = path.index(subgroup_name)
                    except ValueError:
                        cycle_start = 0
                    cycle = [*path[cycle_start:], subgroup_name]
                    raise ValueError(
                        f"The SRDF subgroup graph contains a cycle"
                        f" ({' -> '.join(cycle)}); group '{subgroup_name}'"
                        " contains itself as a subgroup."
                    )
                state[subgroup_name] = 1
                path.append(subgroup_name)
                stack.append((subgroup_name, 0))
        return order

    def _describe_group(
        self, group_name: str, stack: Tuple[str, ...]
    ) -> MoveGroupDescription:
        if group_name in stack:
            raise ValueError(
                f"The SRDF group '{group_name}' contains itself as a subgroup."
            )
        joints, links, subgroup_names, chain_endpoints = self._collect_members(
            group_name, stack
        )

        link_names = self._urdf.sort_links(links)
        joint_names, passive_joint_names = [], []
        ordered_movable_joint_names = []
        for joint_name in self._urdf.sort_joints(joints):
            if not self._urdf.is_movable(joint_name):
                continue
            ordered_movable_joint_names.append(joint_name)
            if joint_name in self._passive_joints or self._urdf.is_mimic(joint_name):
                passive_joint_names.append(joint_name)
            else:
                joint_names.append(joint_name)

        base_link_name, end_effector_name = self._resolve_endpoints(
            group_name, link_names, chain_endpoints
        )
        description = MoveGroupDescription(
            name=group_name,
            joint_names=joint_names,
            passive_joint_names=passive_joint_names,
            link_names=link_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            subgroup_names=subgroup_names,
            named_states={
                state_name: dict(positions)
                for state_name, positions in self._states.get(group_name, {}).items()
            },
        )
        object.__setattr__(
            description, "_ordered_movable_joint_names", ordered_movable_joint_names
        )
        return description

    def _collect_members(
        self, group_name: str, stack: Tuple[str, ...]
    ) -> Tuple[List[str], List[str], List[str], Optional[Tuple[str, str]]]:
        joints: List[str] = []
        links: List[str] = []
        subgroup_names: List[str] = []
        chain_endpoints: Optional[Tuple[str, str]] = None

        for child in self._elements[group_name]:
            name = child.get("name", "")
            if child.tag in ("joint", "passive_joint") and not name:
                raise ValueError(
                    f"The SRDF group '{group_name}' contains a joint without a name."
                )
            if child.tag == "link" and not name:
                raise ValueError(
                    f"The SRDF group '{group_name}' contains a link without a name."
                )
            if child.tag in ("joint", "passive_joint") and name in self._virtual_joints:
                virtual_link = self._virtual_joints[name]
                if self._urdf.has_link(virtual_link):
                    links.append(virtual_link)
            elif child.tag in ("joint", "passive_joint") and name:
                joints.append(name)
                links.append(self._urdf.child_link(name))
            elif child.tag == "link" and name:
                links.append(name)
                parent_joint = self._urdf.parent_joint(name)
                if parent_joint is not None:
                    joints.append(parent_joint)
            elif child.tag == "chain":
                base_link = child.get("base_link", "")
                tip_link = child.get("tip_link", "")
                chain_joints, chain_links = self._urdf.chain(base_link, tip_link)
                joints.extend(chain_joints)
                links.extend(chain_links)
                if chain_endpoints is None:
                    chain_endpoints = (base_link, tip_link)
            elif child.tag == "group" and name:
                subgroup = self._subgroup(group_name, name, stack)
                subgroup_names.append(name)
                joints.extend(subgroup.movable_joint_names)
                links.extend(subgroup.link_names)

        return joints, links, subgroup_names, chain_endpoints

    def _subgroup(
        self, group_name: str, subgroup_name: str, stack: Tuple[str, ...]
    ) -> MoveGroupDescription:
        if subgroup_name not in self._elements:
            raise ValueError(
                f"The SRDF group '{group_name}' refers to unknown subgroup"
                f" '{subgroup_name}'."
            )
        subgroup = self._groups.get(subgroup_name) or self._describe_group(
            subgroup_name, (*stack, group_name)
        )
        self._groups.setdefault(subgroup_name, subgroup)
        return subgroup

    def _resolve_endpoints(
        self,
        group_name: str,
        link_names: List[str],
        chain_endpoints: Optional[Tuple[str, str]],
    ) -> Tuple[str, str]:
        if chain_endpoints is not None:
            base_link_name, end_effector_name = chain_endpoints
        elif link_names:
            base_link_name, end_effector_name = link_names[0], link_names[-1]
        else:
            base_link_name = end_effector_name = self._urdf.root_link
        for effector_group, parent_group, parent_link in self._end_effectors:
            if not parent_link:
                continue
            if parent_group == group_name and chain_endpoints is None:
                end_effector_name = parent_link
            elif effector_group == group_name and not link_names:
                base_link_name = parent_link
        return base_link_name, end_effector_name


class _UrdfModel:
    def __init__(self, urdf_xml: str):
        root = _parse_xml(urdf_xml, "URDF")
        self.name: str = root.get("name", "")
        self._joint_order: Dict[str, int] = {}
        self._joint_types: Dict[str, str] = {}
        self._mimic_joints: set[str] = set()
        self._mimic_targets: Dict[str, str] = {}
        self._parent_links: Dict[str, str] = {}
        self._child_links: Dict[str, str] = {}
        self._parent_joints: Dict[str, str] = {}
        self._link_order = self._parse_links(root)
        children = self._parse_joints(root)

        self._validate_mimic_graph()
        self._validate_kinematic_graph(children)
        self.root_link, self._depths = self._derive_depths(children)

    def _parse_links(self, root: ElementTree.Element) -> Dict[str, int]:
        link_order: Dict[str, int] = {}
        for index, link in enumerate(root.findall("link")):
            name = link.get("name")
            if not name:
                raise ValueError("The URDF contains a link without a name.")
            if name in link_order:
                raise ValueError(f"The URDF defines duplicate link '{name}'.")
            link_order[name] = index
        if not link_order:
            raise ValueError("The URDF does not define any link.")
        return link_order

    def _parse_joints(self, root: ElementTree.Element) -> Dict[str, List[str]]:
        children: Dict[str, List[str]] = {link: [] for link in self._link_order}
        for index, joint in enumerate(root.findall("joint")):
            self._parse_joint(index, joint, children)
        return children

    def _parse_joint(
        self,
        index: int,
        joint: ElementTree.Element,
        children: Dict[str, List[str]],
    ) -> None:
        name = joint.get("name")
        if not name:
            raise ValueError("The URDF contains a joint without a name.")
        if name in self._joint_order:
            raise ValueError(f"The URDF defines duplicate joint '{name}'.")
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(
                f"The URDF joint '{name}' must define both parent and child links."
            )
        parent_link, child_link = parent.get("link", ""), child.get("link", "")
        if not parent_link or not child_link:
            raise ValueError(
                f"The URDF joint '{name}' has an empty parent or child link."
            )
        if parent_link not in self._link_order:
            raise ValueError(
                f"The URDF joint '{name}' refers to unknown parent link"
                f" '{parent_link}'."
            )
        if child_link not in self._link_order:
            raise ValueError(
                f"The URDF joint '{name}' refers to unknown child link '{child_link}'."
            )
        if child_link in self._parent_joints:
            previous = self._parent_joints[child_link]
            raise ValueError(
                f"The URDF link '{child_link}' has multiple parent joints"
                f" ('{previous}' and '{name}')."
            )
        self._joint_order[name] = index
        self._joint_types[name] = joint.get("type", "fixed")
        self._parent_links[name] = parent_link
        self._child_links[name] = child_link
        self._parent_joints[child_link] = name
        children[parent_link].append(name)

        mimic = joint.find("mimic")
        if mimic is not None:
            target = mimic.get("joint", "")
            if not target:
                raise ValueError(f"The URDF mimic joint '{name}' has no target joint.")
            self._mimic_joints.add(name)
            self._mimic_targets[name] = target

    def _derive_depths(
        self, children: Dict[str, List[str]]
    ) -> Tuple[str, Dict[str, int]]:
        roots = [link for link in self._link_order if link not in self._parent_joints]
        if len(roots) != 1:
            if not roots:
                raise ValueError("The URDF kinematic tree has no root link.")
            raise ValueError(
                "The URDF kinematic graph must have exactly one root link; found "
                f"{', '.join(roots)}."
            )
        root_link = roots[0]
        depths = {root_link: 0}
        queue: List[str] = [root_link]
        for link in queue:
            for joint_name in sorted(children[link]):
                child_link = self._child_links[joint_name]
                depths[child_link] = depths[link] + 1
                queue.append(child_link)
        if len(depths) != len(self._link_order):
            missing = sorted(set(self._link_order) - set(depths))
            raise ValueError(
                "The URDF kinematic graph is disconnected; unreachable links: "
                f"{', '.join(missing)}."
            )
        return root_link, depths

    def _validate_mimic_graph(self) -> None:
        for joint_name, target in self._mimic_targets.items():
            if target not in self._joint_order:
                raise ValueError(
                    f"The URDF mimic joint '{joint_name}' refers to unknown joint"
                    f" '{target}'."
                )

        state: Dict[str, int] = {name: 0 for name in self._mimic_targets}
        for start in self._mimic_targets:
            if state[start] != 0:
                continue
            state[start] = 1
            path: List[str] = [start]
            current = start
            while current in self._mimic_targets:
                target = self._mimic_targets[current]
                if target not in self._mimic_targets:
                    state[current] = 2
                    path.pop()
                    break
                if state[target] == 2:
                    for item in path:
                        state[item] = 2
                    break
                if state[target] == 1:
                    try:
                        cycle_start = path.index(target)
                    except ValueError:
                        cycle_start = 0
                    cycle = [*path[cycle_start:], target]
                    raise ValueError(
                        f"The URDF mimic graph contains a cycle: {' -> '.join(cycle)}."
                    )
                state[target] = 1
                path.append(target)
                current = target
            for item in path:
                state[item] = 2

    def _validate_kinematic_graph(self, children: Dict[str, List[str]]) -> None:
        color: Dict[str, int] = {link: 0 for link in self._link_order}
        for root in self._link_order:
            if color[root] != 0:
                continue
            color[root] = 1
            stack: List[Tuple[str, int]] = [(root, 0)]
            while stack:
                link, child_index = stack[-1]
                joints = children[link]
                if child_index == len(joints):
                    color[link] = 2
                    stack.pop()
                    continue
                joint_name = joints[child_index]
                stack[-1] = (link, child_index + 1)
                child_link = self._child_links[joint_name]
                if color[child_link] == 1:
                    raise ValueError(
                        "The URDF kinematic graph contains a cycle involving link "
                        f"'{child_link}'."
                    )
                if color[child_link] == 0:
                    color[child_link] = 1
                    stack.append((child_link, 0))

    def has_link(self, link_name: str) -> bool:
        return link_name in self._link_order

    def has_joint(self, joint_name: str) -> bool:
        return joint_name in self._joint_order

    def is_movable(self, joint_name: str) -> bool:
        return self._joint_types.get(joint_name, "fixed") != "fixed"

    def is_mimic(self, joint_name: str) -> bool:
        return joint_name in self._mimic_joints

    def child_link(self, joint_name: str) -> str:
        if joint_name not in self._child_links:
            raise ValueError(f"The URDF does not define joint '{joint_name}'.")
        return self._child_links[joint_name]

    def parent_joint(self, link_name: str) -> Optional[str]:
        if link_name not in self._link_order:
            raise ValueError(f"The URDF does not define link '{link_name}'.")
        return self._parent_joints.get(link_name)

    def chain(self, base_link: str, tip_link: str) -> Tuple[List[str], List[str]]:
        for link in (base_link, tip_link):
            if link not in self._link_order:
                raise ValueError(f"The URDF does not define link '{link}'.")
        joints: List[str] = []
        links: List[str] = [tip_link]
        link = tip_link
        visited: set[str] = set()
        while link != base_link:
            if link in visited or len(visited) >= len(self._link_order):
                raise ValueError(
                    f"The URDF chain from '{base_link}' to '{tip_link}' is cyclic."
                )
            visited.add(link)
            joint = self._parent_joints.get(link)
            if joint is None:
                raise ValueError(
                    f"Link '{tip_link}' is not a descendant of '{base_link}'."
                )
            joints.append(joint)
            link = self._parent_links[joint]
            links.append(link)
        joints.reverse()
        links.reverse()
        return joints, links

    def sort_joints(self, joint_names: Sequence[str]) -> List[str]:
        unique = list(dict.fromkeys(joint_names))
        for joint_name in unique:
            if joint_name not in self._joint_order:
                raise ValueError(f"The URDF does not define joint '{joint_name}'.")

        selected = set(unique)
        dependents: Dict[str, List[str]] = {name: [] for name in unique}
        indegree: Dict[str, int] = {name: 0 for name in unique}
        for joint_name in unique:
            target = self._mimic_targets.get(joint_name)
            if target in selected:
                dependents[target].append(joint_name)
                indegree[joint_name] += 1

        def key(joint_name: str) -> Tuple[int, str]:
            return self._depths[self._child_links[joint_name]], joint_name

        available = sorted(
            (name for name, degree in indegree.items() if degree == 0), key=key
        )
        ordered: List[str] = []
        while available:
            joint_name = available.pop(0)
            ordered.append(joint_name)
            for dependent in sorted(dependents[joint_name], key=key):
                indegree[dependent] -= 1
                if indegree[dependent] == 0:
                    available.append(dependent)
            available.sort(key=key)
        if len(ordered) != len(unique):
            raise ValueError(
                "The selected URDF joints contain a cyclic mimic dependency."
            )
        return ordered

    def sort_links(self, link_names: Sequence[str]) -> List[str]:
        unique = dict.fromkeys(link_names)
        for link in unique:
            if link not in self._link_order:
                raise ValueError(f"The URDF does not define link '{link}'.")

        def key(link: str) -> Tuple[int, str, str]:
            parent_joint = self._parent_joints.get(link, "")
            return self._depths[link], parent_joint, link

        return sorted(unique, key=key)


def _parse_xml(xml: str, kind: str) -> ElementTree.Element:
    try:
        root = ElementTree.fromstring(xml)
    except ElementTree.ParseError as ex:
        raise ValueError(f"Unable to parse the {kind}: {ex}") from ex
    if root.tag != "robot":
        raise ValueError(f"The {kind} root element is '{root.tag}' instead of 'robot'.")
    return root


def _select_state(
    group: MoveGroupDescription,
    state_name: Optional[str],
    candidates: Sequence[str],
    kind: str,
) -> str:
    if state_name is not None:
        if state_name not in group.named_states:
            raise ValueError(
                f"Group '{group.name}' has no state '{state_name}'"
                f" (known: {sorted(group.named_states)})."
            )
        return state_name
    for candidate in candidates:
        if candidate in group.named_states:
            return candidate
    if not group.named_states:
        raise ValueError(f"The SRDF defines no state for group '{group.name}'.")
    raise ValueError(
        f"Group '{group.name}' defines no {kind} state"
        f" (known: {sorted(group.named_states)}), pass the state name explicitly."
    )


def _finite_state_value(value: object, state_name: str, joint_name: str) -> float:
    try:
        position = float(value)
    except (TypeError, ValueError) as ex:
        raise ValueError(
            f"The SRDF state '{state_name}' has an invalid value for"
            f" joint '{joint_name}'."
        ) from ex
    if not math.isfinite(position):
        raise ValueError(
            f"The SRDF state '{state_name}' has a non-finite value for"
            f" joint '{joint_name}'."
        )
    return position


def _fetch_parameter_response(
    node: Node,
    service_name: str,
    urdf_parameter: str,
    srdf_parameter: str,
    callback_group: Optional[CallbackGroup],
    deadline: "_Deadline",
) -> object:
    client = None
    future = None
    try:
        client = node.create_client(
            GetParameters, service_name, callback_group=callback_group
        )
        if not client.wait_for_service(timeout_sec=deadline.remaining()):
            raise TimeoutError(f"Service '{service_name}' is not available.")
        if deadline.expired() and deadline.has_positive_budget:
            raise TimeoutError(f"Timed out while discovering '{service_name}'.")
        try:
            future = client.call_async(
                GetParameters.Request(names=[urdf_parameter, srdf_parameter])
            )
        except (RuntimeError, AttributeError, TypeError) as ex:
            raise RuntimeError(
                f"Unable to call '{service_name}': {type(ex).__name__}: {ex}"
            ) from ex
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=deadline.remaining()):
            _cancel_pending_request(client, future)
            raise TimeoutError(f"Timed out while calling '{service_name}'.")
        try:
            return future.result()
        except Exception as ex:
            raise RuntimeError(
                f"Unable to receive '{service_name}' response: "
                f"{type(ex).__name__}: {ex}"
            ) from ex
    except TimeoutError:
        raise
    except (RuntimeError, AttributeError, TypeError) as ex:
        if client is not None and future is not None:
            _cancel_pending_request(client, future)
        raise RuntimeError(
            f"Unable to query '{service_name}': {type(ex).__name__}: {ex}"
        ) from ex
    finally:
        if client is not None:
            try:
                node.destroy_client(client)
            except (RuntimeError, AttributeError, TypeError):
                pass


class _Deadline:
    def __init__(self, timeout_sec: Optional[float]):
        self.has_positive_budget = False
        if timeout_sec is None:
            self._deadline: Optional[float] = None
            return
        try:
            timeout = float(timeout_sec)
        except (TypeError, ValueError) as ex:
            raise ValueError("timeout_sec must be a finite number or None.") from ex
        if not math.isfinite(timeout):
            raise ValueError("timeout_sec must be finite or None.")
        self.has_positive_budget = timeout > 0.0
        self._deadline = time.monotonic() + max(0.0, timeout)

    def remaining(self) -> Optional[float]:
        if self._deadline is None:
            return None
        return max(0.0, self._deadline - time.monotonic())

    def expired(self) -> bool:
        return self._deadline is not None and time.monotonic() >= self._deadline


def _cancel_pending_request(client: object, future: object) -> None:
    try:
        done = getattr(future, "done", None)
        if callable(done) and done():
            return
        cancel = getattr(future, "cancel", None)
        if callable(cancel):
            cancel()
    except (RuntimeError, AttributeError, TypeError):
        pass
    try:
        remove = getattr(client, "remove_pending_request", None)
        if callable(remove):
            remove(future)
    except (RuntimeError, AttributeError, TypeError):
        pass


def _copy_group_description(group: MoveGroupDescription) -> MoveGroupDescription:
    copied = MoveGroupDescription(
        name=group.name,
        joint_names=list(group.joint_names),
        passive_joint_names=list(group.passive_joint_names),
        link_names=list(group.link_names),
        base_link_name=group.base_link_name,
        end_effector_name=group.end_effector_name,
        subgroup_names=list(group.subgroup_names),
        named_states={
            state_name: dict(positions)
            for state_name, positions in group.named_states.items()
        },
    )
    object.__setattr__(
        copied, "_ordered_movable_joint_names", list(group.movable_joint_names)
    )
    return copied
