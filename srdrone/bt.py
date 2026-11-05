"""Deterministic validator for the audited subset of the public ROS1 controller.

This checks syntax, ports, bounded control flow and conservative blackboard dataflow.
It is not a proof of mission success or physical safety.
"""

from __future__ import annotations

import hashlib
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field

# Only interfaces audited against controller/src/ros1_node.cpp are exposed.
# port -> (type, direction); optional outputs need not be connected.
ACTIONS = {
    "TakeOff": {"takeoff_target": ("position", "in")},
    "FlyToCoordinates": {"coordinates": ("position", "in")},
    **{
        f"Move{d}": {f"Move{d}_distance": ("positive", "in")}
        for d in ("Forward", "Back", "Left", "Right", "Up", "Down")
    },
    "SetYawAngle": {"yaw": ("number", "in")},
    "TurnLeft": {"turn_attitude": ("positive", "in")},
    "TurnRight": {"turn_attitude": ("positive", "in")},
    "Land": {},
    "ForwardDetectFrame": {
        "FramePosition": ("position", "out"),
        "ForwardDetectFrameData": ("objects", "out"),
    },
    "DownDetectH": {
        "DownDetectHData": ("object", "out"),
        "DownDetectHDxy": ("dxy", "out"),
        "SendVelocity": ("velocity", "out"),
    },
}
LOGIC = {
    "Sequence": {},
    "Fallback": {},
    "Inverter": {},
    "Repeat": {"num_cycles": ("count", "in")},
    "RetryUntilSuccessful": {"num_attempts": ("count", "in")},
}
REF = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}\Z")


@dataclass
class ValidationResult:
    valid: bool
    errors: list[str] = field(default_factory=list)
    node_count: int = 0
    profile: str = "public-controller-subset-v1"

    def to_dict(self) -> dict:
        return asdict(self)


def parse_xml(xml: str) -> ET.Element:
    if not isinstance(xml, str) or len(xml) > 200_000:
        raise ValueError("Plan must be XML under 200 kB")
    if re.search(r"<!\s*(DOCTYPE|ENTITY)", xml, re.I):
        raise ValueError("DTD/entities are forbidden")
    return ET.fromstring(xml)


def plan_digest(xml: str) -> str:
    """Whitespace/attribute-order independent digest; preserves child order."""

    def canonical(node):
        return (
            node.tag,
            tuple(sorted(node.attrib.items())),
            (node.text or "").strip(),
            tuple(canonical(child) for child in node),
        )

    return hashlib.sha256(repr(canonical(parse_xml(xml))).encode()).hexdigest()


def strategy_space() -> dict:
    return {
        "coordinate_system": "NWU, meters, yaw degrees, positive counterclockwise",
        "actions": ACTIONS,
        "logic": LOGIC,
        "max_repetitions": 5,
        "blackboard": "Whole entry references only, e.g. {frame_position}. "
        "No arithmetic, member access, or placeholder commands.",
        "example": '<root BTCPP_format="4"><BehaviorTree ID="MainTree">'
        '<Sequence><TakeOff takeoff_target="0;0;1.5"/>'
        '<ForwardDetectFrame FramePosition="{frame_position}"/>'
        '<FlyToCoordinates coordinates="{frame_position}"/>'
        "<Land/></Sequence></BehaviorTree></root>",
    }


def validate_bt(xml: str, *, max_nodes: int = 256, max_depth: int = 32) -> ValidationResult:
    errors: list[str] = []
    try:
        root = parse_xml(xml)
    except (ET.ParseError, ValueError) as exc:
        return ValidationResult(False, [f"Invalid XML: {exc}"])
    if root.tag != "root" or root.get("BTCPP_format") != "4":
        errors.append('Expected <root BTCPP_format="4">')
    if set(root.attrib) - {"BTCPP_format", "main_tree_to_execute"}:
        errors.append("Unknown root attributes")
    trees = list(root)
    if len(trees) != 1 or trees[0].tag != "BehaviorTree":
        return ValidationResult(False, errors + ["Exactly one BehaviorTree required"])
    tree = trees[0]
    if tree.get("ID") != "MainTree" or set(tree.attrib) != {"ID"} or len(tree) != 1:
        errors.append('Expected BehaviorTree ID="MainTree" with one child')
    if root.get("main_tree_to_execute", "MainTree") != "MainTree":
        errors.append("Unknown main tree")
    count = 0

    def visit(node: ET.Element, known: dict[str, str], path: str, depth: int) -> dict:
        nonlocal count
        count += 1
        if count > max_nodes or depth > max_depth:
            errors.append(f"{path}: node/depth budget exceeded")
            return known
        if (node.text or "").strip() or any((c.tail or "").strip() for c in node):
            errors.append(f"{path}: unexpected text")
        if node.tag not in ACTIONS and node.tag not in LOGIC:
            errors.append(f"{path}: unsupported node {node.tag}")
            return known
        ports = ACTIONS.get(node.tag, LOGIC.get(node.tag, {}))
        allowed = set(ports) | {"name"}
        for key in set(node.attrib) - allowed:
            errors.append(f"{path}: unknown port {key}")
        output = dict(known)
        for key, (kind, direction) in ports.items():
            value = node.get(key)
            if value is None:
                if direction == "in":
                    errors.append(f"{path}: missing input {key}")
                continue
            ref = REF.fullmatch(value)
            if ref:
                entry = ref[1]
                if direction == "out":
                    if entry in output and output[entry] != kind:
                        errors.append(f"{path}: incompatible output type for {entry}")
                    output[entry] = kind
                elif known.get(entry) != kind:
                    errors.append(f"{path}: undefined or incompatible blackboard entry {entry}")
                continue
            if direction == "out":
                errors.append(f"{path}: output {key} requires {{entry}}")
                continue
            try:
                if kind == "count":
                    if not value.isdigit() or not 1 <= int(value) <= 5:
                        raise ValueError()
                else:
                    parts = value.split(";")
                    nums = [float(part) for part in parts]
                    if not all(math.isfinite(n) for n in nums):
                        raise ValueError()
                    if len(nums) != (3 if kind == "position" else 1):
                        raise ValueError()
                    if kind == "positive" and nums[0] <= 0:
                        raise ValueError()
            except ValueError:
                errors.append(f"{path}: invalid literal {key}={value!r}")
        if node.tag in ACTIONS:
            if len(node):
                errors.append(f"{path}: action must be a leaf")
            return output
        if node.tag in {"Sequence", "Fallback"}:
            if not len(node):
                errors.append(f"{path}: composite requires children")
            if node.tag == "Sequence":
                for index, child in enumerate(node):
                    output = visit(child, output, f"{path}/{index}", depth + 1)
                return output
            # Each alternative starts with the incoming state. Only entries that
            # every successful alternative writes are guaranteed afterwards.
            branches = [
                visit(child, dict(known), f"{path}/{i}", depth + 1) for i, child in enumerate(node)
            ]
            if not branches:
                return known
            return {
                k: v
                for k, v in branches[0].items()
                if all(branch.get(k) == v for branch in branches)
            }
        if len(node) != 1:
            errors.append(f"{path}: decorator requires exactly one child")
            return known
        child_result = visit(node[0], dict(known), path + "/0", depth + 1)
        return known if node.tag == "Inverter" else child_result

    if len(tree) == 1:
        visit(tree[0], {}, "/MainTree/0", 1)

    def leaf_budget(node):
        if node.tag in ACTIONS:
            return 1
        subtotal = sum(leaf_budget(child) for child in node)
        if node.tag in {"Repeat", "RetryUntilSuccessful"}:
            value = node.get("num_cycles", node.get("num_attempts", "1"))
            subtotal *= int(value) if value.isdigit() and len(value) < 3 else 6
        return min(subtotal, 4097)

    if not errors and leaf_budget(tree) > 4096:
        errors.append("Worst-case expanded action budget exceeds 4096")
    return ValidationResult(not errors, errors, count)


def require_valid_bt(xml: str) -> str:
    result = validate_bt(xml)
    if not result.valid:
        raise ValueError("BT validation failed: " + "; ".join(result.errors))
    return xml


def extract_xml(text: str) -> str:
    cleaned = text.strip()
    if cleaned.startswith("```"):
        lines = cleaned.splitlines()
        if lines[-1].strip() != "```":
            raise ValueError("Unclosed XML fence")
        cleaned = "\n".join(lines[1:-1])
    return require_valid_bt(cleaned)
