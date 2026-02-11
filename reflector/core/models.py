from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class LayeredBTAnalysis:
    """Hierarchical analysis findings over BT behavior/logical/structure layers."""

    raw_text: str = ""
    action_layer_flaws: List[Dict[str, Any]] = field(default_factory=list)
    logic_layer_flaws: List[Dict[str, Any]] = field(default_factory=list)
    mission_layer_flaws: List[Dict[str, Any]] = field(default_factory=list)
    findings: List[Dict[str, Any]] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ConstraintCheckResult:
    """Dual-constraint validation result."""

    operational_feasibility: str
    structural_validity: str
    raw_text: str = ""
    action_space: List[str] = field(default_factory=list)
    logic_space: List[str] = field(default_factory=list)
    validator_result: Dict[str, Any] = field(default_factory=dict)
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class NodeLevelCorrectionSpec:
    """Candidate node-level correction specification."""

    raw_text: str = ""
    reflective_experiences: List[Dict[str, Any]] = field(default_factory=list)
    proposals: List[Dict[str, Any]] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ExperienceUpdatePlan:
    """Experience base update plan for closed-loop refinement."""

    record_policy: str
    raw_text: str = ""
    payload: Dict[str, Any] = field(default_factory=dict)
