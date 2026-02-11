from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional


@dataclass
class MissionContext:
    mission_id: str
    bt_id: str
    bt_path: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ExecutionSnapshot:
    timestamp: datetime
    cycle_index: int
    state_payload: Dict[str, Any]


@dataclass
class EvaluationResult:
    cycle_index: int
    success: bool
    summary: str
    anomaly_attribution: List[Dict[str, Any]] = field(default_factory=list)
    task_narrative: str = ""
    diagnostics: Dict[str, Any] = field(default_factory=dict)
    stage_outputs: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ReflectionPlan:
    cycle_index: int
    needs_modification: bool
    summary: str
    proposed_changes: List[Dict[str, Any]] = field(default_factory=list)
    constraints_check: Dict[str, Any] = field(default_factory=dict)
    layered_findings: List[Dict[str, Any]] = field(default_factory=list)
    experience_update: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ExperienceRecord:
    mission_id: str
    bt_id: str
    cycle_index: int
    snapshot: Dict[str, Any]
    evaluation: Dict[str, Any]
    reflection: Dict[str, Any]
    created_at: str


@dataclass
class WorkflowCycleOutput:
    snapshot: ExecutionSnapshot
    evaluation: EvaluationResult
    reflection: ReflectionPlan
