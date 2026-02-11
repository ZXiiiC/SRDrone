from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class ActionAnnotatedState:
    """Optional parsed element from LLM text for one action state."""

    action: str
    state_vector: Dict[str, Any]
    modality: str
    environment_observation: Dict[str, Any] = field(default_factory=dict)
    failure_tag: Optional[str] = None


@dataclass
class ActionStateSequence:
    """Stage A output.

    Primary output is `raw_text` (LLM free-form narrative).
    `sequence` is optional parsed structure for downstream engineering use.
    """

    cycle_index: int
    raw_text: str = ""
    sequence: List[ActionAnnotatedState] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class CMSRRepresentation:
    """Stage B output.

    Primary output is `raw_text` (LLM free-form narrative for semantic trajectory).
    Structured fields are optional parsed artifacts.
    """

    raw_text: str = ""
    ego_trajectory: List[Dict[str, Any]] = field(default_factory=list)
    environment_state: List[Dict[str, Any]] = field(default_factory=list)
    temporal_spatial_relations: List[Dict[str, Any]] = field(default_factory=list)
    semantic_trajectory: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TaskDetermination:
    """Stage C output.

    Primary output is `raw_text` (LLM free-form evaluation explanation).
    """

    success: bool
    summary: str
    raw_text: str = ""
    dimension_scores: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    anomaly_attribution: List[Dict[str, Any]] = field(default_factory=list)
    diagnostics: Dict[str, Any] = field(default_factory=dict)
