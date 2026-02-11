from __future__ import annotations

from abc import ABC, abstractmethod

from workflow.core.models import EvaluationResult, ExecutionSnapshot, MissionContext

from .models import (
    ConstraintCheckResult,
    ExperienceUpdatePlan,
    LayeredBTAnalysis,
    NodeLevelCorrectionSpec,
)


class HierarchicalAnalyzer(ABC):
    @abstractmethod
    def run(
        self,
        mission: MissionContext,
        snapshot: ExecutionSnapshot,
        evaluation: EvaluationResult,
    ) -> LayeredBTAnalysis:
        """Stage A: hierarchical BT analysis over multiple levels."""


class DualConstraintProcessor(ABC):
    @abstractmethod
    def run(
        self,
        mission: MissionContext,
        analysis: LayeredBTAnalysis,
    ) -> ConstraintCheckResult:
        """Stage B: operational feasibility + structural validity checks."""


class NodeCorrectionGenerator(ABC):
    @abstractmethod
    def run(
        self,
        mission: MissionContext,
        analysis: LayeredBTAnalysis,
        constraints: ConstraintCheckResult,
    ) -> NodeLevelCorrectionSpec:
        """Stage C: generate node-level BT correction specs."""


class ExperienceBaseUpdater(ABC):
    @abstractmethod
    def run(
        self,
        mission: MissionContext,
        analysis: LayeredBTAnalysis,
        correction_spec: NodeLevelCorrectionSpec,
    ) -> ExperienceUpdatePlan:
        """Stage D: prepare experience-base update payload."""

