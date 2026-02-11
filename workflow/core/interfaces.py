from __future__ import annotations

from abc import ABC, abstractmethod

from .models import EvaluationResult, ExecutionSnapshot, MissionContext, ReflectionPlan


class StateProvider(ABC):
    @abstractmethod
    def capture(self, mission: MissionContext, cycle_index: int) -> ExecutionSnapshot:
        """Capture action-centric execution snapshot for one cycle."""


class Evaluator(ABC):
    @abstractmethod
    def evaluate(self, mission: MissionContext, snapshot: ExecutionSnapshot) -> EvaluationResult:
        """Evaluate execution outcome and produce interpretable diagnostics."""


class Reflector(ABC):
    @abstractmethod
    def reflect(
        self,
        mission: MissionContext,
        snapshot: ExecutionSnapshot,
        evaluation: EvaluationResult,
    ) -> ReflectionPlan:
        """Generate hierarchical BT modification proposal under constraints."""

