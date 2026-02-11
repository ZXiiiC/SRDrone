from __future__ import annotations

from abc import ABC, abstractmethod

from workflow.core.models import ExecutionSnapshot, MissionContext

from .models import ActionStateSequence, CMSRRepresentation, TaskDetermination


class ActionCentricStateCapture(ABC):
    @abstractmethod
    def run(self, mission: MissionContext, snapshot: ExecutionSnapshot) -> ActionStateSequence:
        """Stage A: filter and capture key action-centric states."""


class CMSRExtractor(ABC):
    @abstractmethod
    def run(self, mission: MissionContext, action_state: ActionStateSequence) -> CMSRRepresentation:
        """Stage B: perform spatiotemporal semantic extraction (CMSR)."""


class TaskOutcomeDeterminer(ABC):
    @abstractmethod
    def run(
        self,
        mission: MissionContext,
        action_state: ActionStateSequence,
        cmsr: CMSRRepresentation,
    ) -> TaskDetermination:
        """Stage C: determine outcome and provide interpretable attribution."""
