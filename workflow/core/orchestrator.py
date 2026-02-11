from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
from typing import List

from .experience_store import ExperienceStore
from .interfaces import Evaluator, Reflector, StateProvider
from .models import ExperienceRecord, MissionContext, WorkflowCycleOutput


class WorkflowOrchestrator:
    """Closed-loop workflow scaffold for SRDrone paper pipeline."""

    def __init__(
        self,
        state_provider: StateProvider,
        evaluator: Evaluator,
        reflector: Reflector,
        experience_store: ExperienceStore,
    ) -> None:
        self._state_provider = state_provider
        self._evaluator = evaluator
        self._reflector = reflector
        self._experience_store = experience_store

    def run(self, mission: MissionContext, max_cycles: int = 3) -> List[WorkflowCycleOutput]:
        outputs: List[WorkflowCycleOutput] = []
        for cycle_index in range(max_cycles):
            snapshot = self._state_provider.capture(mission=mission, cycle_index=cycle_index)
            evaluation = self._evaluator.evaluate(mission=mission, snapshot=snapshot)
            reflection = self._reflector.reflect(
                mission=mission,
                snapshot=snapshot,
                evaluation=evaluation,
            )

            output = WorkflowCycleOutput(
                snapshot=snapshot,
                evaluation=evaluation,
                reflection=reflection,
            )
            outputs.append(output)

            self._experience_store.append(
                ExperienceRecord(
                    mission_id=mission.mission_id,
                    bt_id=mission.bt_id,
                    cycle_index=cycle_index,
                    snapshot=asdict(snapshot),
                    evaluation=asdict(evaluation),
                    reflection=asdict(reflection),
                    created_at=datetime.utcnow().isoformat(),
                )
            )

            if evaluation.success and not reflection.needs_modification:
                break

        return outputs

