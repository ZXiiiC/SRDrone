from __future__ import annotations

from workflow.core.interfaces import Evaluator
from workflow.core.models import EvaluationResult, ExecutionSnapshot, MissionContext


class ContinuousStateEvaluatorStub(Evaluator):
    """Scaffold for paper §3.2 Continuous State Evaluation.

    This placeholder keeps interfaces stable while algorithm implementation is pending.
    """

    def evaluate(self, mission: MissionContext, snapshot: ExecutionSnapshot) -> EvaluationResult:
        return EvaluationResult(
            cycle_index=snapshot.cycle_index,
            success=False,
            summary=(
                "Stub evaluation result: algorithm not implemented yet. "
                "Pending action-centric filtering + CMSR semantic extraction."
            ),
            anomaly_attribution=[
                {
                    "layer": "execution",
                    "type": "not_implemented",
                    "detail": "Continuous State Evaluation module is scaffold-only.",
                }
            ],
            diagnostics={
                "mission_id": mission.mission_id,
                "placeholder": True,
            },
        )

