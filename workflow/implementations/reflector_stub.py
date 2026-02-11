from __future__ import annotations

from workflow.core.interfaces import Reflector
from workflow.core.models import (
    EvaluationResult,
    ExecutionSnapshot,
    MissionContext,
    ReflectionPlan,
)


class HierarchicalBTReflectorStub(Reflector):
    """Scaffold for paper §3.3 Hierarchical BT Modification.

    Produces non-destructive modification proposals only.
    """

    def reflect(
        self,
        mission: MissionContext,
        snapshot: ExecutionSnapshot,
        evaluation: EvaluationResult,
    ) -> ReflectionPlan:
        return ReflectionPlan(
            cycle_index=snapshot.cycle_index,
            needs_modification=not evaluation.success,
            summary=(
                "Stub reflection result: hierarchical BT modification pending implementation."
            ),
            proposed_changes=[
                {
                    "target_bt": mission.bt_id,
                    "change_type": "proposal_only",
                    "description": "No direct BT file mutation in scaffold stage.",
                }
            ],
            constraints_check={
                "operational_feasibility": "unknown",
                "structural_validity": "unknown",
                "placeholder": True,
            },
        )

