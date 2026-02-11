from __future__ import annotations

from workflow.core.interfaces import Reflector
from workflow.core.models import (
    EvaluationResult,
    ExecutionSnapshot,
    MissionContext,
    ReflectionPlan,
)

from .core.interfaces import (
    DualConstraintProcessor,
    ExperienceBaseUpdater,
    HierarchicalAnalyzer,
    NodeCorrectionGenerator,
)
from .core.stages import (
    DualConstraintStage,
    ExperienceBaseUpdateStage,
    HierarchicalBTAnalysisStage,
    NodeLevelCorrectionStage,
)


class HierarchicalBTReflectorPipeline(Reflector):
    """Paper-aligned reflector skeleton for §3.3.

    Stage order:
      1) Hierarchical BT analysis
      2) Dual-constraint processing
      3) Node-level correction specification
      4) Experience-base update planning
    """

    def __init__(
        self,
        analyzer: HierarchicalAnalyzer | None = None,
        constraints: DualConstraintProcessor | None = None,
        correction_generator: NodeCorrectionGenerator | None = None,
        experience_updater: ExperienceBaseUpdater | None = None,
    ) -> None:
        self._analyzer = analyzer or HierarchicalBTAnalysisStage()
        self._constraints = constraints or DualConstraintStage()
        self._correction_generator = correction_generator or NodeLevelCorrectionStage()
        self._experience_updater = experience_updater or ExperienceBaseUpdateStage()

    def reflect(
        self,
        mission: MissionContext,
        snapshot: ExecutionSnapshot,
        evaluation: EvaluationResult,
    ) -> ReflectionPlan:
        analysis = self._analyzer.run(
            mission=mission,
            snapshot=snapshot,
            evaluation=evaluation,
        )
        constraints_result = self._constraints.run(mission=mission, analysis=analysis)
        correction_spec = self._correction_generator.run(
            mission=mission,
            analysis=analysis,
            constraints=constraints_result,
        )
        experience_update = self._experience_updater.run(
            mission=mission,
            analysis=analysis,
            correction_spec=correction_spec,
        )

        summary = (
            "Hierarchical BT Modification skeleton executed. "
            "Core correction algorithm pending implementation."
        )
        needs_modification = (not evaluation.success) and bool(correction_spec.proposals)

        return ReflectionPlan(
            cycle_index=snapshot.cycle_index,
            needs_modification=needs_modification,
            summary=summary,
            proposed_changes=correction_spec.proposals,
            constraints_check={
                "hierarchical_analysis_raw_text": analysis.raw_text,
                "dual_constraint_raw_text": constraints_result.raw_text,
                "node_correction_raw_text": correction_spec.raw_text,
                "operational_feasibility": constraints_result.operational_feasibility,
                "structural_validity": constraints_result.structural_validity,
                "action_space": constraints_result.action_space,
                "logic_space": constraints_result.logic_space,
                "validator_result": constraints_result.validator_result,
                "details": constraints_result.details,
            },
            layered_findings=[
                {
                    "F_alpha": analysis.action_layer_flaws,
                    "F_lambda": analysis.logic_layer_flaws,
                    "F_pi": analysis.mission_layer_flaws,
                },
                *analysis.findings,
            ],
            experience_update={
                "record_policy": experience_update.record_policy,
                "raw_text": experience_update.raw_text,
                "payload": experience_update.payload,
                "Enew": correction_spec.reflective_experiences,
            },
        )
