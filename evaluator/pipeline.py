from __future__ import annotations

from typing import Dict

from workflow.core.interfaces import Evaluator
from workflow.core.models import EvaluationResult, ExecutionSnapshot, MissionContext

from .core.interfaces import ActionCentricStateCapture, CMSRExtractor, TaskOutcomeDeterminer
from .core.stages import (
    ActionCentricStateCaptureStage,
    CMSRExtractionStage,
    TaskOutcomeDeterminationStage,
)


class ContinuousStateEvaluatorPipeline(Evaluator):
    """Paper-aligned evaluator skeleton for §3.2.

    Stage order:
      1) Action-Centric State Capture
      2) CMSR semantic extraction
      3) Task determination and interpretable attribution
    """

    def __init__(
        self,
        state_capture: ActionCentricStateCapture | None = None,
        cmsr_extractor: CMSRExtractor | None = None,
        determiner: TaskOutcomeDeterminer | None = None,
    ) -> None:
        self._state_capture = state_capture or ActionCentricStateCaptureStage()
        self._cmsr_extractor = cmsr_extractor or CMSRExtractionStage()
        self._determiner = determiner or TaskOutcomeDeterminationStage()

    def evaluate(self, mission: MissionContext, snapshot: ExecutionSnapshot) -> EvaluationResult:
        action_state = self._state_capture.run(mission=mission, snapshot=snapshot)
        cmsr = self._cmsr_extractor.run(mission=mission, action_state=action_state)
        determination = self._determiner.run(
            mission=mission,
            action_state=action_state,
            cmsr=cmsr,
        )

        stage_outputs: Dict[str, object] = {
            "action_centric_state_filtering": {
                "raw_text": action_state.raw_text,
                "cycle_index": action_state.cycle_index,
                "sequence": [
                    {
                        "action": item.action,
                        "state_vector": item.state_vector,
                        "modality": item.modality,
                        "environment_observation": item.environment_observation,
                        "failure_tag": item.failure_tag,
                    }
                    for item in action_state.sequence
                ],
                "metadata": action_state.metadata,
            },
            "cmsr": {
                "raw_text": cmsr.raw_text,
                "ego_trajectory": cmsr.ego_trajectory,
                "environment_state": cmsr.environment_state,
                "temporal_spatial_relations": cmsr.temporal_spatial_relations,
                "semantic_trajectory": cmsr.semantic_trajectory,
                "metadata": cmsr.metadata,
            },
            "semantic_based_task_evaluation": {
                "raw_text": determination.raw_text,
                "dimension_scores": determination.dimension_scores,
            },
        }

        return EvaluationResult(
            cycle_index=snapshot.cycle_index,
            success=determination.success,
            summary=determination.summary,
            anomaly_attribution=determination.anomaly_attribution,
            task_narrative=cmsr.semantic_trajectory,
            diagnostics={
                **determination.diagnostics,
                "dimension_scores": determination.dimension_scores,
                "evaluation_raw_text": determination.raw_text,
            },
            stage_outputs=stage_outputs,
        )
