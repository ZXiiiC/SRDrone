from __future__ import annotations

from workflow.core.models import ExecutionSnapshot, MissionContext

from .interfaces import ActionCentricStateCapture, CMSRExtractor, TaskOutcomeDeterminer
from .models import (
    ActionAnnotatedState,
    ActionStateSequence,
    CMSRRepresentation,
    TaskDetermination,
)


class ActionCentricStateCaptureStage(ActionCentricStateCapture):
    """Skeleton for paper §3.2 Action-Centric State Capture."""

    def run(self, mission: MissionContext, snapshot: ExecutionSnapshot) -> ActionStateSequence:
        state_payload = snapshot.state_payload
        log_text = state_payload.get("filtered_log_text", "")
        plan_xml = state_payload.get("plan_xml", "")
        session_index = state_payload.get("log_session_index", -1)
        session_count = state_payload.get("log_session_count", 0)

        annotated = ActionAnnotatedState(
            action="ControllerFilteredLogEntry",
            state_vector={},
            modality="controller_log",
            environment_observation={
                "plan_xml": plan_xml,
                "session_index": session_index,
                "session_count": session_count,
            },
            failure_tag=None,
        )

        return ActionStateSequence(
            cycle_index=snapshot.cycle_index,
            raw_text=(
                f"[Action-Centric Filtering] mission={mission.mission_id}, cycle={snapshot.cycle_index}. "
                f"Using controller-provided session split from source={state_payload.get('log_source', 'unknown')} "
                f"(session={session_index}/{session_count}).\n"
                f"[PLAN XML]\n{plan_xml}\n"
                f"[FILTERED LOG]\n"
                f"{log_text}"
            ),
            sequence=[annotated],
            metadata={
                "mission_id": mission.mission_id,
                "stage": "action_centric_state_filtering",
                "format": "text_first",
                "source": "controller_log",
                "session_index": session_index,
                "session_count": session_count,
                "placeholder": True,
            },
        )


class CMSRExtractionStage(CMSRExtractor):
    """Skeleton for CMSR spatiotemporal semantic extraction."""

    def run(self, mission: MissionContext, action_state: ActionStateSequence) -> CMSRRepresentation:
        ego_trajectory = []
        temporal_spatial_relations = []
        environment_state = []

        for index, item in enumerate(action_state.sequence):
            motion_semantic = {
                "step": index,
                "behavior_type": item.action,
                "displacement": {
                    "x": item.state_vector.get("x", 0.0),
                    "y": item.state_vector.get("y", 0.0),
                    "z": item.state_vector.get("z", 0.0),
                },
                "orientation_change": item.state_vector.get("yaw", 0.0),
            }
            ego_trajectory.append(motion_semantic)

            if item.environment_observation:
                environment_state.append(item.environment_observation)
                temporal_spatial_relations.append(
                    {
                        "step": index,
                        "relations": [
                            {
                                "phi_intent": "unknown",
                                "phi_proximity": "unknown",
                                "phi_safety": "unknown",
                            }
                        ],
                    }
                )

        semantic_trajectory = (
            f"Mission {mission.mission_id} cycle {action_state.cycle_index}: "
            "CMSR semantic trajectory placeholder from action-annotated states."
        )
        raw_text = (
            f"[CMSR] Built semantic trajectory for mission={mission.mission_id}, cycle={action_state.cycle_index}. "
            f"Ego-motion items={len(ego_trajectory)}, environment_updates={len(environment_state)}. "
            f"Narrative={semantic_trajectory}"
        )

        return CMSRRepresentation(
            raw_text=raw_text,
            ego_trajectory=ego_trajectory,
            environment_state=environment_state,
            temporal_spatial_relations=temporal_spatial_relations,
            semantic_trajectory=semantic_trajectory,
            metadata={
                "format": "text_first",
                "placeholder": True,
            },
        )


class TaskOutcomeDeterminationStage(TaskOutcomeDeterminer):
    """Skeleton for outcome determination and interpretable failure attribution."""

    def run(
        self,
        mission: MissionContext,
        action_state: ActionStateSequence,
        cmsr: CMSRRepresentation,
    ) -> TaskDetermination:
        summary = (
            "Semantic-Based Task Evaluation skeleton executed. "
            "Multi-dimensional scoring logic pending implementation."
        )
        raw_text = (
            f"[Semantic-Based Task Evaluation] mission={mission.mission_id}, cycle={action_state.cycle_index}. "
            "Evaluated three dimensions: navigational_intent_verification, proximity_awareness, "
            "collision_centric_safety. Current output is explanatory placeholder text."
        )

        dimension_scores = {
            "navigational_intent_verification": {
                "status": "not_implemented",
                "detail": "Check completeness and directional fidelity.",
            },
            "proximity_awareness": {
                "status": "not_implemented",
                "detail": "Check logical consistency of spatial interactions.",
            },
            "collision_centric_safety": {
                "status": "not_implemented",
                "detail": "Check safety constraint compliance in trajectory.",
            },
        }

        anomaly_attribution = [
            {
                "layer": "execution",
                "type": "insufficient_decision_logic",
                "detail": "Semantic-based multi-dimensional evaluator is scaffold-only.",
            }
        ]
        diagnostics = {
            "mission_id": mission.mission_id,
            "cycle_index": action_state.cycle_index,
            "semantic_ready": bool(cmsr.semantic_trajectory),
            "sequence_size": len(action_state.sequence),
            "placeholder": True,
        }
        return TaskDetermination(
            success=False,
            summary=summary,
            raw_text=raw_text,
            dimension_scores=dimension_scores,
            anomaly_attribution=anomaly_attribution,
            diagnostics=diagnostics,
        )
