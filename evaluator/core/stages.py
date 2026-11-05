"""Reconstructed continuous state evaluation (§III-B, Algorithms 1)."""

from __future__ import annotations

from dataclasses import asdict

from srdrone.llm import ChatModel, ModelError, TextModel, json_object, require_text
from srdrone.logs import parse_action_states
from srdrone.prompts import prompt
from workflow.core.models import ExecutionSnapshot, MissionContext

from .interfaces import ActionCentricStateCapture, CMSRExtractor, TaskOutcomeDeterminer
from .models import ActionStateSequence, CMSRRepresentation, TaskDetermination


class ActionCentricStateCaptureStage(ActionCentricStateCapture):
    """Read already filtered completion events; never tail away earlier actions."""

    def run(self, mission: MissionContext, snapshot: ExecutionSnapshot) -> ActionStateSequence:
        payload = snapshot.state_payload
        log = payload.get("filtered_log_text", "")
        plan = payload.get("plan_xml", "")
        if not plan.strip() or not log.strip():
            raise ValueError("Evaluation requires a plan and a nonempty execution log")
        sequence = parse_action_states(log)
        if not sequence:
            raise ValueError("No completion/failure events found in execution log")
        return ActionStateSequence(
            cycle_index=snapshot.cycle_index,
            raw_text=log,
            sequence=sequence,
            metadata={
                "mission_id": mission.mission_id,
                "plan_xml": plan,
                "source": payload.get("log_source", "provided_snapshot"),
                "log_session_index": payload.get("log_session_index"),
                "root_status": payload.get("root_status", "unknown"),
                "frame": "NWU",
                "yaw_unit": "degrees",
            },
        )


class CMSRExtractionStage(CMSRExtractor):
    """Recovered trajectory prompt + motion/environment reasoning prompt.

    The notebook's idealized trajectory is auxiliary; raw observations are always
    passed to task determination so reconstruction cannot erase real deviations.
    """

    def __init__(self, model: TextModel | None = None) -> None:
        self.model = model or ChatModel.from_env()

    def run(self, mission: MissionContext, action_state: ActionStateSequence) -> CMSRRepresentation:
        evidence = {
            "task": mission.metadata.get("task", ""),
            "scene": mission.metadata.get("scene", {}),
            "plan_xml": action_state.metadata["plan_xml"],
            "raw_log": action_state.raw_text,
            "measured_states": [asdict(x) for x in action_state.sequence],
        }
        trajectory = self.model.complete(
            "trajectory",
            prompt(
                "trajectory",
                'Return JSON {"idealized_trajectory": "...", "uncertainties": ["..."]}. '
                "This is an inferred ideal trajectory, not a measured one. Do not treat "
                "commanded targets as reached. Record missing initial pose and missing "
                "TakeOff snapshots as uncertain. Preserve failure events.",
            ),
            evidence,
        )
        ideal = json_object(trajectory)
        require_text(ideal, "idealized_trajectory")
        raw = self.model.complete(
            "cmsr",
            prompt(
                "cmsr",
                'Return JSON with nonempty "semantic_trajectory" string and lists '
                '"ego_trajectory", "environment_state", "temporal_spatial_relations". '
                "Follow Algorithm 1: derive motion between consecutive observed states; "
                "update persistent environment state, retaining last-seen objects through "
                "blind spots with source/provenance and staleness; reason intent, "
                "proximity and safety; align them in temporal order. Use NWU and yaw "
                "degrees. Wrap angular differences to [-180,180]. Keep measured motion "
                "distinct from ideal motion. A detection failure is not proof that an "
                "object disappeared. Endpoint logs cannot prove an obstacle-free "
                "continuous path; missing geometry/dimensions makes safety unknown.",
            ),
            {**evidence, "idealized_reconstruction": ideal},
        )
        data = json_object(raw)
        narrative = require_text(data, "semantic_trajectory")
        for key in ("ego_trajectory", "environment_state", "temporal_spatial_relations"):
            if not isinstance(data.get(key), list) or not all(
                isinstance(x, dict) for x in data[key]
            ):
                raise ModelError(f"Expected CMSR list of objects: {key}")
        return CMSRRepresentation(
            raw_text=raw,
            semantic_trajectory=narrative,
            ego_trajectory=data["ego_trajectory"],
            environment_state=data["environment_state"],
            temporal_spatial_relations=data["temporal_spatial_relations"],
            metadata={
                "idealized_reconstruction": ideal,
                "trajectory_raw_text": trajectory,
                "implementation": "reconstructed_prompt_pipeline",
            },
        )


class TaskOutcomeDeterminationStage(TaskOutcomeDeterminer):
    def __init__(self, model: TextModel | None = None) -> None:
        self.model = model or ChatModel.from_env()

    def run(
        self, mission: MissionContext, action_state: ActionStateSequence, cmsr: CMSRRepresentation
    ) -> TaskDetermination:
        task = mission.metadata.get("task", "")
        if not isinstance(task, str) or not task.strip():
            raise ValueError("A natural-language task is required for evaluation")
        evidence = {
            "task": task,
            "scene": mission.metadata.get("scene", {}),
            "plan_xml": action_state.metadata["plan_xml"],
            "raw_log": action_state.raw_text,
            "measured_states": [asdict(x) for x in action_state.sequence],
            "root_status": action_state.metadata.get("root_status", "unknown"),
        }
        execution_raw = self.model.complete(
            "execution",
            prompt(
                "execution",
                'Return JSON {"is_completed": "YES|NO|UNKNOWN", "executed_action": "...", '
                '"action_details": "...", "logic_details": "..."}. Account for Fallback '
                "and Retry: a failed child does not imply root failure. Completion and "
                "mission success are separate. Missing log events are unknown, not "
                "evidence that an action succeeded.",
            ),
            evidence,
        )
        execution = json_object(execution_raw)
        if execution.get("is_completed") not in {"YES", "NO", "UNKNOWN"}:
            raise ModelError("Invalid execution completeness result")
        raw = self.model.complete(
            "outcome",
            prompt(
                "outcome",
                'Return JSON {"result": "success|failure|unknown", "task_requirement": "...", '
                '"relevant_trajectory": "...", "gap_assessment": "...", "explanation": "...", '
                '"dimensions": {"navigational_intent_verification": {"status": "...", "detail": "..."}, '
                '"proximity_awareness": {"status": "...", "detail": "..."}, '
                '"collision_centric_safety": {"status": "...", "detail": "..."}}, '
                '"anomalies": [{"layer": "execution|action|logic|mission", "target": "...", "detail": "..."}]}. '
                "Use status pass/fail/unknown. Verify the whole trajectory against the "
                "task, including intended path, spatial interactions and collision "
                "constraints. Do not infer task success merely from BT completion, "
                "arrival at a frame center, or an ideal reconstruction. State uncertainty "
                "where sensor coverage, root status or obstacle geometry is insufficient.",
            ),
            {**evidence, "execution": execution, "cmsr": asdict(cmsr)},
        )
        data = json_object(raw)
        if data.get("result") not in {"success", "failure", "unknown"}:
            raise ModelError("Invalid task result")
        dimensions = data.get("dimensions")
        keys = (
            "navigational_intent_verification",
            "proximity_awareness",
            "collision_centric_safety",
        )
        if not isinstance(dimensions, dict) or set(dimensions) != set(keys):
            raise ModelError("Missing evaluation dimensions")
        for key in keys:
            item = dimensions.get(key)
            if not isinstance(item, dict) or item.get("status") not in {"pass", "fail", "unknown"}:
                raise ModelError(f"Invalid evaluation dimension {key}")
            require_text(item, "detail")
        anomalies = data.get("anomalies", [])
        if not isinstance(anomalies, list) or not all(isinstance(x, dict) for x in anomalies):
            raise ModelError("Invalid anomalies")
        # A reported success cannot override explicitly failed task dimensions.
        if data["result"] == "success" and any(v["status"] == "fail" for v in dimensions.values()):
            raise ModelError("Contradictory success and failed task dimension")
        return TaskDetermination(
            success=data["result"] == "success",
            summary=require_text(data, "explanation"),
            raw_text=raw,
            dimension_scores=dimensions,
            anomaly_attribution=anomalies,
            diagnostics={
                "result": data["result"],
                "execution": execution,
                "execution_raw_text": execution_raw,
                "assessment": data,
                "implementation": "reconstructed_prompt_pipeline",
            },
        )
