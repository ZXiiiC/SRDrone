"""Reconstructed hierarchical analysis and structured experience (§III-C)."""

from __future__ import annotations

from dataclasses import asdict

from srdrone.bt import ACTIONS, LOGIC, plan_digest, strategy_space, validate_bt
from srdrone.llm import ChatModel, ModelError, TextModel, json_object, require_text
from srdrone.prompts import prompt
from workflow.core.models import EvaluationResult, ExecutionSnapshot, MissionContext

from .interfaces import (
    DualConstraintProcessor,
    ExperienceBaseUpdater,
    HierarchicalAnalyzer,
    NodeCorrectionGenerator,
)
from .models import (
    ConstraintCheckResult,
    ExperienceUpdatePlan,
    LayeredBTAnalysis,
    NodeLevelCorrectionSpec,
)


class HierarchicalBTAnalysisStage(HierarchicalAnalyzer):
    def __init__(self, model: TextModel | None = None) -> None:
        self.model = model or ChatModel.from_env()

    def run(
        self, mission: MissionContext, snapshot: ExecutionSnapshot, evaluation: EvaluationResult
    ) -> LayeredBTAnalysis:
        payload = {
            "task": mission.metadata.get("task", ""),
            "scene": mission.metadata.get("scene", {}),
            "plan_xml": snapshot.state_payload["plan_xml"],
            "evaluation": asdict(evaluation),
        }
        raw = self.model.complete(
            "analysis",
            prompt(
                "analysis",
                'Return JSON {"action_layer_flaws": [{"target": "node or path", "problem_analysis": "..."}], '
                '"logic_layer_flaws": [...], "mission_layer_flaws": [...]}. '
                "Apply Algorithm 2: action alpha = correctness/completeness of atomic "
                "actions; logic lambda = control-flow semantics; mission pi = global "
                "intent alignment. Diagnose the current plan using the actual evaluation "
                "and execution evidence. Empty lists are allowed; do not invent defects "
                "just to fill every layer. Use XML paths to distinguish repeated nodes.",
            ),
            payload,
        )
        data = json_object(raw)
        for key in ("action_layer_flaws", "logic_layer_flaws", "mission_layer_flaws"):
            if not isinstance(data.get(key), list):
                raise ModelError(f"Expected analysis list {key}")
            for flaw in data[key]:
                if not isinstance(flaw, dict):
                    raise ModelError("Analysis flaw must be an object")
                require_text(flaw, "target")
                require_text(flaw, "problem_analysis")
        return LayeredBTAnalysis(
            raw_text=raw,
            action_layer_flaws=data["action_layer_flaws"],
            logic_layer_flaws=data["logic_layer_flaws"],
            mission_layer_flaws=data["mission_layer_flaws"],
            metadata=payload,
        )


class DualConstraintStage(DualConstraintProcessor):
    """Validate the source plan and expose the audited action/logic spaces.

    Final candidate validation happens after regeneration in the pipeline.
    """

    def run(self, mission: MissionContext, analysis: LayeredBTAnalysis) -> ConstraintCheckResult:
        result = validate_bt(analysis.metadata["plan_xml"])
        return ConstraintCheckResult(
            operational_feasibility="interface_checked" if result.valid else "rejected_source",
            structural_validity="valid" if result.valid else "invalid",
            raw_text="Deterministic source-plan validation; physical feasibility remains unverified.",
            action_space=list(ACTIONS),
            logic_space=list(LOGIC),
            validator_result=result.to_dict(),
            details={"strategy_space": strategy_space()},
        )


class NodeLevelCorrectionStage(NodeCorrectionGenerator):
    def __init__(self, model: TextModel | None = None) -> None:
        self.model = model or ChatModel.from_env()

    def run(
        self,
        mission: MissionContext,
        analysis: LayeredBTAnalysis,
        constraints: ConstraintCheckResult,
    ) -> NodeLevelCorrectionSpec:
        raw = self.model.complete(
            "correction",
            prompt(
                "correction",
                'Return JSON {"experiences": [{"layer": "action|logic|mission", '
                '"target": "node name or XML path", "operation": "concrete imperative edit", '
                '"rationale": "functional justification"}]}. This is Equation 7: '
                "layer tau plus OPERATION plus RATIONALE, not a replacement XML. "
                "Every edit must address an identified flaw using only the supplied "
                "strategy space. Empty experiences are allowed when there is no "
                "supported correction. Do not add unsupported actions. Preserve "
                "the mission intent.",
            ),
            {**analysis.metadata, "analysis": asdict(analysis), "constraints": asdict(constraints)},
        )
        data = json_object(raw)
        experiences = data.get("experiences")
        if not isinstance(experiences, list):
            raise ModelError("Expected experiences list")
        for entry in experiences:
            if not isinstance(entry, dict) or entry.get("layer") not in {
                "action",
                "logic",
                "mission",
            }:
                raise ModelError("Invalid experience layer")
            for key in ("target", "operation", "rationale"):
                require_text(entry, key)
        return NodeLevelCorrectionSpec(
            raw_text=raw,
            reflective_experiences=experiences,
            proposals=experiences,
            metadata={"source_plan_digest": plan_digest(analysis.metadata["plan_xml"])},
        )


class ExperienceBaseUpdateStage(ExperienceBaseUpdater):
    def run(
        self,
        mission: MissionContext,
        analysis: LayeredBTAnalysis,
        correction_spec: NodeLevelCorrectionSpec,
    ) -> ExperienceUpdatePlan:
        return ExperienceUpdatePlan(
            record_policy="retain_with_provenance_pending_execution",
            raw_text="Store reflective experience separately from execution validation.",
            payload={
                "task": mission.metadata.get("task", ""),
                "task_category": mission.metadata.get("task_category", "unspecified"),
                "source_plan_digest": correction_spec.metadata.get("source_plan_digest"),
                "experiences": correction_spec.reflective_experiences,
                "status": "proposed",
            },
        )
