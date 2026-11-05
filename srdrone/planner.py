"""One-shot BT generation and experience-guided regeneration (§IV-B)."""

from __future__ import annotations

from .bt import extract_xml, strategy_space
from .llm import ChatModel, TextModel
from .prompts import prompt


class BTPlanner:
    def __init__(self, model: TextModel | None = None, repair_attempts: int = 1) -> None:
        self.model = model or ChatModel.from_env()
        if not 0 <= repair_attempts <= 3:
            raise ValueError("repair_attempts must be 0..3")
        self.repair_attempts = repair_attempts

    def generate(self, task: str, scene: dict | str, experiences: list[dict] | None = None) -> str:
        return self._generate(
            "plan", {"task": task, "scene": scene, "experiences": experiences or []}
        )

    def refine(
        self, task: str, scene: dict | str, plan_xml: str, evaluation: dict, experiences: list[dict]
    ) -> str:
        return self._generate(
            "refine",
            {
                "task": task,
                "scene": scene,
                "original_plan_xml": plan_xml,
                "evaluation": evaluation,
                "experiences": experiences,
            },
        )

    def _generate(self, stage: str, payload: dict) -> str:
        if not isinstance(payload["task"], str) or not payload["task"].strip():
            raise ValueError("A task is required")
        contract = (
            "Return only a complete BehaviorTree.CPP v4 XML document, MainTree. "
            "The supplied strategy_space overrides the recovered prompt: use ONLY "
            "its registered nodes and exact port names. In particular the notebook "
            "DetectTarget/ForwardDetectBalloon/DownScanHtag examples are not supported "
            "by this controller profile. Positions are NWU meters; yaw is degrees. "
            "Use a whole blackboard reference {frame_position}, never member access "
            "or arithmetic in port strings. Require detection before consuming its "
            "result; provide bounded failure recovery where needed. Make only "
            "changes justified by the task and reflective experiences. Scene bounds "
            "are operational constraints. Do not claim physical validation.\n"
        )
        data = {**payload, "strategy_space": strategy_space()}
        for attempt in range(self.repair_attempts + 1):
            raw = self.model.complete(
                stage if attempt == 0 else "xml_repair", prompt("actor", contract), data
            )
            try:
                return extract_xml(raw)
            except ValueError as exc:
                if attempt == self.repair_attempts:
                    raise ValueError(f"No valid BT after {attempt + 1} attempts: {exc}") from exc
                data = {**data, "rejected_xml": raw, "validation_error": str(exc)}
        raise RuntimeError("Unreachable generation state")
