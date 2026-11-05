"""Hand-authored fixture demonstration; no LLM inference or flight simulation."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from evaluator.pipeline import ContinuousStateEvaluatorPipeline
from reflector.pipeline import HierarchicalBTReflectorPipeline
from workflow.core.models import ExecutionSnapshot, MissionContext

from .bt import plan_digest
from .experience import ExperienceBank
from .llm import RecordedModel
from .planner import BTPlanner
from .runner import RefinementRunner

EXAMPLES = Path(__file__).parent / "data" / "frame_crossing"


class FixtureExecutor:
    def execute(
        self, mission: MissionContext, plan_xml: str, cycle_index: int
    ) -> ExecutionSnapshot:
        name = "initial" if cycle_index == 0 else "corrected"
        expected = (EXAMPLES / (name + ".xml")).read_text(encoding="utf-8")
        if cycle_index > 1 or plan_digest(plan_xml) != plan_digest(expected):
            raise ValueError("Demo fixture does not match requested plan")
        return ExecutionSnapshot(
            datetime.now(timezone.utc),
            cycle_index,
            {
                "plan_xml": plan_xml,
                "filtered_log_text": (EXAMPLES / (name + ".txt")).read_text(encoding="utf-8"),
                "root_status": "SUCCESS",
                "execution_kind": "hand_authored_fixture",
                "log_source": "synthetic demo fixture; not experimental evidence",
            },
        )


def run_demo(output: str):
    model = RecordedModel(json.loads((EXAMPLES / "responses.json").read_text(encoding="utf-8")))
    mission = MissionContext(
        "fixture-frame-crossing",
        "MainTree",
        metadata={
            "task": "Find the square frame, pass through its opening, and land.",
            "task_category": "obstacle_navigation",
            "scene": {
                "frame_center_nwu": [2, 0, 1.5],
                "frame_normal_nwu": [1, 0, 0],
                "opening_width_m": 1.3,
                "opening_height_m": 1.3,
                "drone_radius_m": 0.1,
                "other_obstacles": [],
                "provenance": "hand-authored ideal fixture, known straight-line segments",
            },
        },
    )
    planner = BTPlanner(model)
    bank = ExperienceBank(str(Path(output).parent / (Path(output).name + "-experience.sqlite3")))
    runner = RefinementRunner(
        FixtureExecutor(),
        ContinuousStateEvaluatorPipeline(model=model),
        HierarchicalBTReflectorPipeline(model=model),
        planner,
        bank,
    )
    results = runner.run(
        mission,
        output,
        initial_plan=(EXAMPLES / "initial.xml").read_text(),
        max_refinements=2,
        retriever="lexical",
        top_k=0,
    )
    if model.responses:
        raise RuntimeError("Unused demo model responses")
    return results
