"""Actual execute/evaluate/reflect/regenerate loop, separate from log replay."""

from __future__ import annotations

import uuid
from dataclasses import asdict
from pathlib import Path

from evaluator.pipeline import ContinuousStateEvaluatorPipeline
from reflector.pipeline import HierarchicalBTReflectorPipeline
from workflow.core.models import MissionContext, WorkflowCycleOutput

from .bt import plan_digest, require_valid_bt
from .execution import Executor, write_json
from .experience import ExperienceBank
from .planner import BTPlanner


class RefinementRunner:
    def __init__(
        self,
        executor: Executor,
        evaluator: ContinuousStateEvaluatorPipeline,
        reflector: HierarchicalBTReflectorPipeline,
        planner: BTPlanner,
        bank: ExperienceBank,
    ) -> None:
        self.executor, self.evaluator, self.reflector = executor, evaluator, reflector
        self.planner, self.bank = planner, bank

    def run(
        self,
        mission: MissionContext,
        output_dir: str,
        *,
        initial_plan: str | None = None,
        max_refinements: int = 5,
        retriever: str = "mpnet",
        top_k: int = 3,
    ) -> list[WorkflowCycleOutput]:
        if not 0 <= max_refinements <= 5:
            raise ValueError("max_refinements must be 0..5")
        task, scene = mission.metadata.get("task", ""), mission.metadata.get("scene", {})
        if not isinstance(task, str) or not task.strip():
            raise ValueError("A task is required")
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=False)
        run_id = uuid.uuid4().hex
        run_metadata = {
            "run_id": run_id,
            "mission": asdict(mission),
            "max_refinements": max_refinements,
            "status": "running",
        }
        write_json(output / "run.json", run_metadata)
        results = []
        try:
            retrieved = self.bank.retrieve(task, top_k, backend=retriever)
            run_metadata["retrieved_experiences"] = retrieved
            write_json(output / "run.json", run_metadata)
            plan = initial_plan or self.planner.generate(task, scene, retrieved)
            require_valid_bt(plan)
            for cycle in range(max_refinements + 1):
                directory = output / f"cycle-{cycle:03d}"
                directory.mkdir()
                (directory / "plan.xml").write_text(plan, encoding="utf-8")
                snapshot = self.executor.execute(mission, plan, cycle)
                write_json(directory / "snapshot.json", asdict(snapshot))
                if plan_digest(snapshot.state_payload["plan_xml"]) != plan_digest(plan):
                    raise ValueError("Snapshot plan does not match executed plan")
                if snapshot.cycle_index != cycle:
                    raise ValueError("Snapshot cycle does not match execution request")
                evaluation = self.evaluator.evaluate(mission, snapshot)
                write_json(directory / "evaluation.json", asdict(evaluation))
                # Do not spend calls producing a candidate after the execution budget.
                if cycle == max_refinements:
                    from workflow.core.models import ReflectionPlan

                    reflection = ReflectionPlan(
                        cycle,
                        False,
                        "Task succeeded." if evaluation.success else "Refinement budget exhausted.",
                    )
                else:
                    reflection = self.reflector.reflect(mission, snapshot, evaluation)
                result = WorkflowCycleOutput(snapshot, evaluation, reflection)
                results.append(result)
                write_json(directory / "result.json", asdict(result))
                if evaluation.success:
                    self.bank.validate_candidate(plan_digest(plan), run_id)
                    break
                if not reflection.needs_modification or not reflection.candidate_bt_xml:
                    break
                candidate = require_valid_bt(reflection.candidate_bt_xml)
                if plan_digest(candidate) == plan_digest(plan):
                    break
                (directory / "candidate.xml").write_text(candidate, encoding="utf-8")
                self.bank.add(
                    task,
                    mission.metadata.get("task_category", "unspecified"),
                    plan_digest(plan),
                    plan_digest(candidate),
                    run_id,
                    reflection.experience_update.get("Enew", []),
                )
                plan = candidate
            write_json(
                output / "summary.json",
                {
                    "run_id": run_id,
                    "success": bool(results and results[-1].evaluation.success),
                    "executions": len(results),
                    "refinements_executed": max(len(results) - 1, 0),
                    "final_result": results[-1].evaluation.diagnostics.get("result")
                    if results
                    else "unknown",
                },
            )
            run_metadata["status"] = "completed"
            write_json(output / "run.json", run_metadata)
            return results
        except Exception as exc:
            run_metadata["status"] = "failed"
            write_json(output / "run.json", run_metadata)
            write_json(output / "error.json", {"type": type(exc).__name__, "message": str(exc)})
            raise
