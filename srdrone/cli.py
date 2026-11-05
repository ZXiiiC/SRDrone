"""Command line entry points for reconstruction, replay and external execution."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from evaluator.pipeline import ContinuousStateEvaluatorPipeline
from reflector.pipeline import HierarchicalBTReflectorPipeline
from workflow.core.models import ExecutionSnapshot, MissionContext
from workflow.parsers.controller_btlog import parse_btlog_sessions

from .bt import validate_bt
from .demo import run_demo
from .execution import FileExchangeExecutor, write_json
from .experience import ExperienceBank
from .llm import ChatModel, ModelError
from .planner import BTPlanner
from .runner import RefinementRunner


def build_parser():
    parser = argparse.ArgumentParser(
        description="Reconstructed SRDrone planning and refinement runtime"
    )
    sub = parser.add_subparsers(dest="command", required=True)
    demo = sub.add_parser("demo", help="Run a hand-authored fixture; no API or ROS required")
    demo.add_argument("--output", default="runs/demo")
    validate = sub.add_parser(
        "validate", help="Validate a BT against the audited controller subset"
    )
    validate.add_argument("plan")
    for command in ("plan", "evaluate", "run"):
        p = sub.add_parser(command)
        p.add_argument("--task", required=True)
        p.add_argument("--scene", help="Path to scene JSON; omitted scene is unknown")
        p.add_argument("--mission-id", default="srdrone-mission")
        p.add_argument("--task-category", default="unspecified")
        p.add_argument("--output", required=True, help="Fresh output directory")
        p.add_argument("--experience-bank", default="workflow/data/experience.sqlite3")
        p.add_argument("--retriever", choices=["mpnet", "lexical"], default="mpnet")
        p.add_argument("--top-k", type=int, default=3)
        if command in {"plan", "run"}:
            p.add_argument("--initial-plan", help="Existing XML instead of model generation")
        if command == "evaluate":
            p.add_argument(
                "--log", required=True, help="Full BTlog session file, or raw events with --plan"
            )
            p.add_argument("--plan", help="XML for a standalone event log")
            p.add_argument(
                "--session-index",
                type=int,
                default=-1,
                help="Chronological session index, default latest",
            )
            p.add_argument(
                "--reflect", action="store_true", help="Generate candidate BT; does not execute it"
            )
        if command == "run":
            p.add_argument(
                "--exchange", required=True, help="Fresh directory for external execution adapter"
            )
            p.add_argument("--execution-timeout", type=float, default=600)
            p.add_argument("--max-refinements", type=int, default=5)
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "validate":
            result = validate_bt(Path(args.plan).read_text(encoding="utf-8"))
            print(json.dumps(result.to_dict(), indent=2))
            return 0 if result.valid else 1
        if args.command == "demo":
            outputs = run_demo(args.output)
            print(
                json.dumps(
                    {
                        "kind": "hand_authored_fixture",
                        "executions": len(outputs),
                        "success": outputs[-1].evaluation.success,
                        "output": args.output,
                    },
                    indent=2,
                )
            )
            return 0
        scene = json.loads(Path(args.scene).read_text(encoding="utf-8")) if args.scene else {}
        mission = MissionContext(
            args.mission_id,
            "MainTree",
            metadata={"task": args.task, "scene": scene, "task_category": args.task_category},
        )
        if args.top_k < 0:
            raise ValueError("top-k must be nonnegative")
        output = Path(args.output)
        model = ChatModel.from_env(trace_path=str(output / "model-trace.jsonl"))
        planner = BTPlanner(model)
        bank = ExperienceBank(args.experience_bank)
        if args.command == "run":
            if output.exists():
                raise ValueError("Use a fresh output directory")
            executor = FileExchangeExecutor(args.exchange, timeout=args.execution_timeout)
            runner = RefinementRunner(
                executor,
                ContinuousStateEvaluatorPipeline(model=model),
                HierarchicalBTReflectorPipeline(model=model),
                planner,
                bank,
            )
            initial = (
                Path(args.initial_plan).read_text(encoding="utf-8") if args.initial_plan else None
            )
            outputs = runner.run(
                mission,
                str(output),
                initial_plan=initial,
                max_refinements=args.max_refinements,
                retriever=args.retriever,
                top_k=args.top_k,
            )
            print((output / "summary.json").read_text(encoding="utf-8"))
            return 0 if outputs[-1].evaluation.success else 2
        output.mkdir(parents=True, exist_ok=False)
        if args.command == "plan":
            retrieved = bank.retrieve(args.task, args.top_k, backend=args.retriever)
            if args.initial_plan:
                from .bt import require_valid_bt

                xml = require_valid_bt(Path(args.initial_plan).read_text(encoding="utf-8"))
            else:
                xml = planner.generate(args.task, scene, retrieved)
            (output / "plan.xml").write_text(xml, encoding="utf-8")
            write_json(output / "retrieval.json", retrieved)
            print(str(output / "plan.xml"))
            return 0
        raw = Path(args.log).read_text(encoding="utf-8")
        if args.plan:
            xml, log = Path(args.plan).read_text(encoding="utf-8"), raw
        else:
            sessions = parse_btlog_sessions(raw)
            if not sessions or not -len(sessions) <= args.session_index < len(sessions):
                raise ValueError("No session at requested index")
            session = sessions[args.session_index]
            xml, log = session.plan_xml, session.filtered_log_text
        snapshot = ExecutionSnapshot(
            datetime.now(timezone.utc),
            0,
            {
                "plan_xml": xml,
                "filtered_log_text": log,
                "log_source": args.log,
                "execution_kind": "historical_replay",
                "root_status": "unknown",
            },
        )
        evaluator = ContinuousStateEvaluatorPipeline(model=model)
        evaluation = evaluator.evaluate(mission, snapshot)
        write_json(output / "evaluation.json", asdict(evaluation))
        if args.reflect:
            reflection = HierarchicalBTReflectorPipeline(model=model).reflect(
                mission, snapshot, evaluation
            )
            write_json(output / "reflection.json", asdict(reflection))
            if reflection.candidate_bt_xml:
                (output / "candidate.xml").write_text(reflection.candidate_bt_xml, encoding="utf-8")
        print(
            json.dumps(
                {
                    "result": evaluation.diagnostics["result"],
                    "summary": evaluation.summary,
                    "execution_kind": "historical_replay",
                    "output": str(output),
                },
                indent=2,
            )
        )
        return 0
    except (ValueError, OSError, RuntimeError, ModelError, TimeoutError) as exc:
        print(f"srdrone: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
