from __future__ import annotations

import argparse

from workflow.core.experience_store import ExperienceStore
from workflow.core.models import MissionContext
from workflow.core.orchestrator import WorkflowOrchestrator
from workflow.implementations.controller_log_state_provider import ControllerLogStateProvider
from workflow.implementations.state_provider_stub import ActionCentricStateProviderStub

from evaluator.pipeline import ContinuousStateEvaluatorPipeline
from reflector.pipeline import HierarchicalBTReflectorPipeline


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run SRDrone workflow scaffold")
    parser.add_argument("--mission-id", default="demo_mission")
    parser.add_argument("--bt-id", default="main_bt")
    parser.add_argument("--bt-path", default="controller/config/mav.xml")
    parser.add_argument("--max-cycles", type=int, default=3)
    parser.add_argument(
        "--experience-store",
        default="workflow/data/experience_records.jsonl",
    )
    parser.add_argument(
        "--state-source",
        choices=["controller_log", "stub"],
        default="controller_log",
        help="Input source for execution state. Default uses controller filtered logs.",
    )
    parser.add_argument(
        "--controller-log-path",
        default="controller/config/BTlog.txt",
        help="Path to controller filtered log file.",
    )
    parser.add_argument(
        "--controller-log-max-lines",
        type=int,
        default=120,
        help="Tail line count read from controller log each cycle.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    mission = MissionContext(
        mission_id=args.mission_id,
        bt_id=args.bt_id,
        bt_path=args.bt_path,
    )

    if args.state_source == "controller_log":
        state_provider = ControllerLogStateProvider(
            log_path=args.controller_log_path,
            max_lines=args.controller_log_max_lines,
        )
    else:
        state_provider = ActionCentricStateProviderStub()

    orchestrator = WorkflowOrchestrator(
        state_provider=state_provider,
        evaluator=ContinuousStateEvaluatorPipeline(),
        reflector=HierarchicalBTReflectorPipeline(),
        experience_store=ExperienceStore(args.experience_store),
    )

    outputs = orchestrator.run(mission=mission, max_cycles=args.max_cycles)
    print(f"[workflow] mission={mission.mission_id} cycles={len(outputs)}")
    for cycle in outputs:
        print(
            "[cycle]"
            f" idx={cycle.snapshot.cycle_index}"
            f" eval_success={cycle.evaluation.success}"
            f" reflect_needs_mod={cycle.reflection.needs_modification}"
        )


if __name__ == "__main__":
    main()
