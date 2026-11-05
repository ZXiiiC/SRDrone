# Workflow compatibility and historical logs

`python -m workflow.cli` now exposes the same `demo`, `validate`, `plan`, `evaluate`, and `run` subcommands as `python -m srdrone`.

Shared dataclasses remain in `workflow/core/models.py`. The historical BTlog session parser accepts complete single-line or multiline root blocks and separates each plan from its following events. It does not attach partial next-plan content to an earlier execution or repeat exhausted sessions.

`ControllerLogStateProvider` and `WorkflowOrchestrator` are retained as historical-analysis helpers. They do not execute generated plans. The new execution loop is `srdrone.runner.RefinementRunner`; it uses fresh matching execution snapshots and manages candidate experience provenance. Earlier stub modules remain for source-history compatibility and are unused by the current CLI.

See the root [README](../README.md), [execution contract](../docs/EXECUTION.md), and [recovery mapping](../docs/RECOVERY.md).
