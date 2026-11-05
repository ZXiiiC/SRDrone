# Implementation of paper sections III-B and III-C

The original text-first scaffold has been replaced by a reconstructed model-driven implementation. Explanatory text remains in stage outputs, with explicit machine-readable output contracts.

- §III-B: `evaluator/core/stages.py`, recovered prompts, raw action-state parser and CMSR representation.
- §III-C: `reflector/core/stages.py`, hierarchical experience and controller strategy space.
- BT generation/validation: `srdrone/planner.py`, `srdrone/bt.py`.
- Experience base and retrieval: `srdrone/experience.py`.
- Actual execution/refinement: `srdrone/runner.py`, `srdrone/execution.py`, optional `srdrone/ros_adapter.py`.
- Historical replay: `workflow/parsers/controller_btlog.py`, separate from new executions.

See [RECOVERY.md](../docs/RECOVERY.md) for source-to-code mapping and engineering choices, and [EXECUTION.md](../docs/EXECUTION.md) for the exact adapter contract.
