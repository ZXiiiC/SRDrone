# SRDrone Workflow Scaffold

This directory contains the **workflow framework** for the SRDrone open-source release, aligned with the paper pipeline:

1. **Task Execution Phase** (existing controller/perception stack)
2. **Continuous State Evaluation** (paper §3.2, interface scaffold only)
3. **Hierarchical BT Modification** (paper §3.3, interface scaffold only)
4. **Experience Base Update** (recording loop outcomes)

> Current status: this is a runnable scaffold. Core evaluator/reflector algorithms are intentionally left as stubs for later implementation.

## Output Convention

- Workflow stages follow a **text-first** convention: LLM-facing outputs are primarily free-form text.
- Mathematical symbols from the paper are used for semantic clarity, not as strict serialization formats.
- Optional structured fields exist for engineering post-processing and observability.

## State Source Convention

- Stage-A (`Action-Centric State Filtering`) defaults to **controller-produced filtered logs**.
- Default log path: `controller/config/BTlog.txt`.
- This avoids re-implementing filtering logic already present in controller.
- BTlog session parsing rule:
  - one `<root ...> ... </root>` block = plan of one run
  - following lines until next `<root ...>` = filtered log of that run
  - cycle selection policy: latest session first

## Design Goals

- Keep `controller/`, `common_msgs/`, and other low-level packages untouched.
- Provide clear extension points for §3.2 and §3.3 implementations.
- Support iterative execution → evaluation → reflection cycles.
- Persist cycle artifacts for later learning/analysis.

## Directory Layout

- `core/models.py`: workflow data contracts.
- `core/interfaces.py`: abstract interfaces (`StateProvider`, `Evaluator`, `Reflector`).
- `core/orchestrator.py`: main closed-loop orchestration.
- `core/experience_store.py`: JSONL persistence for experience records.
- `implementations/*_stub.py`: temporary placeholder implementations.
- `config/default_workflow.yaml`: runtime config template.
- `cli.py`: local runner for scaffold validation.

## Quick Start

From repository root:

```bash
python3 -m workflow.cli --mission-id demo_mission --max-cycles 3
```

Explicit controller-log mode:

```bash
python3 -m workflow.cli \
  --state-source controller_log \
  --controller-log-path controller/config/BTlog.txt \
  --controller-log-max-lines 120
```

Or:

```bash
python3 workflow/scripts/run_workflow.py --mission-id demo_mission --max-cycles 3
```

## Integration Boundaries

- The scaffold can **read** execution-state inputs from adapters, but does **not** patch BT XML automatically.
- Reflection output is a **proposed plan/spec**, not direct mutation.
- Future implementation should replace:
  - `ContinuousStateEvaluatorPipeline` internals with paper §3.2 full algorithms.
  - `HierarchicalBTReflectorPipeline` internals with paper §3.3 full algorithms.
