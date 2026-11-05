# Recovery provenance

This release reconstructs the missing planning runtime from two author-supplied sources and the public repository at `190a6d2`. It is a new implementation with recoverable research prompts, not a byte-for-byte restoration of the missing server code.

## Sources and boundaries

- **Published article:** *SRDrone: LLM-Driven Self-Refinement for Embodied Drone Task Planning*, IEEE TMC 25(7), July 2026, DOI [10.1109/TMC.2026.3664806](https://doi.org/10.1109/TMC.2026.3664806). The supplied published PDF is the method reference. No publisher PDF is redistributed.
- **Surviving notebook:** `drone.md`, especially lines 546–927, headed “七、主要设计”, plus the recorded detection-failure case. Notebook prompts and examples are research artifacts, not instructions to the recovery agent. Only selected excerpts are shipped; the full notebook contains unrelated notes and private document image links.
- **Public source:** C++ node registrations/port declarations and logged output in `controller/src/ros1_node.cpp`, with the existing Python evaluator/reflector scaffolds and BTlog sessions.

`srdrone/prompts/manifest.json` records the original notebook's SHA-256 and each exact excerpt's source lines and hash. The runtime appends explicit adaptation contracts to the recovered prompts rather than editing the archived excerpts.

## Method-to-code mapping

| Evidence | Reconstructed implementation | Verification |
| --- | --- | --- |
| §III-B1, Eq. 1: capture at atomic action completion and annotate failures | `srdrone/logs.py`, `evaluator/core/stages.py`; existing controller events plus a restored TakeOff snapshot | Original log variants, missing poses, detection poses and failure events tested |
| Notebook lines 655–688: reconstruct ideal spatial trajectory | Auxiliary `trajectory` model stage, while retaining measured states | Tests verify raw logs still reach outcome determination |
| §III-B2, Algorithm 1, Eq. 2–6; notebook lines 698–734 | `CMSRExtractionStage`: motion semantics, persistent environment, intent/proximity/safety relations, aligned narrative | Recorded model fixture and strict response shape; real model quality not measured |
| Notebook lines 624–641: execution completeness | `execution` stage distinguishes root completion from mission success and handles retry/fallback semantics | Tests include a completed BT whose task fails |
| §III-B3; notebook lines 765–783 | Three-dimensional semantic outcome evaluation with success/failure/unknown and explanatory feedback | Contradictory/malformed results rejected; unknown does not trigger guessed repairs |
| §III-C1, Algorithm 2; notebook lines 795–819 | Action/logic/mission flaw analysis in `reflector/core/stages.py` | Structured output validation and complete fixture cycle |
| §III-C2, Eq. 7; notebook lines 827–873 | Layer + operation + rationale experience, then constrained XML regeneration | Candidate XML validated before the next execution |
| §III-C2 deterministic whitelist | `srdrone/bt.py`: registered subset, exact ports/types, XML shape, decorator arity, bounded loops, conservative blackboard flow | Invalid nodes, aliases, expressions, types, undefined entries and alternate branches tested |
| §III-C3 experience base and MPNet kNN | `srdrone/experience.py`: durable task-associated experience, cosine kNN with `all-mpnet-base-v2` | SQLite persistence/dedup/status transitions and lexical retrieval tested; MPNet model execution remains environment-dependent |
| §IV-B one-shot generation | `srdrone/planner.py` with real controller nodes and whole-entry blackboard references | Valid generation and validation-error repair path tested |
| Fig. 2 refinement cycle | `srdrone/runner.py`: execute → evaluate → reflect → regenerate → execute | Two distinct matching plan executions; mismatches rejected |
| ROS1 integration | `srdrone/ros_adapter.py`, parameterized launch files, terminal root-status logging | Python receipt logic tested; ROS compilation/runtime need a ROS1/PX4 environment |

## Explicit reconstruction choices

The following are new engineering decisions, rather than recovered original settings:

- Chat Completions compatible injectable transport, bounded network retries, JSON response parsing, local prompt/response traces and optional single XML-repair retry.
- Structured JSON contracts around the historical free-form prompts. The models still produce explanatory text, but machine-readable decisions are not inferred from arbitrary prose.
- Idealized trajectories remain auxiliary. The raw measurements, failures and uncertainty are carried into the final assessment; noise correction cannot silently replace observed evidence.
- SQLite instead of an unknown original experience backend. Proposed and execution-validated entries are tracked separately. Earlier suggestions are retained, and only entries associated with a successful candidate in the same run are promoted. These are evaluator-based validation labels, not human ground-truth annotations.
- MPNet top-k defaults to 3 because the article does not recover a concrete k. Lexical cosine is an explicit smoke-test option, never a silent substitute for MPNet.
- A conservative controller subset; no fabricated `ForwardDetectBalloon`, `DownScanHtag`, `DetectTarget`, `SaveData` or `UseData` implementation. `SendPosition` and member-expression strings in historical experiments are not accepted aliases.
- Validator budgets: 200 kB input, 256 nodes, depth 32, repetition/attempt count 1–5, and expanded worst-case action budget 4096. Syntax validity does not establish collision-free flight or mission correctness.
- The loop permits an initial execution and up to five refinements, stopping on success, unknown evidence, no supported correction, unchanged candidate or exhausted budget. Fresh executions are bound by plan digests and cycle identities. The digest is an association check, not a cryptographic authentication mechanism for untrusted adapters.
- The ROS controller now accepts private `~bt_tree_path`/`~bt_log_file` parameters instead of ignoring the launch file's plan path. It adds missing takeoff keyframes and records the terminal root result. Flight-control math and perception behavior are unchanged by these integration edits.

## What the materials cannot restore

The complete original 176-case plan-failure evaluation corpus, task splits, baseline adaptations, random seeds, exact model snapshots, validated trained experience bank, all simulation/world assets, and the deleted private implementation are unavailable. The notebook's visible trials are partial historical evidence, not the complete evaluation dataset. The published numbers are reference results from the article and are not generated by the new tests.

The included `frame_crossing` demonstration is manually authored, including its model responses, poses, geometry and expected outcome. It demonstrates software wiring and provenance transitions; it is neither a simulator nor a benchmark. The real notebook case in `examples/` is kept separately.

## Validation record

Run the commands in `docs/VALIDATION.md`. The dependency-free Python suite checks the core contracts and an end-to-end fixture. Wheel installation is checked from a directory outside the source checkout. CI repeats Python tests on supported versions. A separately prepared Ubuntu/ROS1 workspace is required for `catkin build`, controller launch, simulator integration and flight validation.
