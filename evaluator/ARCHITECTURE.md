# Evaluator architecture

1. `ActionCentricStateCaptureStage`: parse controller-filtered completion/failure events, retaining raw logs and missing observations.
2. `CMSRExtractionStage`: recovered trajectory prompt followed by Algorithm-1 motion/spatial reasoning; auxiliary ideal reconstruction is kept separate from measured evidence.
3. `TaskOutcomeDeterminationStage`: execution completeness, then task requirement/trajectory comparison with intent, proximity and safety dimensions.

`EvaluationResult` contains explanatory narratives, structured diagnostics and all stage outputs. The result enum is success/failure/unknown; the success boolean is true only for explicit success. Missing logs, malformed model responses and contradictory success/failure dimensions cannot silently succeed.

Full provenance and reconstruction choices: [RECOVERY.md](../docs/RECOVERY.md).
