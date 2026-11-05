# Continuous state evaluator

`ContinuousStateEvaluatorPipeline(model=...)` implements the reconstructed §III-B pipeline. It parses existing action-completion/failure logs, retains measured state, reconstructs an auxiliary ideal trajectory, derives CMSR motion/environment semantics, checks execution completeness, and returns task outcome/explanation with three evaluation dimensions.

Use `python -m srdrone evaluate --help`. An actual task, plan and execution log are required. Models implement `srdrone.llm.TextModel`; `ChatModel` calls a configured provider and `RecordedModel` is explicitly for fixtures. Default construction reads the configured model environment; it does not return a placeholder decision.

See [method provenance](../docs/RECOVERY.md), [interfaces](../docs/INTERFACES.md) and [validation](../docs/VALIDATION.md).
