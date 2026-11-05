# Validation

## Dependency-free Python checks

```bash
python -m pip install -e .
python -m unittest discover -s srdrone/test -v
python -m srdrone demo --output runs/validation-demo
python -m srdrone validate examples/notebook_detection_failure.xml
```

The suite covers:

- Full failure → analysis → experience → regenerated XML → matching new execution → success/provenance update.
- Whitelist/port/type checks, blackboard typing and conservative fallback dataflow, decorator arity, finite values and bounded repetition.
- Real controller log punctuation, scientific notation, failed states without a pose, observation/ego-pose separation, session boundaries and incomplete plans.
- Strict JSON, unknown outcomes, contradictory success, invalid XML repair, credential-safe provider errors.
- Durable experience deduplication, validation states, lexical retrieval, wrong-run promotion prevention.
- Fresh execution identity checks, ROS log receipt validation and recovery prompt hashes.

Demo model outputs and trajectories are manually authored fixtures. Passing these checks establishes software behavior, not learned model quality or the paper's experimental performance.

## Installed wheel check

Build a wheel, install it into a fresh Python environment, change to a directory outside this repository and run the installed `srdrone demo` command. This checks that recovered prompts and demonstration data are included in the package rather than accidentally loaded from the checkout.

CI runs the Python suite and offline demonstration on Python 3.10 and 3.12. No API credentials, model weights or drone hardware are required for CI.

## Environment-dependent checks

These checks need resources absent from the recovery machine and are not claimed as passed by the reconstruction:

- Real endpoint model calls, response quality, latency and costs.
- Actual `all-mpnet-base-v2` encoding and retrieval quality with an experimental experience bank.
- Linux/ROS1 `catkin build`, updated node launch and sensor/topic verification.
- PX4/RflySim mission execution and physical flight.
- Original paper benchmark, baseline comparisons, 176 failure-case evaluation and simulation-to-real metrics.

The reconstruction intentionally does not invent expected benchmark scores. New experimental runs should retain their task definitions, scenes, exact model IDs, prompts/responses, execution logs, plan hashes and human annotations where required.
