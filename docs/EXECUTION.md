# Execution and ROS1 integration

## Python runtime boundary

`python -m srdrone evaluate` analyzes an existing log. It can generate a candidate XML, but does not execute it. `python -m srdrone run` requires a fresh execution for every candidate. Historical sessions are never substituted for a new execution.

The loop uses the injectable `Executor.execute(mission, plan_xml, cycle_index)` interface. Adapters must return an `ExecutionSnapshot` with the actual plan XML, the correct cycle, raw completion/failure events, and the observed root status when available.

The built-in `FileExchangeExecutor` publishes each request in a fresh directory:

```text
runs/exchange/cycle-000/
  plan.xml
  request.json
  result.json          # written atomically by the execution adapter
```

`request.json` includes `mission_id`, `cycle_index`, `plan_digest`, `task`, `scene`, and `plan_file`. The adapter executes `plan.xml` once, collects fresh action-centric logs, and atomically writes:

```json
{
  "mission_id": "srdrone-mission",
  "cycle_index": 0,
  "plan_digest": "copy-the-digest-from-request.json-after-verifying-the-executed-plan",
  "root_status": "SUCCESS",
  "filtered_log_text": "Takeoff finished. drone pose[nwu,yaw]:(0,0,1.5,0)\n..."
}
```

The example above specifies the result format, not a runnable mission or a success claim. Valid root statuses are `SUCCESS`, `FAILURE` and `UNKNOWN`. Root success means the tree finished successfully; task success is evaluated separately. The result must contain a nonempty log. Wrong mission/cycle/digest, empty logs and explicit `error.json` abort the loop. Timeout aborts instead of recycling an earlier result.

The adapter must verify the plan actually used by the controller, not simply echo a digest after running another file. `srdrone.ros_adapter.collect_result` compares the plan embedded in the controller log with the request. The file protocol assumes a trusted local adapter.

## Prepared ROS1 workspace

The original flight stack targets Linux/ROS1, MAVROS/PX4 and BehaviorTree.CPP v4. The Python package is independently installable. ROS requirements include OpenCV, PCL, `cv_bridge`, custom `common_msgs`, and the external planner's `quadrotor_msgs/PositionCommand` headers. Sensor/localization/planner components and RflySim assets must be supplied/configured separately.

In a catkin workspace with this repository and the required external packages:

```bash
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

Existing `sh/` scripts contain historical machine paths. Configure the appropriate sensor, localization, planner and MAVROS components for your environment. Run a single controller instance per vehicle.

The controller now reads private `~bt_tree_path` and `~bt_log_file` parameters. Both supplied launch files expose these arguments:

```bash
roslaunch controller sim.launch \
  bt_tree_path:=/absolute/path/to/plan.xml \
  bt_log_file:=/absolute/path/to/fresh-BTlog.txt
```

The launch files default to the repository's `controller/config/mav.xml` and `/tmp/srdrone-BTlog.txt`. Direct `rosrun` invocation must supply both private parameters. Completion adds `BT_ROOT_STATUS: SUCCESS` or `FAILURE` to the log; `TakeOff` also records a measured keyframe. The controller still runs the original flight behavior, including arming/mode changes and the existing Land semantics. These integration edits do not establish readiness for physical flight.

## End-to-end ROS adapter

In a prepared simulation environment, start the runtime in one terminal:

```bash
python -m srdrone run \
  --task 'Find a square frame, pass through it, and land.' \
  --initial-plan /absolute/path/to/initial.xml \
  --exchange runs/ros-exchange --output runs/ros-mission \
  --max-refinements 5 --execution-timeout 600
```

Start the adapter in another terminal using the same exchange directory and sourced catkin environment:

```bash
python -m srdrone.ros_adapter \
  --exchange runs/ros-exchange --launch sim \
  --execution-timeout 300 --idle-timeout 600 --max-executions 6
```

The adapter launches the existing controller using an argument list without a shell. Each execution gets its own plan, BTlog and ROS output file. It checks the embedded plan, requires exactly one final root-status marker, and atomically publishes the result. On timeout/interruption it stops only its launch process group. An execution error is saved to `error.json` and returned to the waiting runner. After the run, the adapter exits when idle timeout or its execution limit is reached; it can also be stopped manually.

Use `--launch control` only with a separately commissioned physical platform; this command can arm and move the drone. This reconstruction has not been validated on the physical platform or in PX4/RflySim. The notebook's original trained experience base is unavailable, so a new installation does not inherit the paper's simulation-to-real validation.

## Artifacts and experience state

A run saves the input plan, raw snapshot, CMSR/evaluation outputs, reflection, validated candidate XML, retrieved experience, and summary. The model trace contains prompts and responses and can include mission data; credentials are excluded.

Generated experience enters SQLite as `proposed`. A later successful execution of that candidate in the same run promotes matching entries to `execution_validated`. A failure, timeout, XML rejection, unknown result, unchanged candidate or exhausted budget cannot promote an entry. The status records the evaluator's assessment of the matching execution, not a human annotation or a safety certificate.
