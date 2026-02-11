# Repository Guidelines

## Project Structure & Module Organization
- `controller/`: C++17 flight-control package; main node in `src/ros1_node.cpp`, behavior-tree and runtime params in `config/`, ROS launch files in `launch/`.
- `object_det/`: Python detection package; runnable nodes and YOLO utilities in `scripts/`.
- `common_msgs/`: shared ROS message definitions in `msg/`.
- `sensor_pkg/`: sensor adapters, utility scripts, and RViz config (`rflysim.rviz`).
- `recognize_aruco/`: ArUco-based perception script(s).
- `evaluator/` and `reflector/`: scaffolded modules with roadmap docs.
- `sh/`: orchestration scripts for simulation and real-flight bring-up.

## Build, Test, and Development Commands
Run from your catkin workspace root (one level above this repo):

```bash
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

Common runtime commands:

```bash
bash sh/start_sim.sh          # full simulation stack
bash sh/start_control.sh      # real-flight stack (hardware-dependent)
roslaunch controller sim.launch
bash sh/kill_ros_pid.sh       # cleanup ROS processes
```

## Coding Style & Naming Conventions
- C++: use C++17 and Google-style conventions; run `clang-format` before opening a PR.
- Python: follow PEP 8, 4-space indentation, and `snake_case` for functions/variables.
- ROS naming: keep topics, params, and message fields descriptive and lowercase with underscores.
- Keep ROS package layout standard (`launch/`, `config/`, `msg/`, `scripts/`) when adding files.

## Testing Guidelines
- There is no full CI test suite yet; use build + runtime smoke checks.
- Minimum validation for code changes:
  - `catkin build` succeeds.
  - Updated node launches without immediate errors (`roslaunch ...`).
  - Relevant topics publish expected messages (for example `/objects` for detection paths).
- For new test coverage, place tests under package-local `test/` and use `rostest`/`catkin_add_gtest` where appropriate.

## Commit & Pull Request Guidelines
- Prefer Conventional Commits (`feat:`, `fix:`, `docs:`); scoped form is encouraged (for example `feat(controller): ...`).
- Keep commits focused by module and include related config/launch updates together.
- PRs should include: intent, affected packages, validation commands run, and logs/screenshots for behavior changes.
- Link relevant issues or discussions when available.

## Configuration & Safety Notes
- Several launch scripts contain machine-specific absolute paths (for example `/root/...` or `/home/nvidia/...`); adapt locally and avoid committing personal path changes.
- Do not commit generated artifacts (`build/`, `devel/`), bag files, secrets, or ad-hoc model binaries.
