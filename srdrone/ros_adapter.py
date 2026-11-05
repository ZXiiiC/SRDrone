"""Optional ROS1 adapter for the file-exchange executor.

This launches the existing controller, which can arm and move a drone. Invoke only
in an explicitly prepared ROS/PX4 environment. It is not used by the offline demo.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path

from workflow.parsers.controller_btlog import parse_btlog_sessions

from .bt import plan_digest, require_valid_bt
from .execution import write_json


def collect_result(directory: Path, request: dict) -> dict:
    sessions = parse_btlog_sessions(
        (directory / "controller-BTlog.txt").read_text(encoding="utf-8")
    )
    if len(sessions) != 1 or plan_digest(sessions[0].plan_xml) != request["plan_digest"]:
        raise ValueError("Controller log does not match the single requested plan")
    log = sessions[0].filtered_log_text
    statuses = re.findall(r"^BT_ROOT_STATUS:\s*(SUCCESS|FAILURE)\s*$", log, re.M)
    if len(statuses) != 1:
        raise ValueError("Controller did not produce exactly one final root status")
    return {
        "mission_id": request["mission_id"],
        "cycle_index": request["cycle_index"],
        "plan_digest": request["plan_digest"],
        "root_status": statuses[0],
        "filtered_log_text": log,
    }


def execute_request(directory: Path, launch: str, timeout: float) -> None:
    request = json.loads((directory / "request.json").read_text(encoding="utf-8"))
    plan = (directory / "plan.xml").resolve()
    xml = require_valid_bt(plan.read_text(encoding="utf-8"))
    if plan_digest(xml) != request["plan_digest"]:
        raise ValueError("Request digest does not match plan.xml")
    log = (directory / "controller-BTlog.txt").resolve()
    if log.exists():
        raise ValueError("Controller log already exists; execution must be fresh")
    command = [
        "roslaunch",
        "controller",
        launch + ".launch",
        "bt_tree_path:=" + str(plan),
        "bt_log_file:=" + str(log),
    ]
    # No shell, no ad-hoc command strings; stop only this launch process group.
    with (directory / "roslaunch-output.txt").open("w", encoding="utf-8") as output:
        process = subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True
        )
        try:
            returncode = process.wait(timeout=timeout)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    if returncode != 0:
        raise RuntimeError(f"roslaunch exited with code {returncode}; inspect roslaunch-output.txt")
    write_json(directory / "result.json", collect_result(directory, request))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run requested BTs through the ROS1 controller")
    parser.add_argument("--exchange", required=True)
    parser.add_argument("--launch", choices=["sim", "control"], default="sim")
    parser.add_argument("--execution-timeout", type=float, default=300)
    parser.add_argument("--idle-timeout", type=float, default=600)
    parser.add_argument("--max-executions", type=int, default=6)
    args = parser.parse_args(argv)
    if min(args.execution_timeout, args.idle_timeout, args.max_executions) <= 0:
        parser.error("Timeouts and max-executions must be positive")
    exchange = Path(args.exchange)
    deadline = time.monotonic() + args.idle_timeout
    completed = 0
    while completed < args.max_executions:
        pending = sorted(
            p
            for p in exchange.glob("cycle-*/request.json")
            if not (p.parent / "result.json").exists() and not (p.parent / "error.json").exists()
        )
        if pending:
            directory = pending[0].parent
            try:
                execute_request(directory, args.launch, args.execution_timeout)
            except Exception as exc:
                write_json(
                    directory / "error.json", {"type": type(exc).__name__, "message": str(exc)}
                )
                return 1
            completed += 1
            deadline = time.monotonic() + args.idle_timeout
        elif time.monotonic() >= deadline:
            return 0 if completed else 1
        else:
            time.sleep(0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
