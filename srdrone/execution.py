"""Execution boundary: plans and fresh logs are paired by digest and cycle."""

from __future__ import annotations

import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Protocol

from workflow.core.models import ExecutionSnapshot, MissionContext

from .bt import plan_digest, require_valid_bt


class Executor(Protocol):
    def execute(
        self, mission: MissionContext, plan_xml: str, cycle_index: int
    ) -> ExecutionSnapshot: ...


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, indent=2, ensure_ascii=False, default=str) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


class FileExchangeExecutor:
    """Wait for a simulator/controller adapter to execute each requested plan.

    Each run requires a new directory. The adapter atomically writes result.json
    only after execution ends; stale/mismatched results are rejected. This class
    does not arm hardware or launch ROS. See docs/EXECUTION.md for the contract.
    """

    def __init__(self, directory: str, timeout: float = 600, poll_interval: float = 0.5) -> None:
        self.directory = Path(directory)
        if self.directory.exists():
            raise ValueError("Execution exchange requires a fresh directory")
        if timeout <= 0 or poll_interval <= 0:
            raise ValueError("Execution timeout and polling interval must be positive")
        self.directory.mkdir(parents=True)
        self.timeout, self.poll_interval = timeout, poll_interval

    def execute(
        self, mission: MissionContext, plan_xml: str, cycle_index: int
    ) -> ExecutionSnapshot:
        require_valid_bt(plan_xml)
        directory = self.directory / f"cycle-{cycle_index:03d}"
        directory.mkdir()
        (directory / "plan.xml").write_text(plan_xml, encoding="utf-8")
        digest = plan_digest(plan_xml)
        write_json(
            directory / "request.json",
            {
                "mission_id": mission.mission_id,
                "cycle_index": cycle_index,
                "plan_digest": digest,
                "task": mission.metadata.get("task", ""),
                "scene": mission.metadata.get("scene", {}),
                "plan_file": "plan.xml",
            },
        )
        deadline = time.monotonic() + self.timeout
        result_path = directory / "result.json"
        while not result_path.exists():
            error_path = directory / "error.json"
            if error_path.exists():
                raise RuntimeError(
                    "Execution adapter failed: " + error_path.read_text(encoding="utf-8")
                )
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Waiting for execution result: {result_path}")
            time.sleep(self.poll_interval)
        result = json.loads(result_path.read_text(encoding="utf-8"))
        if (
            result.get("plan_digest") != digest
            or result.get("cycle_index") != cycle_index
            or result.get("mission_id") != mission.mission_id
        ):
            raise ValueError("Execution result does not match the requested plan/mission/cycle")
        if result.get("root_status") not in {"SUCCESS", "FAILURE", "UNKNOWN"}:
            raise ValueError("Invalid root status from executor")
        log = result.get("filtered_log_text")
        if not isinstance(log, str) or not log.strip():
            raise ValueError("Executor returned an empty execution log")
        return ExecutionSnapshot(
            datetime.now(timezone.utc),
            cycle_index,
            {
                "plan_xml": plan_xml,
                "filtered_log_text": log,
                "plan_digest": digest,
                "root_status": result["root_status"],
                "log_source": str(result_path),
                "execution_kind": "external_adapter",
            },
        )
