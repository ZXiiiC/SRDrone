from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from workflow.core.interfaces import StateProvider
from workflow.core.models import ExecutionSnapshot, MissionContext
from workflow.parsers.controller_btlog import parse_btlog_sessions, select_session_for_cycle


class ControllerLogStateProvider(StateProvider):
    """Historical replay, not an executor for newly generated plans."""

    def __init__(self, log_path: str, max_lines: int | None = None) -> None:
        self._log_path = Path(log_path)
        if max_lines is not None:
            raise ValueError("Log truncation is disabled: full action history is required")

    def capture(self, mission: MissionContext, cycle_index: int) -> ExecutionSnapshot:
        sessions = parse_btlog_sessions(self._log_path.read_text(encoding="utf-8"))
        selected = select_session_for_cycle(sessions, cycle_index)
        if selected is None:
            raise ValueError("No unused complete historical BTlog session")
        if not selected.filtered_log_text:
            raise ValueError("Selected session contains a plan but no execution events")
        return ExecutionSnapshot(
            datetime.now(timezone.utc),
            cycle_index,
            {
                "mission_id": mission.mission_id,
                "bt_id": mission.bt_id,
                "log_source": str(self._log_path),
                "log_session_index": selected.session_index,
                "log_session_count": len(sessions),
                "plan_xml": selected.plan_xml,
                "filtered_log_text": selected.filtered_log_text,
                "execution_kind": "historical_replay",
                "root_status": "unknown",
            },
        )
