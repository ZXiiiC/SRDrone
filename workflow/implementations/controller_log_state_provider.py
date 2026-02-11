from __future__ import annotations

from datetime import datetime
from pathlib import Path

from workflow.parsers.controller_btlog import parse_btlog_sessions, select_session_for_cycle

from workflow.core.interfaces import StateProvider
from workflow.core.models import ExecutionSnapshot, MissionContext


class ControllerLogStateProvider(StateProvider):
    """State provider that reads action-centric filtered logs from controller output."""

    def __init__(self, log_path: str, max_lines: int = 120) -> None:
        self._log_path = Path(log_path)
        self._max_lines = max_lines

    def capture(self, mission: MissionContext, cycle_index: int) -> ExecutionSnapshot:
        if not self._log_path.exists():
            filtered_text = (
                f"[controller_log_missing] path={self._log_path}. "
                "No filtered action log found for this cycle."
            )
            plan_xml = ""
            session_index = -1
            session_count = 0
            source_exists = False
        else:
            raw_text = self._log_path.read_text(encoding="utf-8", errors="ignore")
            sessions = parse_btlog_sessions(raw_text)
            session_count = len(sessions)
            selected = select_session_for_cycle(sessions=sessions, cycle_index=cycle_index)

            if selected is None:
                plan_xml = ""
                session_index = -1
                filtered_text = (
                    "[controller_log_parse_empty] No '<root>... </root>' session parsed from BTlog."
                )
            else:
                plan_xml = selected.plan_xml
                session_index = selected.session_index
                filtered_lines = selected.filtered_log_text.splitlines()
                filtered_window = filtered_lines[-self._max_lines :] if filtered_lines else []
                filtered_text = "\n".join(filtered_window)

            source_exists = True

        return ExecutionSnapshot(
            timestamp=datetime.utcnow(),
            cycle_index=cycle_index,
            state_payload={
                "mission_id": mission.mission_id,
                "bt_id": mission.bt_id,
                "status": "controller_filtered_log",
                "log_source": str(self._log_path),
                "log_source_exists": source_exists,
                "log_session_index": session_index,
                "log_session_count": session_count,
                "plan_xml": plan_xml,
                "filtered_log_text": filtered_text,
                "notes": "Action-Centric filtering is provided by controller log sessions.",
            },
        )
