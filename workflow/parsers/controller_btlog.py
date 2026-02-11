from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class BTLogSession:
    """One controller log session: plan XML + filtered action log."""

    session_index: int
    plan_xml: str
    filtered_log_text: str
    plan_start_line: int
    plan_end_line: int


def parse_btlog_sessions(raw_text: str) -> List[BTLogSession]:
    """Split BTlog into sessions.

    Session format (as observed in controller log):
      1) one `<root ...> ... </root>` plan block
      2) following lines as filtered log
      3) next `<root ...>` begins a new session
    """

    lines = raw_text.splitlines()
    sessions: List[BTLogSession] = []

    cursor = 0
    session_id = 0
    total_lines = len(lines)

    while cursor < total_lines:
        plan_start = _find_next_root_start(lines, cursor)
        if plan_start is None:
            break

        plan_end = _find_root_end(lines, plan_start)
        if plan_end is None:
            plan_end = total_lines - 1

        next_plan_start = _find_next_root_start(lines, plan_end + 1)
        if next_plan_start is None:
            filtered_lines = lines[plan_end + 1 :]
            cursor = total_lines
        else:
            filtered_lines = lines[plan_end + 1 : next_plan_start]
            cursor = next_plan_start

        plan_xml = "\n".join(lines[plan_start : plan_end + 1]).strip()
        filtered_log_text = "\n".join(filtered_lines).strip()

        sessions.append(
            BTLogSession(
                session_index=session_id,
                plan_xml=plan_xml,
                filtered_log_text=filtered_log_text,
                plan_start_line=plan_start + 1,
                plan_end_line=plan_end + 1,
            )
        )
        session_id += 1

    return sessions


def select_session_for_cycle(
    sessions: List[BTLogSession],
    cycle_index: int,
) -> Optional[BTLogSession]:
    """Select session for a workflow cycle.

    Policy: cycle 0 -> latest session, cycle 1 -> second latest, etc.
    """

    if not sessions:
        return None

    reverse_index = min(max(cycle_index, 0), len(sessions) - 1)
    return sessions[-1 - reverse_index]


def _find_next_root_start(lines: List[str], start: int) -> Optional[int]:
    for index in range(start, len(lines)):
        if lines[index].lstrip().startswith("<root"):
            return index
    return None


def _find_root_end(lines: List[str], start: int) -> Optional[int]:
    for index in range(start, len(lines)):
        if lines[index].strip() == "</root>":
            return index
    return None

