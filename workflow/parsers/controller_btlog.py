"""Session parser for historical controller BTlog files (replay only)."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional

from srdrone.bt import parse_xml


@dataclass
class BTLogSession:
    session_index: int
    plan_xml: str
    filtered_log_text: str
    plan_start_line: int
    plan_end_line: int


def parse_btlog_sessions(raw_text: str) -> list[BTLogSession]:
    """Accept single/multiline XML, isolate logs from the next (even partial) plan.

    Incomplete or malformed XML is not a session. A complete plan with an empty
    log remains visible to the caller as an unexecuted/incomplete run.
    """
    starts = list(re.finditer(r"<root\b", raw_text))
    sessions = []
    for index, start in enumerate(starts):
        end = starts[index + 1].start() if index + 1 < len(starts) else len(raw_text)
        block = raw_text[start.start() : end]
        close = re.search(r"</root\s*>", block)
        if not close:
            continue
        xml = block[: close.end()]
        try:
            parse_xml(xml)
        except (ET.ParseError, ValueError):
            continue
        sessions.append(
            BTLogSession(
                session_index=index,
                plan_xml=xml,
                filtered_log_text=block[close.end() :].strip(),
                plan_start_line=raw_text[: start.start()].count("\n") + 1,
                plan_end_line=raw_text[: start.start() + close.end()].count("\n") + 1,
            )
        )
    return sessions


def select_session_for_cycle(
    sessions: list[BTLogSession], cycle_index: int
) -> Optional[BTLogSession]:
    """Latest-first historical replay; never repeat oldest data on exhaustion."""
    if not 0 <= cycle_index < len(sessions):
        return None
    return sessions[-1 - cycle_index]
