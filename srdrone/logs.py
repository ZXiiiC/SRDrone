"""Parse completion/failure keyframes, preserving raw measurements and gaps."""

from __future__ import annotations

import math
import re

from evaluator.core.models import ActionAnnotatedState

NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
ALIASES = {"Takeoff": "TakeOff"}


def parse_action_states(log: str) -> list[ActionAnnotatedState]:
    result: list[ActionAnnotatedState] = []
    observations: dict[str, dict] = {}
    for line_number, line in enumerate(log.splitlines(), 1):
        match = re.match(r"\s*(?:Attempt\s+\d+:\s*)?(\w+)\s+(.*)", line)
        if not match:
            continue
        action, detail = ALIASES.get(match[1], match[1]), match[2]
        is_environment = bool(re.search(r"(?:frame|balloon|target) position", detail, re.I))
        # Values follow the final bracket/colon. Labels contain no numbers.
        numbers = [float(n) for n in re.findall(NUMBER, detail)]
        if not all(math.isfinite(n) for n in numbers):
            raise ValueError(f"Nonfinite log measurement on line {line_number}")
        if is_environment and len(numbers) >= 3:
            observations[action] = {
                "position_nwu": numbers[-3:],
                "source_line": line_number,
                "raw": line,
            }
        failed = bool(re.match(r"(?:FAILURE|failed)\b", detail, re.I))
        finished = bool(re.match(r"finished\b", detail, re.I))
        if not (failed or finished):
            continue
        pose = {}
        if not is_environment and len(numbers) == 4:
            pose = dict(zip(("x", "y", "z", "yaw"), numbers))
        # Do not invent a pose on FAILURE, or convert a detected object position
        # into the drone's own pose.
        captured = observations.pop(action, {})
        environment = captured if finished else {}
        result.append(
            ActionAnnotatedState(
                action=action,
                state_vector=pose,
                modality="information_capture" if environment or "Detect" in action else "motion",
                environment_observation=environment,
                failure_tag=detail if failed else None,
            )
        )
    return result
