from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

from .models import ExperienceRecord


class ExperienceStore:
    """Append-only JSONL store for workflow experience records."""

    def __init__(self, file_path: str) -> None:
        self._path = Path(file_path)
        self._path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, record: ExperienceRecord) -> None:
        with self._path.open("a", encoding="utf-8") as file:
            file.write(
                json.dumps(asdict(record), ensure_ascii=False, default=str) + "\n"
            )
