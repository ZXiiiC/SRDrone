"""Durable task-unit experience repository with explicit validation provenance."""

from __future__ import annotations

import hashlib
import json
import math
import re
import sqlite3
from pathlib import Path


class ExperienceBank:
    """SQLite makes inserts/status transitions atomic; identical entries deduplicate.

    proposed: generated from a failure, not yet validated by a new execution.
    execution_validated: the associated candidate succeeded in the same run.
    This is a traceability status, not an independent scientific validity claim.
    """

    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as connection:
            connection.execute("""CREATE TABLE IF NOT EXISTS experiences (
                id TEXT PRIMARY KEY, task TEXT NOT NULL, category TEXT NOT NULL,
                source_plan TEXT NOT NULL, candidate_plan TEXT NOT NULL,
                run_id TEXT NOT NULL, content TEXT NOT NULL,
                status TEXT NOT NULL DEFAULT 'proposed')""")

    def _connect(self):
        return sqlite3.connect(str(self.path), timeout=30)

    def add(
        self,
        task: str,
        category: str,
        source_plan: str,
        candidate_plan: str,
        run_id: str,
        experiences: list[dict],
    ) -> list[str]:
        identifiers = []
        with self._connect() as connection:
            for entry in experiences:
                content = json.dumps(entry, ensure_ascii=False, sort_keys=True)
                identity = json.dumps(
                    [task, category, source_plan, candidate_plan, run_id, content]
                )
                identifier = hashlib.sha256(identity.encode()).hexdigest()
                connection.execute(
                    "INSERT OR IGNORE INTO experiences "
                    "(id,task,category,source_plan,candidate_plan,run_id,content) VALUES (?,?,?,?,?,?,?)",
                    (identifier, task, category, source_plan, candidate_plan, run_id, content),
                )
                identifiers.append(identifier)
        return identifiers

    def validate_candidate(self, candidate_plan: str, run_id: str) -> None:
        with self._connect() as connection:
            connection.execute(
                "UPDATE experiences SET status='execution_validated' "
                "WHERE candidate_plan=? AND run_id=?",
                (candidate_plan, run_id),
            )

    def records(self, *, include_proposed: bool = False) -> list[dict]:
        with self._connect() as connection:
            connection.row_factory = sqlite3.Row
            rows = connection.execute("SELECT * FROM experiences ORDER BY rowid").fetchall()
        return [
            {**dict(row), "experience": json.loads(row["content"])}
            for row in rows
            if include_proposed or row["status"] == "execution_validated"
        ]

    def retrieve(
        self, task: str, k: int = 3, *, backend: str = "mpnet", category: str | None = None
    ) -> list[dict]:
        if k < 0:
            raise ValueError("k must be nonnegative")
        rows = self.records()
        if category:
            rows = [r for r in rows if r["category"] == category]
        if not rows or not k:
            return []
        texts = [
            r["task"] + " " + r["experience"]["operation"] + " " + r["experience"]["rationale"]
            for r in rows
        ]
        if backend == "mpnet":
            try:
                from sentence_transformers import SentenceTransformer
            except ImportError as exc:
                raise RuntimeError(
                    "Install .[retrieval] for paper-style MPNet retrieval; "
                    "use --retriever lexical only for smoke tests"
                ) from exc
            model = SentenceTransformer("sentence-transformers/all-mpnet-base-v2")
            embeddings = model.encode([task] + texts, normalize_embeddings=True)
            scores = [float(embeddings[0] @ item) for item in embeddings[1:]]
        elif backend == "lexical":
            # Explicit engineering fallback, not the paper's embedding method.
            def vector(text):
                words = re.findall(r"\w+", text.lower())
                return {word: words.count(word) for word in set(words)}

            query = vector(task)

            def cosine(text):
                item = vector(text)
                denominator = math.sqrt(
                    sum(v * v for v in query.values()) * sum(v * v for v in item.values())
                )
                return (
                    sum(v * item.get(w, 0) for w, v in query.items()) / denominator
                    if denominator
                    else 0.0
                )

            scores = [cosine(text) for text in texts]
        else:
            raise ValueError("Unknown retrieval backend")
        ranked = sorted(zip(scores, rows), key=lambda pair: (-pair[0], pair[1]["id"]))
        selected, seen = [], set()
        for score, row in ranked:
            normalized = json.dumps(row["experience"], sort_keys=True).lower()
            if normalized in seen:
                continue
            seen.add(normalized)
            selected.append({**row, "score": score, "retriever": backend})
            if len(selected) == k:
                break
        return selected
