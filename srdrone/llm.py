"""Small injectable text-model interface; no network calls at import time."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Protocol
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen


class ModelError(RuntimeError):
    pass


class TextModel(Protocol):
    def complete(self, stage: str, system: str, payload: dict) -> str: ...


def json_object(text: str) -> dict:
    """Accept JSON or a single fenced JSON block, never eval model output."""
    cleaned = text.strip()
    if cleaned.startswith("```"):
        lines = cleaned.splitlines()
        if len(lines) < 3 or lines[-1].strip() != "```":
            raise ModelError("Unclosed JSON fence")
        cleaned = "\n".join(lines[1:-1])
    try:
        value = json.loads(cleaned)
    except (ValueError, TypeError) as exc:
        raise ModelError("Model output is not a JSON object") from exc
    if not isinstance(value, dict):
        raise ModelError("Model output must be a JSON object")
    return value


def require_text(data: dict, key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ModelError(f"Expected nonempty string: {key}")
    return value


class ChatModel:
    """Chat Completions compatible transport, with bounded retries and audit logs.

    SRDRONE_MODEL is explicit: historical GPT-4o availability is provider-dependent.
    Payloads contain mission logs and prompts; credentials are never traced.
    """

    def __init__(
        self,
        model: str,
        api_key: str,
        base_url: str,
        temperature: float = 0.2,
        timeout: float = 90,
        retries: int = 2,
        trace_path: str | None = None,
    ) -> None:
        if not model or not api_key:
            raise ModelError("Set SRDRONE_MODEL and SRDRONE_API_KEY")
        parsed = urlparse(base_url)
        if parsed.scheme != "https" and not (
            parsed.scheme == "http" and parsed.hostname in {"localhost", "127.0.0.1", "::1"}
        ):
            raise ModelError("Use HTTPS or a local HTTP endpoint")
        if parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ModelError("Base URL must not contain credentials, query or fragment")
        if timeout <= 0 or retries < 0 or not 0 <= temperature <= 2:
            raise ModelError("Invalid timeout, retry count or temperature")
        self.model, self._key = model, api_key
        self.url = base_url.rstrip("/") + "/chat/completions"
        self.temperature, self.timeout, self.retries = temperature, timeout, retries
        self.trace_path = Path(trace_path) if trace_path else None

    @classmethod
    def from_env(cls, trace_path: str | None = None) -> ChatModel:
        return cls(
            os.getenv("SRDRONE_MODEL", ""),
            os.getenv("SRDRONE_API_KEY", ""),
            os.getenv("SRDRONE_BASE_URL", "https://api.openai.com/v1"),
            float(os.getenv("SRDRONE_TEMPERATURE", "0.2")),
            float(os.getenv("SRDRONE_TIMEOUT", "90")),
            trace_path=trace_path,
        )

    def complete(self, stage: str, system: str, payload: dict) -> str:
        messages = [
            {"role": "system", "content": system},
            {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
        ]
        body = json.dumps(
            {"model": self.model, "temperature": self.temperature, "messages": messages}
        ).encode()
        for attempt in range(self.retries + 1):
            request = Request(
                self.url,
                data=body,
                headers={
                    "Authorization": "Bearer " + self._key,
                    "Content-Type": "application/json",
                },
            )
            try:
                with urlopen(request, timeout=self.timeout) as response:
                    result = json.load(response)
                choice = result["choices"][0]
                if choice.get("finish_reason") not in {None, "stop"}:
                    raise ModelError(f"Incomplete model response at {stage}")
                text = choice["message"]["content"]
                if not isinstance(text, str) or not text.strip():
                    raise ModelError(f"Empty model response at {stage}")
                if self.trace_path:
                    self.trace_path.parent.mkdir(parents=True, exist_ok=True)
                    with self.trace_path.open("a", encoding="utf-8") as file:
                        file.write(
                            json.dumps(
                                {
                                    "stage": stage,
                                    "model": self.model,
                                    "messages": messages,
                                    "response": text,
                                    "usage": result.get("usage", {}),
                                },
                                ensure_ascii=False,
                            )
                            + "\n"
                        )
                return text
            except HTTPError as exc:
                retryable = exc.code in {408, 429, 500, 502, 503, 504}
                if not retryable or attempt == self.retries:
                    raise ModelError(f"Model HTTP {exc.code} at {stage}") from None
            except (URLError, TimeoutError):
                if attempt == self.retries:
                    raise ModelError(f"Model connection failed at {stage}") from None
            except (KeyError, IndexError, TypeError, ValueError):
                raise ModelError(f"Malformed provider response at {stage}") from None
            time.sleep(min(2**attempt, 4))
        raise ModelError("Retries exhausted")


class RecordedModel:
    """Explicit fixture transport for demos/tests; never claims fresh inference."""

    def __init__(self, responses: list[dict]) -> None:
        self.responses = list(responses)
        self.calls: list[dict[str, Any]] = []

    def complete(self, stage: str, system: str, payload: dict) -> str:
        self.calls.append({"stage": stage, "system": system, "payload": payload})
        if not self.responses:
            raise ModelError(f"No recorded response for {stage}")
        item = self.responses.pop(0)
        if item["stage"] != stage:
            raise ModelError(f"Expected recorded stage {item['stage']}, got {stage}")
        response = item["response"]
        return response if isinstance(response, str) else json.dumps(response)
