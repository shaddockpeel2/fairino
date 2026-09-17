"""Thread-safe values shared by the service, clients, and future GUI."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class AgentEvent:
    kind: str
    state: str
    message: str = ''
    request_id: str = ''
    elapsed_s: Optional[float] = None


@dataclass(frozen=True)
class ChatResult:
    request_id: str
    text: str
    first_token_s: Optional[float]
    total_s: float
