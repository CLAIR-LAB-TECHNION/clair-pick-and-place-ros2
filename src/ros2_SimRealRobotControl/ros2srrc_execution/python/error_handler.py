#!/usr/bin/env python3
"""
Centralized Error Handling (HLD): thin layer that receives result objects
from Pick / Place / Traverse and maps them to a single outcome. No retries,
no recovery logic, no new behavior — only centralization. Replaces direct
exit() by forwarding the same decision.
"""

from __future__ import annotations

from typing import Any, Dict

# Outcome constants for HLD mapping
SUCCESS = "SUCCESS"
FAILURE = "FAILURE"
ABORT = "ABORT"


def interpret(result: Dict[str, Any] | None) -> str:
    """
    Map a result dict (from Pick, Place, Traverse, or other steps) to
    {SUCCESS, FAILURE, ABORT}. No side effects.

    Args:
        result: Dict with at least "Success" (bool); "Message" optional.

    Returns:
        SUCCESS, FAILURE, or ABORT.
    """
    if result is None:
        return ABORT
    if isinstance(result.get("Success"), bool):
        return SUCCESS if result["Success"] else FAILURE
    return ABORT


def handle_step_result(result: Dict[str, Any] | None, step_name: str = "") -> None:
    """
    Centralized handling after a step: interpret result and exit on failure
    (same behavior as previous direct exit() calls). No retries or recovery.

    Args:
        result: Result dict from the step (e.g. RES from Pick/Place/RobMove).
        step_name: Optional step name for logging.
    """
    outcome = interpret(result)
    if outcome == SUCCESS:
        return
    # Forward the same decision: exit on failure
    msg = (result or {}).get("Message", "Unknown error")
    print("ERROR: Execution FAILED!")
    if step_name:
        print(f"Step: {step_name}")
    print("Message -> " + str(msg))
    print("")
    print("Closing... BYE!")
    exit()
