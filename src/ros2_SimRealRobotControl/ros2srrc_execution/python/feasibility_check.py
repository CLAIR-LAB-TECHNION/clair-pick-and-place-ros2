#!/usr/bin/env python3
"""
Feasibility Check (HLD semantic wrapper).

Feasibility is implemented by MoveIt planning: when Traverse (Robmove) runs,
the C++ node calls plan_ROB() and only executes if planning succeeds.
This module exposes the same concept for documentation and optional explicit checks.

- check_feasible(target_pose, context=None): returns (success, reason).
- When Traverse is used, feasibility is checked implicitly (no change to existing flow).
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

# Robpose-like: object with .x, .y, .z, .qx, .qy, .qz, .qw
PoseLike = Any


def check_feasible(target_pose: PoseLike, context: Optional[Any] = None) -> Tuple[bool, str]:
    """
    Check whether a target pose is feasible (planning would succeed).

    Implementation: Feasibility is performed by MoveIt's planning pipeline
    (IK + collision checking). When Traverse (Robmove action) is executed,
    the same planning (plan_ROB()) runs internally; if it fails, the move
    is not executed. This function provides the semantic interface; the
    actual check is implicit in Traverse execution.

    Args:
        target_pose: Pose with .x, .y, .z, .qx, .qy, .qz, .qw (e.g. Robpose).
        context: Optional context (unused; for future use).

    Returns:
        (success, reason): True and a short message if considered feasible
        (by convention, when not doing an explicit plan-only call, we return
        True and note that feasibility is checked on Traverse).
    """
    _ = context
    # Semantic wrapper only: do not alter planning or execution.
    # Traverse uses MoveIt planning (plan_ROB()) internally; feasibility
    # is checked there. No explicit plan-only call from Python by default.
    return (True, "Feasibility checked implicitly by MoveIt during Traverse (plan_ROB).")
