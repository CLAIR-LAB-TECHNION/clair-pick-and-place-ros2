#!/usr/bin/env python3
"""
Set Configuration (HLD): optional program step for system-level planning
or execution parameters (e.g. planner ID, planning time, named joint preset).
Does not override Pick/Place YAML configs. If never invoked, defaults to
current behavior (no effect).
"""

from __future__ import annotations

from typing import Any, Dict

# Global configuration store (opt-in only; existing code does not read this).
_global_config: Dict[str, Any] = {}


def set_configuration(step_dict: dict) -> dict:
    """
    Apply a SetConfiguration step: store global planning/execution parameters.
    Supported keys (all optional): planner_id, planning_time_sec, max_velocity_scaling,
    joint_preset (dict of joint names to values for future use).
    Does NOT change C++ or MoveIt parameters from Python; storage only for
    documentation and future use. Existing behavior unchanged if never called.

    Args:
        step_dict: Step from YAML, e.g. {"Type": "SetConfiguration", "Input": {...}}.

    Returns:
        {"Success": bool, "Message": str}
    """
    global _global_config
    inp = step_dict.get("Input") or step_dict.get("input") or {}
    if not inp:
        return {"Success": True, "Message": "SetConfiguration: no input (no-op)."}

    if "planner_id" in inp:
        _global_config["planner_id"] = str(inp["planner_id"])
    if "planning_time_sec" in inp:
        _global_config["planning_time_sec"] = float(inp["planning_time_sec"])
    if "max_velocity_scaling" in inp:
        _global_config["max_velocity_scaling"] = float(inp["max_velocity_scaling"])
    if "joint_preset" in inp:
        _global_config["joint_preset"] = dict(inp["joint_preset"])

    return {"Success": True, "Message": "SetConfiguration: stored (no runtime effect)."}


def get_configuration() -> dict:
    """Return current global configuration (for documentation/future use)."""
    return dict(_global_config)
