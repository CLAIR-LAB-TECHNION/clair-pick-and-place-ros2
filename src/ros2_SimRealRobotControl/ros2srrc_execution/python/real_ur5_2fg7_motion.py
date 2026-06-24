"""
UR5 + OnRobot 2FG7 real-robot pick/place Z calibration (shared by Hanoi, pick.py, place.py).

Convention (matches hanoi_tower_demo.py):
  - pick_ref Z: stand collision top (0.84 m) + heights of cubes below the target (not board, not center)
  - cube center Z: geometric center used by SpawnObjectMoveIt / MoveIt scene
  - MoveIt stops at pick_ref + offset; physical fingers reach cube via grasp_extra_descend_m
"""

REAL_PEG_Z = 0.84
REAL_EE_TO_FINGER_Z = 0.055
REAL_GRASP_Z_OFFSET_LOW = 0.01
REAL_MIN_MOVEIT_GRASP_Z = 0.87
REAL_DEFAULT_BOARD_HEIGHT_M = 0.02
# If object_pose.z is above stand top by more than this, treat Z as cube center (MoveIt spawn).
CENTER_Z_THRESHOLD_ABOVE_PEG_M = 0.015


def compute_real_ee_motion_params(z_pick_ref, z_center):
    """
    Return (moveit_z_offset, extra_descend_m, z_ee_target) for pick.py / place.py.

    moveit_z_offset is relative to pick_ref Z (object_pose.z for pick, place ref for place).
  extra_descend_m is physical MoveL down after MoveIt reach pose.
    """
    z_ee_target = z_center - REAL_EE_TO_FINGER_Z
    z_moveit_z = max(z_ee_target + REAL_GRASP_Z_OFFSET_LOW, REAL_MIN_MOVEIT_GRASP_Z)
    moveit_z_offset = round(z_moveit_z - z_pick_ref, 4)
    extra_descend = round(max(0.0, z_moveit_z - z_ee_target), 4)
    return moveit_z_offset, extra_descend, z_ee_target


def infer_pick_ref_and_center(z, cube_size, board_height=None, z_is_pick_ref=None):
    """
    Convert a single Z value to (pick_ref, center) for a bottom cube on the stand.

    Args:
        z: Z from MoveIt scene (center) or explicit pick_ref (0.84 for bottom cube).
        cube_size: cube edge length (m).
        board_height: board on stand (m); default REAL_DEFAULT_BOARD_HEIGHT_M.
        z_is_pick_ref: True/False to force interpretation; None = auto-detect.
    """
    board = REAL_DEFAULT_BOARD_HEIGHT_M if board_height is None else float(board_height)
    half = float(cube_size) / 2.0
    z = float(z)

    if z_is_pick_ref is True:
        z_pick_ref = z
        z_center = z + board + half
    elif z_is_pick_ref is False:
        z_center = z
        z_pick_ref = z - board - half
    elif z > REAL_PEG_Z + CENTER_Z_THRESHOLD_ABOVE_PEG_M:
        z_center = z
        z_pick_ref = z - board - half
    else:
        z_pick_ref = z
        z_center = z + board + half
    return round(z_pick_ref, 4), round(z_center, 4)
