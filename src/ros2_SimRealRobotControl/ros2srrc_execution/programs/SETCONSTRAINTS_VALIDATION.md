# SetConstraints validation: baseline → blocker → baseline

Use these three runs to confirm (1) unchanged behavior without SetConstraints, (2) planning fails or detours when a big collision box blocks the path, (3) removing the step restores behavior.

**Prerequisites:** Simulation or real robot + MoveIt 2 running (e.g. `ros2 launch ros2srrc_launch simulation.launch.py` or bringup), and `source install/setup.bash`.

---

## If "Planning failed" on Step 1 (MoveJ)

Planning happens in the **move** and **robmove** nodes; they need a **valid robot state** from the simulation or real robot. If you see `MoveJ:FAILED. Reason -> Planning failed.`:

1. **Start the full stack first** (in another terminal):
   - **Simulation:**  
     `ros2 launch ros2srrc_launch simulation.launch.py`  
     (or your usual sim launch that starts Gazebo + MoveIt and the move/robmove nodes.)
   - **Real robot:**  
     Your bringup launch that starts the driver + MoveIt + move/robmove.

2. **Confirm ur5_demo works** (same launch, no other changes):
   ```bash
   ros2 run ros2srrc_execution ExecuteProgram.py package:=ros2srrc_execution program:=ur5_demo
   ```
   If **Step 1 of ur5_demo also fails**, the problem is the launch/environment (no robot state, wrong config, or move_group not ready). Fix that before using the SetConstraints programs.

3. **Try the no-EE baseline** (same sequence, no gripper loaded):
   ```bash
   ros2 run ros2srrc_execution ExecuteProgram.py package:=ros2srrc_execution program:=ur5_setconstraints_baseline_no_ee
   ```
   If this **succeeds** but `ur5_setconstraints_baseline` (with ParallelGripper) fails, the difference is loading the end-effector client.

---

## 1. Run existing YAML unchanged → confirm identical behavior

Run the **baseline** program (no SetConstraints): same specs as a minimal pick-and-place style run — home, one Traverse, return home.

```bash
ros2 run ros2srrc_execution ExecuteProgram.py package:=ros2srrc_execution program:=ur5_setconstraints_baseline
```

**Expected:** All steps succeed (MoveJ home → RobMove to pose in front → MoveJ home). No errors.

---

## 2. Add SetConstraints (big box) + Traverse through it → confirm planning fails or detours

Run the **blocker** program: adds a big collision box in front of the robot at the same pose as the Traverse goal, then attempts that Traverse.

```bash
ros2 run ros2srrc_execution ExecuteProgram.py package:=ros2srrc_execution program:=ur5_setconstraints_with_blocker
```

**Expected:** Step 1 (SetConstraints) succeeds. Step 2 (MoveJ home) succeeds. Step 3 (RobMove to 0.15, 0.48, 0.84) should **fail** with a planning error (e.g. `GOAL_IN_COLLISION` or `PLANNING_FAILED`) because the goal is inside the box. The program then exits with "Execution FAILED!". If the planner sometimes finds a detour, you might see success instead; in that case the box is not fully blocking the goal.

---

## 3. Remove the SetConstraints step → confirm behavior like before

Run the **baseline** program again (no SetConstraints step).

```bash
ros2 run ros2srrc_execution ExecuteProgram.py package:=ros2srrc_execution program:=ur5_setconstraints_baseline
```

**Expected:** Same as step 1: all steps succeed. Behavior is identical to the first run.

---

## Files

| File | Purpose |
|------|--------|
| `ur5_setconstraints_baseline.yaml` | Home → Traverse to (0.15, 0.48, 0.865) → home. No constraints. Uses ParallelGripper. |
| `ur5_setconstraints_baseline_no_ee.yaml` | Same sequence as baseline but `EndEffector: None` (like ur5_demo). Use if baseline fails to isolate EE vs launch. |
| `ur5_setconstraints_with_blocker.yaml` | SetConstraints (big box at goal) → home → Traverse to same pose (blocked) → home. |

The blocker box is 0.5×0.5×0.4 m at (0.15, 0.48, 0.84), so the RobMove goal lies inside it and planning should fail.
