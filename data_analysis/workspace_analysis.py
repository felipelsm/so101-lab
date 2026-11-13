# Part 4b: workspace at each stacking height, checking NaN AND joint limits
import numpy as np
import matplotlib
matplotlib.use("Agg")  # save to PNG files instead of opening windows
import matplotlib.pyplot as plt

from pathlib import Path
import sys
SIM_CODE = Path(__file__).resolve().parents[1] / 'simulation_code'
sys.path.insert(0, str(SIM_CODE))
from so101_inverse_kinematics import get_inverse_kinematics, get_gripper_target

# Joint limits in degrees (from the robot model)
JOINT_LIMITS = {
    'shoulder_pan': (-109.95, 109.95),
    'shoulder_lift': (-99.69, 99.69),
    'elbow_flex': (-96.83, 96.83),
    'wrist_flex': (-94.54, 94.54),
    'wrist_roll': (-157.0, 162.7),
}

def is_reachable(block_position):
    target = get_gripper_target(block_position)
    with np.errstate(invalid='ignore'):           # hide the arccos warning
        config = get_inverse_kinematics(target)
    for joint, (low, high) in JOINT_LIMITS.items():
        angle = config[joint]
        if np.isnan(angle):                       # too far away: triangle can't close
            return False
        if angle < low or angle > high:           # math works, but the joint can't bend that far
            return False
    return True

heights = [0.014, 0.043, 0.071, 0.100]  # middle of the 1st, 2nd, 3rd, 4th block in a stack
x_values = np.linspace(0.0, 0.30, 61)
y_values = np.linspace(-0.30, 0.30, 61)

for z in heights:
    reachable, unreachable = [], []
    for x in x_values:
        for y in y_values:
            if is_reachable([x, y, z]):
                reachable.append([x, y])
            else:
                unreachable.append([x, y])
    reachable, unreachable = np.array(reachable), np.array(unreachable)

    plt.figure(figsize=(6.4, 4.8))
    plt.scatter(unreachable[:, 0], unreachable[:, 1], s=4, c='red', marker='s')
    if len(reachable):
        plt.scatter(reachable[:, 0], reachable[:, 1], s=4, c='green', marker='s')
    plt.title(f"End-Effector Workspace for Block Placed at Z = {z:.3f} m")
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.savefig(f"so101_workspace_z{z:.3f}.png", dpi=100)
    plt.close()

    if len(reachable):
        print(f"z = {z:.3f}: x {reachable[:, 0].min():.3f} to {reachable[:, 0].max():.3f}, "
              f"y {reachable[:, 1].min():.3f} to {reachable[:, 1].max():.3f}")
    else:
        print(f"z = {z:.3f}: nothing reachable")