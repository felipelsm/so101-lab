# Part 2a: which block positions can the arm reach with a vertical grasp?
import numpy as np
import matplotlib
matplotlib.use("Agg")  # save to a PNG instead of opening a window
import matplotlib.pyplot as plt
from so101_inverse_kinematics import get_inverse_kinematics, get_gripper_target

z = 0.014  # height of the middle of a block sitting on the table

# Grid of block positions to try
x_values = np.linspace(0.0, 0.30, 61)    # every 5 mm from 0 to 0.30 m
y_values = np.linspace(-0.30, 0.30, 121)  # every 5 mm from -0.30 to 0.30 m

reachable = []
unreachable = []

for x in x_values:
    for y in y_values:
        block_position = [x, y, z]
        target = get_gripper_target(block_position)   # same offset we use when grabbing a block

        with np.errstate(invalid='ignore'):           # hide the arccos "invalid value" warning
            joint_config = get_inverse_kinematics(target)

        if np.isnan(joint_config['shoulder_lift']):   # NaN = the triangle can't close = unreachable
            unreachable.append([x, y])
        else:
            reachable.append([x, y])

reachable = np.array(reachable)
unreachable = np.array(unreachable)

plt.figure(figsize=(7, 5))
plt.scatter(unreachable[:, 0], unreachable[:, 1], s=3, c='red', label='unreachable')
plt.scatter(reachable[:, 0], reachable[:, 1], s=3, c='green', label='reachable')
plt.title(f"End-Effector Workspace for Block at z = {z} m")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.legend(loc='upper right')
plt.savefig("workspace_z0.014.png", dpi=120)
print("saved workspace_z0.014.png")

print(f"reachable x: {reachable[:, 0].min():.3f} to {reachable[:, 0].max():.3f} m")
print(f"reachable y: {reachable[:, 1].min():.3f} to {reachable[:, 1].max():.3f} m")