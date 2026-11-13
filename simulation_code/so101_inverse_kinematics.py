import numpy as np

from so101_forward_kinematics import make_pose, get_gw1, get_g12, get_g45, get_g5t

def get_wrist_flex_position(target_position, target_orientation=np.eye(3)):
    "Where the wrist flex frame (frame 4) has to be so the tool frame lands on the target"
    # Desired tool pose in the world, as a 4x4
    gwt = make_pose(target_orientation, target_position)
    g4t = get_g45(0) @ get_g5t()
    gw4 = gwt @ np.linalg.inv(g4t)
    wrist_flex_position = gw4[0:3, 3]
    wrist_flex_orientation = gw4[0:3, 0:3]
    return wrist_flex_position, wrist_flex_orientation

def get_inverse_kinematics(target_position, target_orientation=np.eye(3)):
    "Geometric appraoch specific to the so-101 arms"

    x, y, z = target_position

    # --- theta1 (shoulder_pan): point the arm at the target ---
    # The pan axis is 0.0388353 m in front of the world origin (along world x)
    x_from_pan = x - 0.0388353
    # Frame 1's z-axis points DOWN, so positive theta1 turns the arm clockwise (seen from above)
    theta1 = -np.degrees(np.arctan2(y, x_from_pan))

    target_wrist_position, target_wrist_orientation = get_wrist_flex_position(target_position)
    # --- theta2 and theta3: law of cosines on the triangle joint 2 / joint 3 / wrist ---
    # Joint 2 position straight from the FK (so the metal base is included)
    gw2 = get_gw1(theta1) @ get_g12(0)
    joint2_position = gw2[0:3, 3]

    # Work in the vertical plane of the arm:
    # r = horizontal distance from the pan axis, h = height above the table
    def to_plane(p):
        r = np.sqrt((p[0] - 0.0388353)**2 + p[1]**2)
        return r, p[2]
    r2, h2 = to_plane(joint2_position)
    r4, h4 = to_plane(target_wrist_position)

    # Link lengths from the FK diagram
    L1 = np.sqrt(0.11257**2 + 0.028**2)              # joint 2 -> joint 3, straight line
    L2 = 0.1349                                      # joint 3 -> joint 4
    beta = np.degrees(np.arctan2(0.11257, 0.028))    # angle of the L1 line above horizontal when theta2 = 0

    # Third side of the triangle: joint 2 -> wrist
    D = np.sqrt((r4 - r2)**2 + (h4 - h2)**2)
    phi = np.degrees(np.arctan2(h4 - h2, r4 - r2)) # angle of D above horizontal

    # Law of cosines: angle at joint 2 between D and the upper arm (NaN if the triangle can't close)
    psi = np.degrees(np.arccos((L1**2 + D**2 - L2**2) / (2 * L1 * D)))

    # Elbow-up: the upper arm sits psi ABOVE the D line
    upper_arm_angle = phi + psi
    theta2 = beta - upper_arm_angle

    r3 = r2 + L1 * np.cos(np.radians(upper_arm_angle))
    h3 = h2 + L1 * np.sin(np.radians(upper_arm_angle))
    c = np.degrees(np.arctan2(h4 - h3, r4 - r3)) # lower arm angle above horizontal
    theta3 = -c - theta2

    # --- theta4: point the gripper straight down ---
    theta4 = 90.0 + c

    # --- theta5: square the jaws up with the block (tool x-axis = world x-axis) ---
    theta5 = theta1

    # Initialize the joint configuration dictionary
    joint_config = {
        'shoulder_pan': theta1,
        'shoulder_lift': theta2,
        'elbow_flex': theta3,
        'wrist_flex': theta4,
        'wrist_roll': theta5,
        'gripper': 0.0
    }

    return joint_config

BLOCK_WIDTH = 0.0285  # meters (1.125 inches)
FIXED_JAW_OFFSET = 0.0079  # meters, from the wrist roll axis to the inner face of the fixed jaw

def get_gripper_target(block_position):
    "Offset from the center of the block so that the jaws surround the block"
    gripper_target = list(block_position)
    gripper_target[0] -= BLOCK_WIDTH / 2 - FIXED_JAW_OFFSET
    gripper_target[2] -= BLOCK_WIDTH / 2
    return gripper_target

