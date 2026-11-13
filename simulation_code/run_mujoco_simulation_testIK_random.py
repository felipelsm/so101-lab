import time
import numpy as np
import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, send_position_command, move_to_pose, hold_position
from so101_forward_kinematics import BASE_HEIGHT
from so101_inverse_kinematics import get_inverse_kinematics, get_gripper_target

m = mujoco.MjModel.from_xml_path('model/scene.xml')
m.body('base').pos[2] = BASE_HEIGHT  # raise the robot to match the metal base
d = mujoco.MjData(m)

# Helper Function to show a cube at a given position and orientation
def show_cube(viewer, position, orientation, geom_num=0, halfwidth=0.01425):
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[geom_num],
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[halfwidth, halfwidth, halfwidth],
        pos=position,
        mat=orientation.flatten(),
        rgba=[1, 0, 0, 0.2]
    )
    viewer.user_scn.ngeom = 1
    viewer.sync()
    return

# Helper function to obtain a random target position from a given range
def get_random_position():
    x_pos_range = [0.13, 0.25]   # taken from workspace analysis
    y_pos_range = [-0.18, 0.18]  # taken from workspace analysis
    x = np.random.uniform(x_pos_range[0], x_pos_range[1])
    y = np.random.uniform(y_pos_range[0], y_pos_range[1])
    return [x, y, 0.014]

# Initial joint configuration at start of simulation
initial_config = {
    'shoulder_pan': 0.0,
    'shoulder_lift': 0.0,
    'elbow_flex': 0.00,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0
}
set_initial_pose(d, initial_config)
send_position_command(d, initial_config)

# Start simulation with mujoco viewer
with mujoco.viewer.launch_passive(m, d) as viewer:

    # Pause for 10 seconds in order to make screen recording easier
    #time.sleep(10)

    while viewer.is_running():

        for i in range(5):
            block_position = get_random_position()
            desired_orientation = np.eye(3)

            # Show the block
            show_cube(viewer, block_position, desired_orientation)

            # Offset the target so that the jaws surround the cube
            desired_position = get_gripper_target(block_position)

            # Get the inverse kinematics solution
            joint_configuration = get_inverse_kinematics(desired_position, desired_orientation)
            joint_configuration['gripper'] = 50  # open the jaws so they fit around the block
            print(f"block at ({block_position[0]:.3f}, {block_position[1]:.3f}) ->",
                  {j: round(float(v), 1) for j, v in joint_configuration.items()})

            # Move the robot to the desired pose
            move_to_pose(m, d, viewer, joint_configuration, 1.0)

            # Hold for two seconds for easy visualization
            hold_position(m, d, viewer, 2.0)