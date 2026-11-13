import time
import numpy as np
import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, send_position_command, move_to_pose, hold_position
from so101_forward_kinematics import BASE_HEIGHT
from so101_inverse_kinematics import get_inverse_kinematics, get_gripper_target, get_wrist_flex_position

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
  
  # Specify the position of the center of the cube to be picked up
  block_position = [0.2, 0.2, 0.014]

  # Add a cube as a site for visualization
  show_cube(viewer, block_position, np.eye(3))
  
  # Offset the target so that the jaws surround the cube
  desired_position = get_gripper_target(block_position)
  
  # First send the robot to a higher position with the gripper open
  joint_configuration = get_inverse_kinematics(desired_position)
  move_to_pose(m, d, viewer, joint_configuration, 1.0)
  
  # Hold position for 10 seconds
  hold_position(m, d, viewer, 10.0)
