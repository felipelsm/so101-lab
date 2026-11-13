import time
import mujoco
import mujoco.viewer
import numpy as np
from so101_mujoco_utils import set_initial_pose, send_position_command, move_to_pose, hold_position
from so101_forward_kinematics import get_forward_kinematics, BASE_HEIGHT

m = mujoco.MjModel.from_xml_path('model/scene.xml')
m.body('base').pos[2] = BASE_HEIGHT  # raise the robot to match the metal base
d = mujoco.MjData(m)

# --- Joint configurations (degrees, gripper 0-100) ---
starting_configuration = {
    'shoulder_pan': -45.0,
    'shoulder_lift': 45.0,
    'elbow_flex': -45.0,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 50
}
starting_configuration_closed = dict(starting_configuration, gripper=5)

# Middle configs: same pan as the pick/place spot, arm raised about 12 cm off the table
lifted_start_configuration = {
    'shoulder_pan': -45.0,
    'shoulder_lift': 10.0,
    'elbow_flex': -30.0,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 5
}
lifted_final_configuration = dict(lifted_start_configuration, shoulder_pan=45.0)

final_configuration_closed = {
    'shoulder_pan': 45.0,
    'shoulder_lift': 45.0,
    'elbow_flex': -45.0,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 5
}
final_configuration = dict(final_configuration_closed, gripper=50)


def show_cubes(viewer, starting_config, final_config, halfwidth=0.013):
    # Use forward kinematics
    starting_object_position, starting_object_orientation = get_forward_kinematics(starting_config)
    final_object_position, final_object_orientation = get_forward_kinematics(final_config)

    # Add starting cube
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[halfwidth, halfwidth, halfwidth],
        pos=starting_object_position,
        mat=starting_object_orientation.flatten(),
        rgba=[1, 0, 0, 0.2]
    )
    # Add final cube
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[1],
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[halfwidth, halfwidth, halfwidth],
        pos=final_object_position,
        mat=final_object_orientation.flatten(),
        rgba=[0, 1, 0, 0.2]
    )
    viewer.user_scn.ngeom = 2
    viewer.sync()
    return


# Print where the cubes should go (use these to place blocks on the acrylic grid)
for name, config in [('pick', starting_configuration), ('place', final_configuration)]:
    p, _ = get_forward_kinematics(config)
    print(f"{name} location (m): x = {p[0]:.4f}, y = {p[1]:.4f}, z = {p[2]:.4f}")

set_initial_pose(d, starting_configuration)
send_position_command(d, starting_configuration)

with mujoco.viewer.launch_passive(m, d) as viewer:
    show_cubes(viewer, starting_configuration, final_configuration)

    hold_position(m, d, viewer, 1.0)                                  # 1. open at pick spot
    move_to_pose(m, d, viewer, starting_configuration_closed, 1.0)    # 2. close gripper
    hold_position(m, d, viewer, 1.0)
    move_to_pose(m, d, viewer, lifted_start_configuration, 2.0)       # 3. lift up
    move_to_pose(m, d, viewer, lifted_final_configuration, 2.0)       # 4. swing over
    move_to_pose(m, d, viewer, final_configuration_closed, 2.0)       # 5. lower down
    hold_position(m, d, viewer, 1.0)
    move_to_pose(m, d, viewer, final_configuration, 1.0)              # 6. open gripper
    hold_position(m, d, viewer, 2.0)