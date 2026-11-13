import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors
from so101_inverse_kinematics import get_inverse_kinematics
import sys

from so101_utils import pick_up_block, place_block

# CONFIGURATION VARIABLES
PORT_ID = "/dev/ttyACM0" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follow0"

# Set 
'''
This is the format of the goal position dictionary used for goal position sync write.
The gripper command takes values of 0-100, while the other joints take values of degrees, based on
the settings specified in the bus initialization.
'''
desired_ee = [0.2, 0.0, 0.03]  # x, y, z in meters
desired_configuration = get_inverse_kinematics(desired_ee)

block_position_1 = [0.25, 0.10, 0.015]   # where the block starts
block_position_2 = [0.15, 0.15, 0.015]   # where the block starts

stack_xy = [0.25, -0.10]                               # where the block should end up
stack_1 = [stack_xy[0], stack_xy[1], 0.015]            # bottom block sits on the table
stack_2 = [stack_xy[0], stack_xy[1], 0.015 + 0.0285]   # top block sits one block higher

move_time = 2.0  # seconds to reach desired position
hold_time = 1.0  # total time to hold at 

home_configuration = {
    'shoulder_pan': 0.0,   # degrees
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 50.0           # 0-100 range
}

# ------------------------
# Configure motors and load calibration:
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)

# Record starting position
starting_pose = bus.sync_read("Present_Position")
print("Starting Pose:", starting_pose)

hold_position(bus, hold_time)
move_to_pose(bus, home_configuration, move_time)
hold_position(bus, hold_time)

# Move to desired
pick_up_block(bus, block_position_1, move_time)
# Hold position for "hold_time" seconds
hold_position(bus, hold_time)
# Move back to starting position
place_block(bus, stack_1, move_time)

hold_position(bus, hold_time)
move_to_pose(bus, home_configuration, move_time)
hold_position(bus, hold_time)

# Move to desired
pick_up_block(bus, block_position_2, move_time)
# Hold position for "hold_time" seconds
hold_position(bus, hold_time)
# Move back to starting position
place_block(bus, stack_2, move_time)

hold_position(bus, hold_time)
move_to_pose(bus, home_configuration, move_time)
hold_position(bus, hold_time)

# Before terminating the program, disable torque on the motors
bus.disable_torque()
