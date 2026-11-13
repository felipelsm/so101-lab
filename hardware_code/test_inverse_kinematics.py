import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors
from so101_inverse_kinematics import get_inverse_kinematics

# CONFIGURATION VARIABLES
PORT_ID = "/dev/tty.usbmodem5A460848371" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follower-1"

# Set 
'''
This is the format of the goal position dictionary used for goal position sync write.
The gripper command takes values of 0-100, while the other joints take values of degrees, based on
the settings specified in the bus initialization.
'''
desired_ee = [0.2, 0.0, 0.03]  # x, y, z in meters
desired_configuration = get_inverse_kinematics(desired_ee)
move_time = 2.0  # seconds to reach desired position
hold_time = 10.0  # total time to hold at 

# ------------------------
# Configure motors and load calibration:
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)

# Record starting position
starting_pose = bus.sync_read("Present_Position")
print("Starting Pose:", starting_pose)

# Move to desired
move_to_pose(bus, desired_configuration, move_time)

# Hold position for "hold_time" seconds
hold_position(bus, hold_time)

# Move back to starting position
move_to_pose(bus, starting_pose, move_time)

# Before terminating the program, disable torque on the motors
bus.disable_torque()
