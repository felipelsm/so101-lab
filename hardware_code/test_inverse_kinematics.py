import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors
from so101_inverse_kinematics import get_inverse_kinematics
import sys

print(sys.argv)
# python run.py 5 hello   ->   ['run.py', '5', 'hello']

arg = sys.argv[1]

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
if arg == "ik":
    desired_configuration = get_inverse_kinematics(desired_ee)
elif arg == "home":
    desired_configuration = {
    'shoulder_pan': 0.0,   # degrees
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 50.0           # 0-100 range
}
else:
    sys.exit(f"Unknown mode '{arg}'. Use 'ik' or 'home'.")

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
