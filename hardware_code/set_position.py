import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors

# CONFIGURATION VARIABLES
PORT_ID = "/dev/ttyACM0" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follow0" # REPLACE WITH YOUR ROBOT NAME! 

# --- Specified Parameters ---
'''
This is the format of the goal position dictionary used for goal position sync write.
The gripper command takes values of 0-100, while the other joints take values of degrees, based on
the settings specified in the bus initialization.
'''
desired_position = {
    'shoulder_pan': 0.0,   # degrees
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 50.0           # 0-100 range
}
move_time = 5.0  # seconds to reach desired position
hold_time = 5.0  # total time to hold at 

# ------------------------

# Setup calibration and motor bus
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)

print("Reading initial pose...")
starting_position = bus.sync_read("Present_Position")
print(" >>>" + str(starting_position))

# # CRITICAL FIX: Tell the motors to stay exactly where they are
# bus.sync_write("Goal_Position", starting_position, normalize=True)
# #bus.enable_torque()

print(f"Holding initial pose for {hold_time} seconds...")
hold_position(bus, hold_time)

print(f"Moving to target pose over {move_time} seconds...")
move_to_pose(bus, desired_position, move_time)
end_position = bus.sync_read("Present_Position")
print(" >>>" + str(end_position))

print(f"Holding target pose for {hold_time} seconds...")
hold_position(bus, hold_time)

print(f"Returning to initial pose over {move_time} seconds...")
move_to_pose(bus, starting_position, move_time)

print("Motion complete. Disabling torque...")
bus.disable_torque()