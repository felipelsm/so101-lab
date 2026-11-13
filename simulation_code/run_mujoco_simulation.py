from typing import Final

import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils import *

ROBOT_MODEL_PATH: Final = "model/scene.xml"
DURATION: Final = 3.0

def main() -> None:
  # Initialize model and data
  model = mujoco.MjModel.from_xml_path(ROBOT_MODEL_PATH)
  data = mujoco.MjData(model)

  starting_position = {
      'shoulder_pan': 0.0,   # in degrees
      'shoulder_lift': -80,
      'elbow_flex': 80,
      'wrist_flex': 50,
      'wrist_roll': 0.0,
      'gripper': 0.0        # 0-100 range
  }

  ending_position = {
      'shoulder_pan': 0.0,   # in degrees
      'shoulder_lift': 0.0,
      'elbow_flex': 0.0,
      'wrist_flex': 0.00,
      'wrist_roll': 0.0,
      'gripper': 0.0        # 0-100 range
  }

  # Initialize physics state before launching viewer
  set_initial_pose(data, starting_position)

  # Launch passive viewer and run for 30 wall-seconds
  with mujoco.viewer.launch_passive(model, data) as viewer:
    # Go to desired position
    move_to_pose(model, data, viewer, ending_position, duration=DURATION)
    
    # Hold Position
    hold_position(model, data, viewer, duration=DURATION)
    
    # Return to starting
    move_to_pose(model, data, viewer, starting_position, duration=DURATION)

if __name__ == "__main__":
  main()