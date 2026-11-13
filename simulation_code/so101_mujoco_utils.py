from typing import List, Dict

import math
import time
import mujoco

def convert_to_dictionary(qpos: List[float]) -> Dict[str, float]:
    """Converts a joint position list (radians) to a named dictionary (degrees/percentage)."""
    return {
        'shoulder_pan': math.degrees(qpos[0]),    # convert to degrees
        'shoulder_lift': math.degrees(qpos[1]),   # convert to degrees
        'elbow_flex': math.degrees(qpos[2]),      # convert to degrees
        'wrist_flex': math.degrees(qpos[3]),      # convert to degrees
        'wrist_roll': math.degrees(qpos[4]),      # convert to degrees
        'gripper': (qpos[5] / math.pi) * 100.0    # convert to 0-100 range
    }
    
def convert_to_list(joint_dict: Dict[str, float]) -> List[float]:
    """Converts a named dictionary (degrees/percentage) back to a joint position list (radians)."""
    return [
        math.radians(joint_dict['shoulder_pan']),
        math.radians(joint_dict['shoulder_lift']),
        math.radians(joint_dict['elbow_flex']),
        math.radians(joint_dict['wrist_flex']),
        math.radians(joint_dict['wrist_roll']),
        (joint_dict['gripper'] / 100.0) * math.pi
    ]

def set_initial_pose(data: mujoco.MjData, position_dict: Dict[str, float]) -> None:
    """Sets the initial joint positions of the robot."""
    data.qpos = convert_to_list(position_dict)

def send_position_command(data: mujoco.MjData, position_dict: Dict[str, float]) -> None:
    """Applies position control commands to the robot's actuators."""
    data.ctrl = convert_to_list(position_dict)

def move_to_pose(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    viewer: mujoco.viewer.Handle,
    desired_position: Dict[str, float],
    duration: float
) -> None:
    """Interpolates smoothly to move the robot to a target pose over a set duration"""
    start_time = time.time()
    sim_start = d.time
    starting_pose = d.qpos.copy()
    starting_pose = convert_to_dictionary(starting_pose)
    
    while True:
        t = time.time() - start_time
        if t > duration:
            break

        # Interpolation factor [0,1] (make sure it doesn't exceed 1)
        alpha = min(t / duration, 1)

        # Interpolate each joint
        position_dict = {}
        for joint in desired_position:
            p0 = starting_pose[joint]
            pf = desired_position[joint]
            position_dict[joint] = (1 - alpha) * p0 + alpha * pf

        # Send command
        send_position_command(d, position_dict)
        while d.time - sim_start < time.time() - start_time:
            mujoco.mj_step(m, d)
        
        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()
    
def hold_position(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    viewer: mujoco.viewer.Handle,
    duration: float
) -> None:
    """Holds the current robot position for a specified duration"""
    current_pos = d.qpos.copy()
    current_pos_dict = convert_to_dictionary(current_pos)
    
    start_time = time.time()
    while True:
        t = time.time() - start_time
        if t > duration:
            break
        send_position_command(d, current_pos_dict)
        mujoco.mj_step(m, d)
        viewer.sync()