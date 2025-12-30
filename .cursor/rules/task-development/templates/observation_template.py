"""Template for custom BALLU observation functions."""

import torch
from isaaclab.envs import ManagerBasedRLEnv


def my_custom_observation(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Template for custom observation function.
    
    Args:
        env: The RL environment instance.
    
    Returns:
        Observation tensor of shape (num_envs, obs_dim).
    """
    # Access robot
    robot = env.scene["robot"]
    
    # Example: Return joint positions
    joint_pos = robot.data.joint_pos
    
    return joint_pos


def base_velocity_observation(env: ManagerBasedRLEnv, in_base_frame: bool = True) -> torch.Tensor:
    """Example: Base velocity observation.
    
    Args:
        env: The RL environment instance.
        in_base_frame: If True, return velocity in base frame. Else world frame.
    
    Returns:
        Velocity tensor of shape (num_envs, 3).
    """
    robot = env.scene["robot"]
    
    if in_base_frame:
        return robot.data.root_lin_vel_b  # Base frame
    else:
        return robot.data.root_lin_vel_w  # World frame


def projected_gravity_observation(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Example: Gravity vector projected into base frame.
    
    Useful for orientation awareness without full quaternion.
    
    Args:
        env: The RL environment instance.
    
    Returns:
        Projected gravity of shape (num_envs, 3).
    """
    robot = env.scene["robot"]
    
    # Gravity in world frame
    gravity_w = torch.tensor([0.0, 0.0, -1.0], device=env.device)
    
    # Get base orientation (quaternion)
    quat_w = robot.data.root_quat_w
    
    # Rotate gravity into base frame
    from isaaclab.utils import math_utils
    gravity_b = math_utils.quat_rotate_inverse(quat_w, gravity_w.unsqueeze(0).repeat(env.num_envs, 1))
    
    return gravity_b


def contact_sensor_observation(env: ManagerBasedRLEnv, sensor_name: str = "contact_forces_tibia") -> torch.Tensor:
    """Example: Binary contact observation.
    
    Args:
        env: The RL environment instance.
        sensor_name: Name of the contact sensor in scene.
    
    Returns:
        Binary contact tensor of shape (num_envs, num_bodies).
    """
    sensor = env.scene[sensor_name]
    
    # Get net contact forces
    net_forces = sensor.data.net_forces_w_history[:, :, -1]  # Most recent
    
    # Compute magnitude
    force_magnitude = torch.norm(net_forces, dim=2)
    
    # Binarize (threshold = 1.0 N)
    in_contact = (force_magnitude > 1.0).float()
    
    return in_contact
