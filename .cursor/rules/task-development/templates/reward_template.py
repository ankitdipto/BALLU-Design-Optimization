"""Template for custom BALLU reward functions."""

import torch
from isaaclab.envs import ManagerBasedRLEnv


def my_custom_reward(env: ManagerBasedRLEnv, param1: float = 1.0) -> torch.Tensor:
    """Template for custom reward function.
    
    Args:
        env: The RL environment instance.
        param1: Example parameter that can be configured in RewardTermCfg.
    
    Returns:
        Reward tensor of shape (num_envs,) with values typically in [-1, 1] range.
    """
    # Access robot
    robot = env.scene["robot"]
    
    # Example: Reward based on joint velocities
    joint_vel = robot.data.joint_vel
    
    # Compute reward (example: penalize high velocities)
    reward = -param1 * torch.sum(torch.abs(joint_vel), dim=1)
    
    return reward


def velocity_tracking_reward(env: ManagerBasedRLEnv, std: float = 0.5) -> torch.Tensor:
    """Example: Exponential reward for velocity tracking.
    
    Args:
        env: The RL environment instance.
        std: Standard deviation for exponential function.
    
    Returns:
        Reward tensor of shape (num_envs,).
    """
    robot = env.scene["robot"]
    
    # Get commanded velocity
    command = env.command_manager.get_command("base_velocity")
    
    # Get actual velocity (base frame)
    actual_vel = robot.data.root_lin_vel_b[:, :2]  # x, y components
    
    # Compute tracking error
    error = torch.sum(torch.square(actual_vel - command[:, :2]), dim=1)
    
    # Exponential reward
    reward = torch.exp(-error / (std ** 2))
    
    return reward


def energy_penalty(env: ManagerBasedRLEnv, weight: float = 1e-4) -> torch.Tensor:
    """Example: Penalize energy consumption.
    
    Args:
        env: The RL environment instance.
        weight: Weight for penalty (already negative).
    
    Returns:
        Penalty tensor of shape (num_envs,).
    """
    robot = env.scene["robot"]
    
    # Get joint torques and velocities
    torques = robot.data.applied_torque
    velocities = robot.data.joint_vel
    
    # Power = torque * velocity
    power = torch.abs(torques * velocities)
    
    # Sum over all joints
    total_power = torch.sum(power, dim=1)
    
    # Return negative (penalty)
    return -weight * total_power
