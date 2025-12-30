---
description: "Step-by-step guide for creating and registering new BALLU tasks and environments"
globs:
  - "**/tasks/ballu_locomotion/__init__.py"
  - "**/tasks/ballu_locomotion/*_env_cfg.py"
  - "**/tasks/ballu_locomotion/mdp/**/*.py"
alwaysApply: false
---

# Task Development Guide

For creating new BALLU tasks and environments.

## Quick Start

Every BALLU task requires:
1. Environment Configuration (`*_env_cfg.py`)
2. Task Registration (`__init__.py`)
3. Agent Configuration (usually reuses `BALLUPPORunnerCfg`)

## Step 1: Create Environment Configuration

Use @templates/observation_template.py, @templates/reward_template.py as starting point.

Create `tasks/ballu_locomotion/my_task_env_cfg.py`:

```python
import math
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import *
from isaaclab.utils import configclass
import ballu_isaac_extension.tasks.ballu_locomotion.mdp as mdp
from ballu_isaac_extension.ballu_assets.ballu_config import BALLU_WALKER_CFG

@configclass
class MyTaskEnvCfg(ManagerBasedRLEnvCfg):
    # Scene: robot, terrain, sensors
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)
    
    # MDP components
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    
    # Settings
    decimation = 4
    episode_length_s = 20.0
```

## Step 2: Register Task

Add to `tasks/ballu_locomotion/__init__.py`:

```python
gym.register(
    id="Isaac-MyTask-BALLU",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.my_task_env_cfg:MyTaskEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BALLUPPORunnerCfg"
    }
)
```

## Step 3: Test

```bash
# List all tasks
python scripts/list_envs.py

# Test with minimal training
python scripts/rsl_rl/train.py \
    --task Isaac-MyTask-BALLU \
    --num_envs 256 \
    --max_iterations 10 \
    --headless
```

## Task ID Naming Convention

```
Isaac-<TaskType>-BALLU-<Variant>-<Modifier>

Examples:
- Isaac-Vel-BALLU-priv           (velocity, privileged obs)
- Isaac-Vel-BALLU-imu-tibia      (velocity, IMU sensors)
- Isc-Vel-BALLU-rough            (velocity, rough terrain)
- Isc-BALLU-hetero-general       (heterogeneous morphology)
```

## Common Task Types

### Velocity Tracking (Most Common)

```python
commands = mdp.UniformVelocityCommandCfg(
    ranges=mdp.UniformVelocityCommandCfg.Ranges(
        lin_vel_x=(0.3, 0.5),  # Forward velocity
        lin_vel_y=(0.0, 0.0),  # Lateral (disabled)
        ang_vel_z=(0.0, 0.0),  # Angular (disabled)
    )
)

rewards = RewardsCfg(
    track_velocity = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.0,
        params={"std": 0.5}
    ),
    is_alive = RewTerm(func=mdp.is_alive, weight=1.0)
)
```

### Obstacle Navigation

```python
scene = MySceneCfg(
    obstacles = ObstacleCfg(
        num_obstacles=1,
        height_range=(0.05, 0.15),
        spacing=2.0
    )
)

rewards = RewardsCfg(
    position_tracking = RewTerm(func=mdp.position_tracking_l1, weight=1.0),
    obstacle_clearance = RewTerm(func=mdp.obstacle_clearance, weight=0.5)
)
```

## Observation Space Guidelines

### Minimal (Fast Training)

```python
observations = [
    velocity_commands(2),
    joint_pos(7),
    joint_vel(7),
]
# Total: 16D
```

### With Base State

```python
observations = [
    velocity_commands(2),
    base_orientation(3),    # Projected gravity
    base_lin_vel(3),
    base_ang_vel(3),
    joint_pos(7),
    joint_vel(7),
]
# Total: 25D
```

### With Sensors

```python
observations = [
    velocity_commands(2),
    imu_data(6),           # Linear acc + angular vel
    contact_sensors(2),    # Binary contact per foot
    joint_pos(7),
    joint_vel(7),
]
# Total: 24D
```

### Privileged (Critic Only)

```python
privileged_observations = [
    ...actor observations...,
    base_position(3),
    base_quaternion(4),
    external_forces(3),
    terrain_height_map(N),
]
```

## Reward Design Tips

1. **Normalize reward scales**:
   - Main task reward: weight = 1.0
   - Penalties: weight ∈ [-0.1, -0.001]
   - Bonuses: weight ∈ [0.1, 0.5]

2. **Start simple**:
   ```python
   rewards = RewardsCfg(
       track_velocity = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.0),
       is_alive = RewTerm(func=mdp.is_alive, weight=1.0),
   )
   ```

3. **Add penalties gradually**:
   ```python
   action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
   dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-0.0002)
   ```

4. **Use exponential rewards for tracking**:
   ```python
   track_lin_vel_xy_exp = RewTerm(
       func=mdp.track_lin_vel_xy_exp,
       weight=1.0,
       params={"std": 0.5}  # Controls sharpness
   )
   ```

## Adding Custom MDP Terms

See @templates/ for reward, observation, and termination templates.

### Custom Reward

```python
# In mdp/rewards.py
def my_reward(env: ManagerBasedRLEnv, param1: float) -> torch.Tensor:
    """Compute custom reward.
    
    Args:
        env: Environment instance
        param1: Custom parameter
    
    Returns:
        Reward tensor of shape (num_envs,)
    """
    robot = env.scene["robot"]
    # Compute reward logic
    return reward_tensor
```

### Custom Observation

```python
# In mdp/observations.py
def my_observation(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Compute custom observation.
    
    Returns:
        Observation tensor of shape (num_envs, obs_dim)
    """
    robot = env.scene["robot"]
    # Compute observation
    return obs_tensor
```

## Troubleshooting

### Task Not Found

```
Error: Environment Isaac-MyTask-BALLU doesn't exist
```

**Fix:** Check `__init__.py` registration and ensure import in `train.py`:
```python
import ballu_isaac_extension.tasks  # noqa: F401
```

### Wrong Observation Dimension

```
Error: Expected obs dim X, got Y
```

**Fix:** Count dimensions in `ObservationsCfg` and verify `concatenate_terms=True`

### Action Space Mismatch

```
Error: Expected 2 actions, got N
```

**Fix:** BALLU always has 2 actions (MOTOR_LEFT, MOTOR_RIGHT)

### Environment Won't Reset

```
Error: Some environments never reset
```

**Fix:** Check termination conditions aren't too permissive

## Template Checklist

When creating a new task:
- [ ] Created `*_env_cfg.py` with all required configs
- [ ] Registered in `__init__.py` with meaningful task ID
- [ ] Tested with `python scripts/list_envs.py`
- [ ] Ran short training test (10 iterations)
- [ ] Verified observations have correct dimensions
- [ ] Confirmed action space is 2D
- [ ] Checked reward scales are reasonable
- [ ] Tested termination conditions
- [ ] Documented task purpose in comments
- [ ] Updated relevant README if needed

## Additional Resources

- Environment templates: @templates/
- Isaac Lab Managers: https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.managers.html
- Training workflow: `scripts/rsl_rl/TRAINING_WORKFLOW.md`
- Existing tasks: `tasks/ballu_locomotion/*_env_cfg.py`
