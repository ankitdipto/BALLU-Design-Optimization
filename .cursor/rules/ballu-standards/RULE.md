---
description: "Core BALLU project standards: naming conventions, robot specifications, coding patterns, and file organization"
alwaysApply: true
---

# BALLU Project Standards

Apply these standards to all BALLU project code for consistency and maintainability.

## Naming Conventions

### Task IDs

Follow this pattern:
```
Isaac-<TaskType>-BALLU-<Sensor>-<Terrain>

Examples:
- Isaac-Vel-BALLU-priv        (velocity tracking, privileged obs)
- Isaac-Vel-BALLU-imu-tibia   (velocity tracking, tibia IMU sensors)
- Isaac-Vel-BALLU-1-obstacle  (velocity, single obstacle)
- Isc-BALLU-hetero-general    (heterogeneous morphology)
```

### File Naming

- Environment configs: `<task_description>_env_cfg.py`
- MDP functions: `<component>.py` (rewards.py, observations.py, terminations.py)
- Scripts: Use snake_case (train.py, multi_run_training.py)

### Variable Naming

```python
# BALLU-specific
self.balloon_buoyancy_mass_t   # Per-environment buoyancy mass (tensor)
self.GCR                        # Gravity Compensation Ratio
self.spcf                       # Spring coefficient

# Standard Isaac Lab
robot = env.scene["robot"]      # Always access robot this way
num_envs = self.scene.num_envs  # Number of parallel environments
self.device                     # torch.device for tensors
```

### Experiment Naming

```python
# In PPO config
experiment_name = "lab_12.10.2025"       # Date-based
experiment_name = "obstacle_study"       # Topic-based
experiment_name = "fl_ratio_exploration" # Parameter-based
```

**Guidelines:**
- Use date format: `lab_MM.DD.YYYY` for daily experiments
- Use descriptive names for studies: `<topic>_<modifier>`
- Use underscores, not spaces or hyphens

## BALLU Robot Specifications

### Joint Configuration
See @specs/robot_joints.json for complete specification.

```python
joint_names = [
    "NECK",         # Index 0
    "HIP_LEFT",     # Index 1
    "HIP_RIGHT",    # Index 2
    "KNEE_LEFT",    # Index 3
    "KNEE_RIGHT",   # Index 4
    "MOTOR_LEFT",   # Index 5
    "MOTOR_RIGHT"   # Index 6
]
```

### Action Space (Always 2D)

```python
# RL policy outputs
actions[:, 0]  # MOTOR_LEFT target position [0, π]
actions[:, 1]  # MOTOR_RIGHT target position [0, π]

# These map to knee targets via indirect actuation
```

### Buoyancy Parameters

```python
# Default values
GCR = 0.84                              # Gravity Compensation Ratio (84% of weight)
DRAG_COEFFICIENT = 0.0                  # Balloon drag (usually disabled)
buoyancy_offset = [0.0, -0.38, 0.0]    # Force application point (local frame)
```

### Body/Link IDs

```python
balloon_body_id = 3  # Balloon body for buoyancy force application
# Use robot.find_bodies("BALLOON") for dynamic lookup
```

## Code Organization

### Directory Structure

```
ballu_isclb_extension/
├── scripts/
│   ├── rsl_rl/          # Training and evaluation scripts
│   ├── analysis/         # Results analysis tools
│   └── morphology_utils/ # Morphology generation/exploration
├── source/ballu_isaac_extension/ballu_isaac_extension/
│   ├── ballu_assets/     # Robot configs, morphology loader
│   ├── tasks/            # Environment definitions
│   │   └── ballu_locomotion/
│   │       ├── __init__.py              # Task registration
│   │       ├── *_env_cfg.py             # Environment configs
│   │       ├── agents/                   # RL agent configs
│   │       └── mdp/                      # MDP components
│   │           ├── rewards.py
│   │           ├── observations.py
│   │           ├── terminations.py
│   │           └── curriculums.py
│   └── morphology/       # Morphology system
```

### Import Organization

```python
# Standard library
import math
import os

# Third-party
import torch
import gymnasium as gym

# Isaac Lab
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import RewardTermCfg as RewTerm

# Local
import ballu_isaac_extension.tasks.ballu_locomotion.mdp as mdp
```

## Tensor Operations

### Always Vectorized

```python
# Good: Vectorized over all environments
rewards = torch.sum(tensor, dim=1)  # Shape: (num_envs,)

# Bad: Loop over environments
for i in range(num_envs):
    rewards[i] = compute_reward(i)
```

### Device Management

```python
# Always specify device
tensor = torch.zeros(num_envs, 3, device=self.device)

# For constants
GRAVITY = torch.tensor([0.0, 0.0, 9.81], device=self.device)
```

### Shape Conventions

```python
observations:     (num_envs, obs_dim)
actions:          (num_envs, action_dim)
rewards:          (num_envs,) or (num_envs, 1)
dones:            (num_envs,) (boolean)
joint_positions:  (num_envs, num_joints)
body_positions:   (num_envs, num_bodies, 3)
```

## Configuration Classes

### Use @configclass Decorator

```python
from isaaclab.utils import configclass

@configclass
class MyEnvCfg(ManagerBasedRLEnvCfg):
    # Configuration fields
    pass
```

### Nested Configurations

```python
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        velocity_commands = ObsTerm(...)
        joint_pos = ObsTerm(...)
    
    policy: PolicyCfg = PolicyCfg()
```

## Documentation

### Docstrings

```python
def my_function(env: ManagerBasedRLEnv, param: float) -> torch.Tensor:
    """Brief description.
    
    Args:
        env: The RL environment instance.
        param: Description of parameter.
    
    Returns:
        Tensor of shape (num_envs, dim) containing the result.
    """
```

### Comments

- Explain **why**, not **what**
- Document BALLU-specific physics
- Note any sim-to-real considerations

## Git Workflow

### Commit Messages

```
feat: add new reward term for energy efficiency
fix: correct buoyancy torque calculation
docs: update training workflow documentation
refactor: simplify indirect actuation logic
```

### Branch Naming

- `feature/<description>`
- `bugfix/<description>`
- `experiment/<name>`

## Common Pitfalls to Avoid

1. ❌ **Don't hardcode number of environments** - Use `self.scene.num_envs`
2. ❌ **Don't assume action dimension** - Always check `self.env.num_actions`
3. ❌ **Don't forget device** - All tensors must be on correct device
4. ❌ **Don't break indirect actuation** - Knee targets come from motor positions
5. ❌ **Don't modify submodules** - Isaac Lab and RSL-RL are forks
6. ❌ **Don't skip observation normalization** - Critical for stable training
7. ❌ **Don't use large reward magnitudes** - Keep rewards in [-10, 10] range

## Testing Checklist

Before committing changes:
- [ ] Code runs without errors
- [ ] Training converges on flat terrain
- [ ] Action space is still 2D
- [ ] Buoyancy is correctly applied
- [ ] Indirect actuation works
- [ ] Logs are properly saved
- [ ] Documentation is updated

## References

- Main README: `ballu_isclb_extension/README.md`
- Training workflow: `scripts/rsl_rl/TRAINING_WORKFLOW.md`
- Morphology system: `source/.../morphology/README.md`
- Multi-run training: `scripts/rsl_rl/README_multi_run_training.md`
