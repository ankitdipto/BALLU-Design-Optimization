# Task Development Guide

For creating new BALLU tasks and environments.

---

## Quick Start

Every BALLU task requires:
1. **Environment configuration** — `tasks/ballu_locomotion/<name>_env_cfg.py`
2. **Task registration** — `tasks/ballu_locomotion/__init__.py`
3. **Agent configuration** — usually reuses `BALLUPPORunnerCfg`

```bash
# Verify registration
python scripts/list_envs.py

# Quick smoke test
python scripts/rsl_rl/train.py \
    --task Isaac-MyTask-BALLU --num_envs 256 --max_iterations 10 --headless
```

---

## Step 1 — Create Environment Configuration

Create `tasks/ballu_locomotion/<name>_env_cfg.py`:

```python
import math
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObsTerm, RewTerm, ObsGroup
from isaaclab.utils import configclass
import ballu_isaac_extension.tasks.ballu_locomotion.mdp as mdp
from ballu_isaac_extension.ballu_assets.ballu_config import BALLU_WALKER_CFG

@configclass
class MyTaskEnvCfg(ManagerBasedRLEnvCfg):
    scene:        MySceneCfg      = MySceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions:      ActionsCfg      = ActionsCfg()
    commands:     CommandsCfg     = CommandsCfg()
    rewards:      RewardsCfg      = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    decimation       = 4        # control at 50 Hz (physics at 200 Hz)
    episode_length_s = 20.0
```

See `templates/` for reward and observation boilerplate.

---

## Step 2 — Register Task

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

### Task ID naming convention

```
Isaac-<TaskType>-BALLU-<Variant>-<Modifier>

Examples:
  Isaac-Vel-BALLU-priv           velocity + privileged obs
  Isaac-Vel-BALLU-imu-tibia      velocity + IMU sensors
  Isc-Vel-BALLU-rough            velocity + rough terrain
  Isc-BALLU-hetero-general       heterogeneous morphology
```

---

## Common Patterns

### Velocity Tracking (Most Common)

```python
@configclass
class CommandsCfg:
    velocity = mdp.UniformVelocityCommandCfg(
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.3, 0.5),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(0.0, 0.0),
        )
    )

@configclass
class RewardsCfg:
    track_velocity = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=1.0, params={"std": 0.5}
    )
    is_alive = RewTerm(func=mdp.is_alive, weight=1.0)
```

### Adding an Observation Term

```python
# In mdp/observations.py
def my_observation(env: ManagerBasedRLEnv) -> torch.Tensor:
    robot = env.scene["robot"]
    # compute ...
    return obs_tensor   # shape: (num_envs, obs_dim)

# In *_env_cfg.py
my_obs = ObsTerm(func=mdp.my_observation)
```

### Adding a Reward Term

```python
# In mdp/rewards.py
def my_reward(env: ManagerBasedRLEnv, param1: float) -> torch.Tensor:
    # compute ...
    return reward_tensor   # shape: (num_envs,)

# In *_env_cfg.py
my_reward = RewTerm(func=mdp.my_reward, weight=1.0, params={"param1": 0.5})
```

---

## Observation Space Sizes

| Config | Dimensions |
|--------|-----------|
| Minimal (commands + joints) | 16 D |
| + Base state (orientation, lin/ang vel) | 25 D |
| + IMU + contact sensors | 24 D |
| + Morphology vector (GCR, spcf) | +2 D |

Privileged observations (critic only) include base position/quaternion,
external forces, and terrain height maps on top of actor observations.

---

## Reward Design Tips

1. Main task reward: `weight = 1.0`
2. Penalties: `weight ∈ [−0.1, −0.001]`
3. Bonuses: `weight ∈ [0.1, 0.5]`
4. Use exponential rewards for smooth velocity tracking:
   `exp(−||v_cmd − v_actual||² / std²)`
5. Add `is_alive` early — prevents trivially-terminated episodes

---

## Checklist for New Tasks

- [ ] `*_env_cfg.py` created with all required config classes
- [ ] Registered in `__init__.py` with meaningful ID
- [ ] `python scripts/list_envs.py` shows the task
- [ ] Short training test passes (10 iterations, 256 envs)
- [ ] Observation dimensions verified (count terms manually)
- [ ] Action space confirmed as 2-D (MOTOR_LEFT, MOTOR_RIGHT)
- [ ] Reward scales reasonable (run 100 iters, check summaries.txt)
- [ ] Termination conditions tested

---

## Troubleshooting

| Error | Fix |
|-------|-----|
| `Environment <id> doesn't exist` | Check `__init__.py` registration and `import ballu_isaac_extension.tasks` in `train.py` |
| `Expected obs dim X, got Y` | Count dimensions in `ObservationsCfg`, verify `concatenate_terms=True` |
| `Expected 2 actions, got N` | BALLU always has 2 actions |
| `Some environments never reset` | Check termination conditions aren't too permissive |

---

## Additional Resources

- Templates: [templates/](templates/)
- Isaac Lab Managers API: https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.managers.html
- Existing tasks: `tasks/ballu_locomotion/*_env_cfg.py`
- Training details: [../training-system/training_workflow.md](../training-system/training_workflow.md)
