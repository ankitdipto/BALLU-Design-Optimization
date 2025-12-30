---
description: "Understanding BALLU training pipeline: code flow, managers, physics simulation, and PPO training loop"
globs:
alwaysApply: false
---

# BALLU Training System Architecture

For detailed step-by-step workflow, see: `scripts/rsl_rl/TRAINING_WORKFLOW.md`

## Quick Reference: Training Flow

```
train.py → Task Registration → Environment Creation → Manager Loading → 
Runner Creation → Training Loop → Environment Step → PPO Update
```

### Code Flow Diagram
See @diagrams/training-flow.md for visual representation.

## Entry Point

```bash
python scripts/rsl_rl/train.py --task Isaac-Vel-BALLU-imu-tibia --num_envs 4096
```

**What happens:**
1. Parse CLI arguments (task, num_envs, seed, GCR, spcf)
2. Launch Isaac Sim via `AppLauncher`
3. Hydra loads configs via `@hydra_task_config` decorator
4. Create environment with `gym.make()`
5. Initialize PPO runner
6. Execute training loop

## Task Registration

Located in `tasks/ballu_locomotion/__init__.py`:

```python
gym.register(
    id="Isaac-Vel-BALLU-imu-tibia",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": "<module>.tibia_imu_env_cfg:BALLU_TibiaIMU_EnvCfg",
        "rsl_rl_cfg_entry_point": "<agents>.rsl_rl_ppo_cfg:BALLUPPORunnerCfg"
    }
)
```

**Key insight:** Each task ID bundles environment config + agent config.

## Manager Architecture

Managers are loaded in specific order (dependencies matter!):

1. **CommandManager** - Generates target velocities
2. **ActionManager** - Processes raw actions → joint commands
3. **ObservationManager** - Collects sensor data
4. **TerminationManager** - Checks episode end conditions
5. **RewardManager** - Computes reward signals
6. **CurriculumManager** - Adjusts difficulty

## BALLU-Specific Physics

Applied every physics step (200Hz):

### 1. Buoyancy Force

```python
GRAVITY = torch.tensor([0.0, 0.0, 9.81], device=self.device)
buoyancy_force_w = GRAVITY * self.balloon_buoyancy_mass_t  # Per-env tensor

# Convert to local frame
buoyancy_force_l = quat_rotate_inverse(balloons_quat_w, buoyancy_force_w)

# Compute torque from offset
distance_from_neck_l = torch.tensor([0.0, -0.38, 0.0])
buoyancy_torque_l = torch.cross(distance_from_neck_l, buoyancy_force_l)
```

### 2. Drag Force

```python
DRAG_COEFFICIENT = 0.0  # Usually disabled
robot_balloon_lin_vel_w = robot.data.body_lin_vel_w[:, balloon_body_id, :]
drag_force_w = -torch.sign(robot_balloon_lin_vel_w) * DRAG_COEFFICIENT * (robot_balloon_lin_vel_w ** 2)
```

### 3. Indirect Actuation (Four-Bar Linkage)

```python
# RL policy outputs: [MOTOR_LEFT, MOTOR_RIGHT] ∈ [0, π]
motor_left_action = processed_actions[:, 0]
motor_right_action = processed_actions[:, 1]

# Linear mapping to knee targets: [KNEE_LEFT, KNEE_RIGHT] ∈ [0, 1.745 rad]
knee_min, knee_max = 0.0, 1.74532925
motor_min, motor_max = 0.0, math.pi

target_knee_left = knee_min + (motor_left_action - motor_min) / (motor_max - motor_min) * (knee_max - knee_min)
target_knee_right = knee_min + (motor_right_action - motor_min) / (motor_max - motor_min) * (knee_max - knee_min)
```

**Why?** Simulates the real BALLU's four-bar linkage for sim-to-real transfer.

## Environment Step Sequence

```python
def step(self, actions):
    # 1. Process actions
    self.action_manager.process_action(actions)
    
    # 2. Physics loop with decimation (4 sub-steps)
    for _ in range(self.cfg.decimation):
        # Apply BALLU physics (buoyancy, drag, indirect actuation)
        # Simulate one physics step at 200Hz
        self.sim.step(render=False)
    
    # 3. Update counters
    self.episode_length_buf += 1
    
    # 4. Compute rewards
    self.reward_manager.compute(dt=self.step_dt)
    
    # 5. Check terminations
    self.termination_manager.compute(dt=self.step_dt)
    
    # 6. Reset terminated environments
    # 7. Update curriculum
    # 8. Compute observations
    # 9. Return (obs, rewards, dones, infos)
```

**Timing:**
- Control frequency: 50 Hz (1 / (decimation × sim_dt))
- Physics frequency: 200 Hz (1 / sim_dt)

## Training Loop

```python
for iteration in range(num_learning_iterations):
    # ========== ROLLOUT PHASE ==========
    with torch.inference_mode():
        for step in range(num_steps_per_env):  # Default: 20 steps
            # Sample actions from policy
            actions = self.alg.act(obs, privileged_obs)
            
            # Step environment
            obs, rewards, dones, infos = self.env.step(actions)
            
            # Normalize observations
            obs = self.obs_normalizer(obs)
            
            # Store rollout data
            self.alg.process_env_step(rewards, dones, infos)
    
    # Compute advantages using GAE
    self.alg.compute_returns(privileged_obs)
    
    # ========== UPDATE PHASE ==========
    loss_dict = self.alg.update()  # 5 epochs × 4 mini-batches
    
    # ========== LOGGING ==========
    if iteration % save_interval == 0:
        self.save(f"model_{iteration}.pt")
```

**Batch size per iteration:** `num_envs × num_steps_per_env`
- Example: 4096 envs × 20 steps = **81,920 transitions**

## Action and Observation Spaces

### Action Space (Always 2D)

```python
actions[:, 0]  # MOTOR_LEFT target position [0, π]
actions[:, 1]  # MOTOR_RIGHT target position [0, π]
```

### Typical Observation Structure

```python
obs = [
    velocity_commands(2),     # Target velocities
    joint_pos(7),             # All joint positions
    joint_vel(7),             # All joint velocities
    # Optional: IMU data, contact sensors, etc.
]
# Total: 16D (minimal)
```

### Privileged Observations (Critic)

```python
privileged_obs = [
    ...actor observations...,
    ground_truth_state,       # Position, orientation
    external_forces,          # True physics state
    terrain_info,             # Height maps, friction
]
```

## PPO Update Mechanics

```python
# Compute policy ratio
ratio = exp(new_log_probs - old_log_probs)

# Clipped surrogate objective
surr1 = ratio * advantages
surr2 = clamp(ratio, 1-clip_param, 1+clip_param) * advantages
policy_loss = -min(surr1, surr2).mean()

# Value loss (clipped)
value_loss = (returns - values).pow(2).mean()

# Total loss
total_loss = policy_loss + value_coef * value_loss - entropy_coef * entropy
```

## Code Generation Guidelines

When generating training-related code:

1. ✅ **Always check task registration** - Task must be in `__init__.py`
2. ✅ **Use BALLU-specific parameters** - GCR, spcf are critical
3. ✅ **Respect manager order** - Dependencies exist between managers
4. ✅ **Handle vectorized tensors** - All operations batched over `num_envs`
5. ✅ **Action dimension is 2** - Never assume different action space
6. ✅ **Physics runs at 200Hz** - Control at 50Hz (decimation=4)

## Common Patterns

### Adding Observation Term

```python
# In mdp/observations.py
def my_observation(env: ManagerBasedRLEnv) -> torch.Tensor:
    robot = env.scene["robot"]
    # Compute observation
    return obs_tensor  # Shape: (num_envs, obs_dim)

# In *_env_cfg.py
my_obs = ObsTerm(func=mdp.my_observation, params={"key": "value"})
```

### Adding Reward Term

```python
# In mdp/rewards.py
def my_reward(env: ManagerBasedRLEnv, param1: float) -> torch.Tensor:
    # Compute reward
    return reward_tensor  # Shape: (num_envs,)

# In *_env_cfg.py
my_reward = RewTerm(func=mdp.my_reward, weight=1.0, params={"param1": 0.5})
```

## Debugging Tips

- Check `logs/.../summaries.txt` for training metrics
- Use TensorBoard: `tensorboard --logdir logs/`
- Enable observation normalization: `empirical_normalization=True`
- Start with flat terrain before rough/obstacles
- Test with privileged observations first (easier)

## Performance Expectations

- **FPS**: 10,000-20,000 steps/s (A100 GPU)
- **Convergence**: 500-1000 iterations for flat terrain
- **Memory**: 8-16 GB GPU for 4096 environments

## Key Files Reference

| File | Purpose |
|------|---------|
| `train.py` | Entry point, orchestration |
| `tasks/ballu_locomotion/__init__.py` | Task registration |
| `*_env_cfg.py` | Scene, observations, rewards |
| `agents/rsl_rl_ppo_cfg.py` | PPO hyperparameters |
| `manager_based_rl_env.py` | BALLU physics, environment step |
| `on_policy_runner.py` | PPO training loop |

## Additional Resources

- **Detailed workflow:** `scripts/rsl_rl/TRAINING_WORKFLOW.md`
- **Isaac Lab docs:** https://isaac-sim.github.io/IsaacLab
- **RSL-RL paper:** [Learning to Walk in Minutes](https://arxiv.org/abs/2109.11978)
- **PPO paper:** [Proximal Policy Optimization](https://arxiv.org/abs/1707.06347)
