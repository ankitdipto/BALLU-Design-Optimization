# BALLU Training Flow Diagram

## High-Level Overview

```
┌─────────────┐
│  train.py   │  Entry point: Parse args, launch Isaac Sim
└──────┬──────┘
       │
       ▼
┌─────────────────────────┐
│   Task Registration     │  gym.register(): Map task ID → configs
│  (__init__.py)          │
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Environment Creation   │  gym.make(): Instantiate ManagerBasedRLEnv
│  (gym.make)             │  Pass BALLU params: GCR, spcf, etc.
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Manager Loading        │  CommandManager → ActionManager →
│  (load_managers)        │  ObservationManager → TerminationManager →
└──────┬──────────────────┘  RewardManager → CurriculumManager
       │
       ▼
┌─────────────────────────┐
│  Runner Creation        │  Create OnPolicyRunner with PPO
│  (OnPolicyRunner)       │  Build ActorCritic network
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│   Training Loop         │  For each iteration:
│  (runner.learn)         │    - Rollout (20 steps × 4096 envs)
└──────┬──────────────────┘    - Compute advantages (GAE)
       │                        - Update policy (PPO)
       │                        - Log and checkpoint
       │
       ▼
     DONE
```

## Detailed Environment Step

```
┌─────────────────────────┐
│   env.step(actions)     │  Input: actions[num_envs, 2]
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Process Actions        │  ActionManager.process_action()
│                         │  → processed_actions (joint targets)
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Physics Loop           │  For _ in range(decimation=4):
│  (4 sub-steps)          │
└──────┬──────────────────┘
       │
       ├─────────────────────┐
       │                     │
       ▼                     ▼
┌──────────────┐     ┌──────────────────┐
│ Apply BALLU  │     │ Indirect         │
│ Physics      │     │ Actuation        │
│              │     │                  │
│ - Buoyancy   │     │ motor_pos →      │
│ - Drag       │     │ knee_target      │
│ - Torques    │     │                  │
└──────┬───────┘     └────────┬─────────┘
       │                      │
       └──────────┬───────────┘
                  │
                  ▼
          ┌───────────────┐
          │  sim.step()   │  PhysX integration at 200Hz
          └───────┬───────┘
                  │
                  └─────── (repeat 4×)
                  
┌─────────────────────────┐
│  Update Counters        │  episode_length_buf += 1
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Compute Rewards        │  RewardManager.compute()
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Check Terminations     │  TerminationManager.compute()
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Reset Terminated       │  _reset_idx(terminated_ids)
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Update Curriculum      │  CurriculumManager.compute()
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Compute Observations   │  ObservationManager.compute()
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Return MDP Tuple       │  (obs, rewards, dones, infos)
└─────────────────────────┘
```

## BALLU Physics Application

```
┌─────────────────────────────────────┐
│      BALLU Physics (Per Step)       │
├─────────────────────────────────────┤
│                                     │
│  1. Buoyancy Force                  │
│     ┌───────────────────────┐       │
│     │ F_buoy = g * m_buoy   │       │
│     │ Convert to local      │       │
│     │ Compute torque        │       │
│     └───────────────────────┘       │
│                                     │
│  2. Drag Force                      │
│     ┌───────────────────────┐       │
│     │ F_drag = -sign(v)*C*v²│       │
│     │ Convert to local      │       │
│     │ Compute torque        │       │
│     └───────────────────────┘       │
│                                     │
│  3. Indirect Actuation              │
│     ┌───────────────────────┐       │
│     │ motor_pos → knee_pos  │       │
│     │ Linear mapping        │       │
│     │ Set joint targets     │       │
│     └───────────────────────┘       │
│                                     │
│  4. Apply to Simulation             │
│     ┌───────────────────────┐       │
│     │ robot.set_external_   │       │
│     │   force_and_torque()  │       │
│     │ PhysX PD control      │       │
│     └───────────────────────┘       │
│                                     │
└─────────────────────────────────────┘
```

## PPO Training Rollout

```
For iteration in [0, max_iterations]:
  │
  ├─ ROLLOUT (with torch.inference_mode):
  │  │
  │  For step in [0, num_steps_per_env]:  # 20 steps
  │    │
  │    ├─ Sample actions: actions = policy(obs, privileged_obs)
  │    │
  │    ├─ Step environment: obs, rewards, dones, infos = env.step(actions)
  │    │
  │    ├─ Normalize: obs = obs_normalizer(obs)
  │    │
  │    └─ Store: storage.add(obs, actions, rewards, dones, values, log_probs)
  │
  ├─ COMPUTE RETURNS:
  │  │
  │  └─ GAE: advantages = rewards + γ*next_values - values
  │
  ├─ UPDATE (5 epochs × 4 mini-batches):
  │  │
  │  For epoch in [0, 5]:
  │    For mini_batch in [0, 4]:
  │      │
  │      ├─ Forward: new_log_probs, entropy, values = policy(obs, actions)
  │      │
  │      ├─ Policy loss: -min(ratio*adv, clamp(ratio)*adv)
  │      │
  │      ├─ Value loss: (returns - values)²
  │      │
  │      ├─ Total: policy_loss + value_loss - entropy_bonus
  │      │
  │      └─ Backprop + gradient clip + optimizer step
  │
  └─ LOG & CHECKPOINT:
     │
     ├─ Log to TensorBoard
     │
     └─ Save model_<iter>.pt, model_best.pt
```

## Manager Dependency Graph

```
┌──────────────────┐
│ CommandManager   │──┐
└──────────────────┘  │
                      │
┌──────────────────┐  │
│ ActionManager    │  │
└──────────────────┘  │
                      │
┌──────────────────┐  │  All depend on
│ ObservationMgr   │──┤  CommandManager
└──────────────────┘  │  and ActionManager
                      │
┌──────────────────┐  │
│ TerminationMgr   │──┤
└──────────────────┘  │
                      │
┌──────────────────┐  │  RewardManager
│ RewardManager    │──┤  depends on
└──────────────────┘  │  TerminationManager
                      │
┌──────────────────┐  │
│ CurriculumMgr    │──┘  (can modify rewards)
└──────────────────┘
```

## Key Takeaways

1. **Vectorized Operations:** Everything operates on `(num_envs, ...)` tensors
2. **Manager Order Matters:** Dependencies between managers
3. **BALLU Physics:** Applied every physics step (200Hz)
4. **Indirect Actuation:** Motor positions → knee targets
5. **Asymmetric Actor-Critic:** Actor sees limited obs, critic sees privileged
6. **PPO Updates:** 5 epochs × 4 mini-batches per iteration
7. **Observation Normalization:** Critical for stable learning
