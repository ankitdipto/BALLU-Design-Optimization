# BALLU Training System — Deep Reference

## Entry Point

```bash
# From ballu_isclb_extension/
python scripts/rsl_rl/train.py \
    --task     Isc-BALLU-hetero-general \
    --num_envs 4096 \
    --max_iterations 1500 \
    --headless
```

Full argument list:

| Flag | Default | Purpose |
|------|---------|---------|
| `--task` | required | Isaac Lab task ID |
| `--num_envs` | 4096 | Parallel simulation environments |
| `--max_iterations` | 1500 | PPO gradient steps |
| `--seed` | 42 | RNG seed |
| `--device` | `cuda:0` | Torch/CUDA device |
| `--headless` | false | Suppress Isaac Sim GUI |
| `--resume_path` | — | Absolute path to `model_best.pt` for warm-start |
| `--GCR_samples_file` | — | Per-env `.npy` GCR array (PEC usage) |
| `--spcf_samples_file` | — | Per-env `.npy` spcf array (PEC usage) |
| `--common_folder` | — | Sub-directory under experiment name |
| `--run_name` | — | Sub-directory under common_folder |
| `--dl` | 0 | Shift all env origins to curriculum level `dl` at init |

**`train.py` prints `EXP_DIR: <path>` once**, at the end of `main()`, after
`env.close()`.  All PEC orchestrators parse this line to find `model_best.pt`.

Log directory structure:
```
logs/rsl_rl/<experiment_name>/<common_folder>/<run_name>/
    model_best.pt
    model_<iter>.pt
    summaries.txt
```

`experiment_name` comes from `agents/rsl_rl_ppo_cfg.py` (auto-set to lab date,
e.g. `lab_03.03.2026`).  PEC sets:
- `common_folder = <pec_run_name>/expert_<id>`
- `run_name = iter_<N>`

---

## BALLU-Specific Physics (applied every physics step at 200 Hz)

### 1. Buoyancy Force

```python
buoyancy_force_w = gravity * balloon_buoyancy_mass   # per-env tensor
buoyancy_force_l = quat_rotate_inverse(balloon_quat_w, buoyancy_force_w)
distance_from_neck_l = [0.0, -0.38, 0.0]
buoyancy_torque_l = cross(distance_from_neck_l, buoyancy_force_l)
```

### 2. Drag Force

```python
DRAG_COEFFICIENT = 0.0  # typically disabled
drag_force_w = -sign(v) * C * v²
```

### 3. Indirect Actuation (Four-Bar Linkage)

```python
# RL policy outputs: [MOTOR_LEFT, MOTOR_RIGHT] ∈ [0, π]
# Linear mapping to knee targets: [KNEE_LEFT, KNEE_RIGHT] ∈ [0, 1.745 rad]
knee_min, knee_max = 0.0, 1.74532925
target_knee = knee_min + (motor - 0) / π * (knee_max - knee_min)
```

This mapping simulates the physical four-bar linkage for sim-to-real transfer.

---

## Task Registration

Located in `tasks/ballu_locomotion/__init__.py`:

```python
gym.register(
    id="Isc-BALLU-hetero-general",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.hetero_env_cfg:HeteroEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BALLUPPORunnerCfg"
    }
)
```

Task ID naming convention:
```
Isaac-<Type>-BALLU-<Variant>     →  Isaac-Vel-BALLU-imu-tibia
Isc-<Type>-BALLU-<Variant>       →  Isc-BALLU-hetero-general
```

---

## Manager Architecture

Managers load in strict order (later managers depend on earlier ones):

1. **CommandManager** — generates target velocities
2. **ActionManager** — raw actions → joint commands
3. **ObservationManager** — collects sensor data
4. **TerminationManager** — checks episode end conditions
5. **RewardManager** — computes reward signals
6. **CurriculumManager** — adjusts obstacle difficulty

---

## Environment Step Sequence

```
env.step(actions)
  ├─ ActionManager.process_action()
  ├─ for _ in range(decimation=4):
  │     apply_ballu_physics()  ← buoyancy + drag + indirect actuation
  │     sim.step()             ← PhysX at 200 Hz
  ├─ episode_length_buf += 1
  ├─ RewardManager.compute()
  ├─ TerminationManager.compute()
  ├─ _reset_idx(terminated_ids)
  ├─ CurriculumManager.compute()
  └─ ObservationManager.compute()
  → return (obs, rewards, dones, infos)
```

Control frequency: 50 Hz (1 / (4 × 0.005 s))

---

## PPO Training Loop

```
for iteration in range(max_iterations):
    # ROLLOUT (torch.inference_mode)
    for step in range(num_steps_per_env=20):
        actions = policy(obs, privileged_obs)
        obs, rewards, dones, infos = env.step(actions)
        storage.add(...)

    # COMPUTE RETURNS
    GAE advantages

    # UPDATE  (5 epochs × 4 mini-batches)
    policy_loss = -min(ratio*adv, clamp(ratio,1±ε)*adv)
    value_loss  = (returns - values)²
    total_loss  = policy_loss + value_coef * value_loss - entropy_coef * entropy
```

Batch size per iteration: `num_envs × num_steps_per_env`
Example: 4096 × 20 = **81 920 transitions**

---

## Curriculum (`obstacle_height_levels_same_row`)

- **Upgrade**: robot x-pos > obstacle_centre_x → move to harder terrain
- **Downgrade**: robot x-pos < obstacle_centre_x − half_size → move to easier terrain
- Env origins clipped to `[−2 m × 74, 0]` along Y → **max 75 levels**
- Level index = `−origin_y / inter_obstacle_spacing_y` (spacing = 2 m along −Y)
- **Warmup guard**: curriculum skipped if `env.rsl_rl_iteration < warmup_period` (default 100)
- Eval bypasses warmup by setting `isaac_env.rsl_rl_iteration = 1000`
- `--dl <N>` shifts ALL env origins to level N at init

---

## Action and Observation Spaces

### Action Space (always 2-D)

```python
actions[:, 0]  # MOTOR_LEFT  ∈ [0, π]
actions[:, 1]  # MOTOR_RIGHT ∈ [0, π]
```

### Observation — per-env GCR/spcf Encoding

`morphology_vector_priv()` in `observations.py` picks the first available source:
1. `GCR_values` / `spcf_values` — per-env list (set by PEC via `gym.make()` kwargs)
2. `GCR_range` / `spcf_range` — uniform sampling
3. `GCR` / `spcf` — scalar

Do **not** revert to uniform sampling when per-env values are needed.

---

## Performance Reference

| Metric | Value |
|--------|-------|
| FPS (A100) | 10 000–20 000 steps/s |
| Convergence (flat terrain) | 500–1000 iterations |
| GPU memory (4096 envs) | 8–16 GB |
| `model_best.pt` stores | PPO iter at which best curriculum level was first achieved |

The `iter` field in `model_best.pt` reflects **when the best score occurred**,
not the final training iteration.  A warm-start from iter 100 with
`--max_iterations 500` will display as "100/600" in logs.

---

## Key Files

| File | Purpose |
|------|---------|
| `scripts/rsl_rl/train.py` | Entry point + EXP_DIR printer |
| `tasks/ballu_locomotion/__init__.py` | Task registration |
| `tasks/ballu_locomotion/*_env_cfg.py` | Scene, observations, rewards |
| `agents/rsl_rl_ppo_cfg.py` | PPO hyperparameters + experiment name |
| `source/ballu_isaac_extension/.../manager_based_rl_env.py` | BALLU physics, env step |
| `source/ballu_isaac_extension/.../on_policy_runner.py` | PPO training loop |
| `source/ballu_isaac_extension/.../observations.py` | morphology_vector_priv() |

---

## Debugging Tips

- Check `logs/.../summaries.txt` for per-iteration training metrics
- TensorBoard: `tensorboard --logdir logs/`
- Enable empirical normalisation: `empirical_normalization=True` in runner cfg
- Start with flat terrain (`--dl 0`) before obstacle levels
- Test with privileged observations first (easier critic, faster convergence)
