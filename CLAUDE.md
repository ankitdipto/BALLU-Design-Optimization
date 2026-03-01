# BALLU Project — Claude Code Guide

BALLU is a **buoyancy-assisted bipedal robot** trained with PPO (RSL-RL) inside
Isaac Lab / Isaac Sim.  This file gives Claude the minimum context needed to
work on any part of the project.  Deeper, topic-specific rules live in
`.claude/rules/` and are linked below.

---

## 1 · Working conventions

| Item | Value |
|------|-------|
| Conda environment | `BALLU_env0` |
| Primary working directory | `ballu_isclb_extension/` |
| GPU setup | **Single GPU, sequential** — never launch two Isaac Sim processes at once |
| Python entry point | `scripts/rsl_rl/train.py` |
| Default task | `Isc-BALLU-hetero-general` |

All `python` commands below assume the shell is inside `ballu_isclb_extension/`
with `BALLU_env0` activated.

---

## 2 · Repository layout

```
BALLU_Project/
├── ballu_isclb_extension/          ← primary workspace (run everything from here)
│   ├── scripts/
│   │   ├── rsl_rl/train.py         ← PPO training entry point
│   │   ├── pec/                    ← PEC pipeline (see §4)
│   │   ├── analysis/               ← evaluation and plotting scripts
│   │   └── morphology_utils/       ← morphology generation helpers
│   ├── source/ballu_isaac_extension/
│   │   └── tasks/ballu_locomotion/ ← task configs, MDP, agents
│   ├── logs/
│   │   ├── rsl_rl/                 ← training checkpoints
│   │   └── pec/                    ← PEC state and results
│   └── morphologies/               ← USD robot files
└── .claude/rules/                  ← Claude-specific deep-dive rules
```

---

## 3 · BALLU physics — essential facts

- **Action space**: always 2-D — `[MOTOR_LEFT, MOTOR_RIGHT] ∈ [0, π]`
- **Indirect actuation**: motor position → knee target (four-bar linkage sim)
- **Physics**: 200 Hz; control: 50 Hz (decimation = 4); ~0.30 kg total mass
- **Key morphology parameters** encoded in every observation:
  - `GCR` (Gravity Compensation Ratio) — range `[0.75, 0.89]`
  - `spcf` (Spring Coefficient) — range `[0.001, 0.010]`
- Kinematic params (femur/tibia lengths) are **not** used as RL design axes

→ Full details: [.claude/rules/training-system/training_workflow.md](.claude/rules/training-system/training_workflow.md)

---

## 4 · PEC (Progressive Expert Coverage) — quick reference

PEC trains **K specialist PPO policies**, each owning a Gaussian region of
the GCR × spcf design space.

### Automated run (recommended)

```bash
python scripts/pec/pec_run.py \
    --run_name  my_run \
    --config    scripts/pec/pec_config_template.yaml \
    --headless
```

`pec_run.py` handles init → train → eval → refit → visualise loop automatically.
It is **resumable**: re-run the same command after a crash and it picks up where
it left off.

### Manual steps (one-off / debugging)

```bash
# 0  Initialise
python scripts/pec/pec_init.py --run_name my_run --K 2 \
    --GCR_range 0.75 0.89 --spcf_range 0.001 0.010 \
    --sigma_scale 0.10 --N_init 50 --usd_rel_path morphologies/.../robot.usd

# 1  Train expert k at iteration n
python scripts/pec/pec_train_expert.py \
    --run_name my_run --expert_id 0 --max_iterations 500 --dl 0 --headless

# 2  Evaluate frontier
python scripts/pec/pec_evaluate_frontier.py \
    --run_name my_run --F 100 --sampling_mode auto --num_episodes 17 \
    --start_difficulty 21 --headless

# 3  Refit Gaussians
python scripts/pec/pec_refit_gaussians.py --run_name my_run

# Visualise
python scripts/pec/pec_visualize.py --run_name my_run \
    --output logs/pec/my_run/plots/current.png
```

### Final evaluation on a held-out designs file

```bash
python scripts/pec/pec_eval_final.py \
    --run_name      my_run \
    --designs_file  logs/pec/my_run/test_1000_uniform.json \
    --num_episodes  16 --start_difficulty 22 --headless
```

→ Full algorithm details: [.claude/rules/training-system/pec_algorithm.md](.claude/rules/training-system/pec_algorithm.md)

---

## 5 · Training a single policy

```bash
python scripts/rsl_rl/train.py \
    --task     Isc-BALLU-hetero-general \
    --num_envs 4096 \
    --max_iterations 1500 \
    --headless
```

Key flags:

| Flag | Purpose |
|------|---------|
| `--GCR_samples_file / --spcf_samples_file` | Per-env `.npy` arrays (used by PEC) |
| `--resume_path` | Absolute path to `model_best.pt` for warm-start |
| `--dl <int>` | Shift all env origins by `dl × (−2.0 m)` — seeds curriculum at level `dl` |
| `--common_folder / --run_name` | Organise logs under `logs/rsl_rl/<exp>/<common_folder>/<run_name>/` |

`train.py` prints **`EXP_DIR: <path>`** once at the end of `main()`.
All orchestrators parse this line to locate `model_best.pt`.

→ Full training details: [.claude/rules/training-system/training_workflow.md](.claude/rules/training-system/training_workflow.md)

---

## 6 · Creating a new task

Every task requires three pieces:
1. `tasks/ballu_locomotion/<name>_env_cfg.py` — scene, obs, rewards, terminations
2. Registration in `tasks/ballu_locomotion/__init__.py`
3. Agent config — usually reuses `BALLUPPORunnerCfg`

```bash
python scripts/list_envs.py          # verify registration
python scripts/rsl_rl/train.py \
    --task Isaac-MyTask-BALLU --num_envs 256 --max_iterations 10 --headless
```

→ Full guide + templates: [.claude/rules/task-development/task-dev.md](.claude/rules/task-development/task-dev.md)

---

## 7 · Morphology system

BALLU morphologies (URDF → USD) are managed via:
- `scripts/morphology_utils/` — batch generation tools
- `morphologies/` — pre-generated USD libraries
- At runtime, `BALLU_USD_REL_PATH` env var points to the active USD file

→ Full guide: [.claude/rules/morphology-system/morphology_pipeline.md](.claude/rules/morphology-system/morphology_pipeline.md)

---

## 8 · Key invariants — do not violate

- `pec_state.json["designs"]` stores `[GCR, spcf]` pairs (not dicts) — index as `d[0]`, `d[1]`
- `checkpoint` in state is always an **absolute** path to `model_best.pt`
- `state["history"]` is **append-only**; written *before* refit overwrites mu/sigma
- PEC score = **final** curriculum level after last episode reset (not running max)
- Visualisation: X = spcf, Y = GCR; `origin="lower"`; **no** `np.flipud`
- `BALLU_USD_REL_PATH` is always injected from `state["usd_rel_path"]` — never pass manually
- Single GPU: all Isaac Sim subprocesses run **sequentially**
