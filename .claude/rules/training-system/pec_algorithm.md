# Progressive Expert Coverage (PEC) — Deep Reference

PEC trains K specialist PPO policies, each owning a **Gaussian region** of the
morphology design space.  Routing at deployment is purely geometric (nearest
Gaussian centre).  No learned gate is used.

PEC supports two modes:

| Mode | Design space | Dimensions |
|------|-------------|-----------|
| **2D** (default) | GCR × spcf | GCR `[0.75, 0.89]`, spcf `[0.001, 0.010]` |
| **3D** | GCR × spcf × leg_length | + leg `[0.20, 0.50]` (femur = tibia = leg) |

In 3D mode `leg_length` becomes a third partitioned axis.  Each unique
leg value requires a separate USD file generated at runtime by
`pec_generate_usds.py` before any Isaac Sim subprocess is launched.

---

## Algorithm Structure

```
INIT     pec_init.py
           seed K Gaussians (grid / stochastic_fps)
           optionally calibrate sigma to a target initial coverage
           assign N_init designs per expert
           write pec_state.json

LOOP  (orchestrated by pec_run.py)
  STEP 1  pec_train_expert.py  ×K
            sample GCR/spcf[/leg] from expert's Gaussian → .npy files
            [3D only] call pec_generate_usds.py → generate N_init USD files
                        write BALLU_USD_ORDER_FILE (ordered list of USD paths)
            call train.py subprocess (warm-start from checkpoint)
            parse EXP_DIR: from stdout → store checkpoint in state

  STEP 2  pec_evaluate_frontier.py
            sample F frontier/border candidates
            [3D only] call pec_generate_usds.py → USD files for frontier legs
                        write BALLU_USD_ORDER_FILE ordered by frontier design index
            one subprocess per expert → pec_eval_expert_frontier.py
            each subprocess runs F designs in parallel envs
            record final curriculum level per design per expert
            save candidates.json + scores.json

  STEP 3  pec_refit_gaussians.py
            snapshot current Gaussians to state["history"]
            argmax-assign each frontier design to winning expert
            MLE refit mu + diagonal covariance (variance floor applied)
            increment iteration, save pec_state.json

  STEP 4  pec_visualize.py  (soft — never aborts run)
            plots/iter_<N>_frontier.png  (Gaussians at eval time + frontier)
            plots/after_iter_<N>_current.png  (updated Gaussians after refit)
```

---

## `pec_run.py` — Automated Orchestrator

```bash
python scripts/pec/pec_run.py \
    --run_name   my_run \
    --config     scripts/pec/pec_config_template.yaml \
    [--max_pec_iterations N]
    [--max_iterations_iter0 1600]
    [--max_iterations 500]
    [--K 2]
    [--leg_range 0.20 0.50]   # enables 3D mode; overrides config if set
    [--headless]
```

**Resumability** — every step has a "done" check:

| Step | Skip condition |
|------|---------------|
| Init | `pec_state.json` exists |
| Train expert k at iter N | `expert["last_trained_pec_iter"] == N` and checkpoint exists |
| Frontier eval at iter N | `frontier_evals/iter_N/scores.json` exists |
| Refit at iter N | `state["iteration"] > N` |
| Visualise | Always re-runs (idempotent) |

**dl auto-scaling** — starting obstacle level for each expert:
```
iter 0:  dl = config["dl_initial"]   (default 0)
iter N:  dl = max(0, floor(mean(expert_k_scores_from_iter_N-1)) - dl_buffer)
```
`dl_buffer` (default 2) keeps the expert slightly below its demonstrated ceiling
so it consolidates before pushing further.

**Config persistence** — the effective config (after CLI overrides) is saved to
`logs/pec/<run_name>/pec_config.yaml` on every invocation.

---

## Shared State — `pec_state.json`

Central source of truth.  All scripts read/write this file.

2D example:
```jsonc
{
  "run_name":     "my_run",
  "iteration":    3,
  "N_init":       50,
  "usd_rel_path": "morphologies/.../robot.usd",    // 2D only; absent in 3D
  "init_strategy": "stochastic_fps",
  "init_seed": 123,
  "init_sigma_scale": 0.412,
  "init_target_coverage": 0.817,
  "init_realized_coverage": 0.818,
  "design_space": { "GCR": [0.75, 0.89], "spcf": [0.001, 0.010] },
  "experts": [
    {
      "id": 0,
      "mu": [0.79, 0.0035],
      "sigma": [[var_gcr, 0.0], [0.0, var_spcf]],
      "designs": [[gcr, spcf], ...],
      "trained": true,
      "checkpoint": "/abs/path/to/model_best.pt",
      "last_trained_pec_iter": 2
    }
  ],
  "history": [
    {
      "iteration": 2,
      "experts_snapshot": [{ "id": 0, "mu": [...], "sigma": [...],
                             "checkpoint": "...", "n_designs": 42 }]
    }
  ]
}
```

3D additions (when `design_space` contains `"leg"`):
```jsonc
{
  "design_space": { "GCR": [0.75, 0.89], "spcf": [0.001, 0.010],
                    "leg": [0.20, 0.50] },   // presence enables 3D mode
  "experts": [{
    "mu": [0.79, 0.0035, 0.35],             // 3-element
    "sigma": [[vg,0,0],[0,vs,0],[0,0,vl]],  // 3×3 diagonal
    "designs": [[gcr, spcf, leg], ...]       // triples
  }]
  // "usd_rel_path" absent — USDs generated per-run by pec_generate_usds.py
}
```

**Key invariants:**
- `designs` stores `[GCR, spcf]` pairs (2D) or `[GCR, spcf, leg]` triples (3D) — never dicts; index as `d[0]`, `d[1]`, `d[2]`
- `checkpoint` is always an **absolute** path to `model_best.pt`
- `N_init` is stored in state so downstream scripts can apply the kinematic cap
- `history` is append-only; `pec_refit_gaussians.py` writes the snapshot
  **before** overwriting `mu`/`sigma`
- `last_trained_pec_iter` is the canonical marker for "expert trained this iteration"
- The initial N_init designs stay in `expert["designs"]` forever — intentional
  anchor preventing experts from migrating far from their seed region
- `init_seed` and `init_strategy` must be preserved for reproducible seed sweeps
- In 3D mode `usd_rel_path` is absent; USD paths come from `pec_generate_usds.py`

---

## Script Roles and CLI Conventions

| Script | Step | Key args |
|--------|------|----------|
| `pec_run.py` | Orchestrator | `--run_name`, `--config`, `--max_pec_iterations`, `--K`, `--max_iterations`, `--max_iterations_iter0`, `--leg_range` |
| `pec_init.py` | 0 — Bootstrap | `--run_name`, `--K`, `--GCR_range`, `--spcf_range`, `--sigma_scale`, `--centers`, `--init_strategy`, `--init_seed`, `--target_init_coverage`, `--usd_rel_path` (2D), `--leg_range` (3D) |
| `pec_generate_usds.py` | 1/2 — USD gen (3D) | `--output_dir`, `--designs_file`, `--leg_precision`, `--skip_existing` |
| `pec_train_expert.py` | 1 — Train | `--run_name`, `--expert_id`, `--max_iterations`, `--num_envs`, `--dl` |
| `pec_evaluate_frontier.py` | 2 — Orchestrate eval | `--run_name`, `--F`, `--sampling_mode`, `--num_episodes`, `--start_difficulty` |
| `pec_eval_expert_frontier.py` | 2 — Isaac subprocess | `--checkpoint_path`, `--frontier_file`, `--output` |
| `pec_refit_gaussians.py` | 3 — Refit | `--run_name`, `--min_var_scale`, `--min_designs` |
| `pec_eval_final.py` | Final eval | `--run_name`, `--designs_file`, `--num_episodes`, `--start_difficulty`, `--leg_precision` |
| `pec_visualize.py` | Any — Plot | `--run_name`, `--itr`, `--no_frontier`, `--no_2sigma` |

**2D mode**: orchestrators inject `BALLU_USD_REL_PATH` from `state["usd_rel_path"]` — never pass manually.

**3D mode**: orchestrators set `BALLU_USD_ORDER_FILE` (a JSON file listing F or num_envs ordered absolute USD paths) — `BALLU_USD_REL_PATH` is not used.  See [3D PEC section](#3d-pec--kinematic-design-axis-leg_length) below.

---

## Gaussian Initialisation

Initial standard deviation per axis:

```
2D:  std_d = sigma_scale × range_d / sqrt(K)
3D:  std_d = sigma_scale × range_d / K^(1/3)
```

Automatic center placement modes:

| Mode | Behaviour | Use when |
|------|-----------|----------|
| `grid` | Legacy square-ish grid truncated to K centers (2D only) | Backward-compatible baseline |
| `stochastic_fps` | Boundary-aware, seed-controlled farthest-point sampling in normalized unit hypercube — works for both 2D `(GCR, spcf)` and 3D `(GCR, spcf, leg)` | Recommended for all K and 3D mode |

`--centers` overrides either automatic mode with explicit `[GCR, spcf]` (2D) or
`[GCR, spcf, leg]` (3D) tuples.

If `--target_init_coverage` is set, `pec_init.py` calibrates `sigma_scale` by a
short 1D search on a fixed MC point set and stores the realised value in
`state["init_sigma_scale"]`.

If `--init_strategy stochastic_fps` is used and `--target_init_coverage` is
omitted, the default target is the legacy grid coverage at the same `K` and
user-provided `sigma_scale`. This lets seed sweeps move the initial centers
without materially changing the initial coverage budget.

| sigma_scale | ~σ as % of range | Overlap | Use when |
|------------|------------------|---------|----------|
| 0.10 | 7% | ~zero | Very narrow specialists |
| 0.15 | 11% | small | Moderate separation |
| 0.20 | 14% | moderate | Broad initial regions |

---

## Warm-Starting and Crash Recovery

On every training run after iteration 0, `pec_train_expert.py` passes
`--resume_path <checkpoint>` to `train.py`.

**If training crashes:**
1. `train.py` still prints `EXP_DIR:` before exiting
2. `pec_train_expert.py` parses `EXP_DIR:`, finds `model_best.pt`, updates state,
   and exits **0** (success with warning) so the orchestrator continues
3. If `EXP_DIR:` was never printed (crash before logging), the script exits 1
4. `pec_run.py` then falls back to the expert's previous checkpoint (if any),
   marks `last_trained_pec_iter = N`, and continues to frontier eval

---

## Per-Environment Parameterisation

### 2D (GCR / spcf)

1. `pec_train_expert.py` writes per-env samples to `.npy` files
2. Passes `--GCR_samples_file` / `--spcf_samples_file` to `train.py`
3. `train.py` forwards them to `gym.make()` as `GCR_values=` / `spcf_values=` kwargs
4. `morphology_vector_priv()` in `observations.py` reads them with priority:
   `GCR_values` > `GCR_range` > `GCR` (scalar)

Do **not** revert to uniform `GCR_range` sampling when per-env values are needed.

### 3D (GCR / spcf / leg_length)

GCR and spcf are handled identically to 2D (freely sampled for all `num_envs`
envs from the expert's Gaussian).  `leg_length` requires a separate USD file
per unique value and is handled differently:

1. **N_init kinematic cap** — only `N_init` unique leg values are generated as
   USDs per expert per iteration (even when `num_envs >> N_init`).  Leg values
   are tiled round-robin across all envs: `leg[i] = leg_designs[i % N_init]`.
   This prevents an explosion of USD generation when training with thousands of
   envs.
2. `pec_train_expert.py` calls `pec_generate_usds.py` with the `N_init`
   leg-only designs, gets back a `{key → usd_path}` map, then builds a
   `usd_order.json` of length `num_envs` (round-robin tiled) and writes it to
   `expert_<k>/samples/iter_<N>/usd_order.json`.
3. `BALLU_USD_ORDER_FILE=<abs_path_to_usd_order.json>` is injected into the
   `train.py` subprocess environment.
4. The env config reads `BALLU_USD_ORDER_FILE`, loads the ordered path list,
   and passes it to `MultiUsdFileCfg(random_choice=False)` so that `env i`
   always gets `usd_paths[i]`, making the leg assignment deterministic.

---

## 3D PEC — Kinematic Design Axis (leg_length)

### Overview

Enabling `leg_range` in the config (or passing `--leg_range LO HI` to
`pec_run.py`) activates 3D mode.  `leg_length = femur_length = tibia_length`
(symmetric knees).  The design space becomes GCR × spcf × leg.

### USD Generation (`pec_generate_usds.py`)

Each unique `round(leg, precision)` value requires a distinct USD robot file.
`pec_generate_usds.py` is a **non-Isaac** script — it uses the Python
morphology tools directly:

```bash
python scripts/pec/pec_generate_usds.py \
    --output_dir  logs/pec/my_run/expert_0/usds \
    --designs_file /path/to/designs.json   # [[gcr, spcf, leg], ...] or [[leg], ...]
    --leg_precision 4 \
    --skip_existing                        # reuse cached USDs
```

Output layout per `output_dir`:
```
<output_dir>/
├── leg_0.3000/leg_0.3000.usd
├── leg_0.3500/leg_0.3500.usd
├── urdf/
│   ├── leg_0.3000.urdf
│   ├── leg_0.3500.urdf
│   └── meshes/                ← STL files copied from ballu_assets/old/urdf/meshes/
└── morphology_registry.json   ← cumulative cache {key: {leg, usd_path, morph_id}}
```

Prints `USD_LEG_MAP: {"0.3000": "/abs/path/...", ...}` for the calling script
to parse.

**Mesh assets** — Isaac Sim's URDF importer resolves `package://urdf/` to the
directory containing the URDF (i.e. `urdf/`).  On the first call to
`pec_generate_usds.py` for a given `output_dir`, the 8 STL files are copied
from `source/ballu_isaac_extension/ballu_isaac_extension/ballu_assets/old/urdf/meshes/`
to `<urdf_dir>/meshes/`.  Subsequent calls skip the copy (directory already exists).

### `BALLU_USD_ORDER_FILE` mechanism

Instead of a single `BALLU_USD_REL_PATH`, 3D mode uses:

```
BALLU_USD_ORDER_FILE = /abs/path/to/usd_order.json
```

where `usd_order.json` is a JSON array of `num_envs` absolute USD paths, one
per environment:

```json
[
  "/abs/.../leg_0.3000/leg_0.3000.usd",
  "/abs/.../leg_0.3500/leg_0.3500.usd",
  "/abs/.../leg_0.3000/leg_0.3000.usd",
  ...
]
```

The env config checks `BALLU_USD_ORDER_FILE` first; if set it passes the list
to `MultiUsdFileCfg(random_choice=False)` so `env i → usd_paths[i]`.

### Config

Add to your YAML config to enable 3D mode:

```yaml
leg_range: [0.20, 0.50]
leg_precision: 4          # decimal places for USD key (default 4)
```

See `scripts/pec/configs/obstacle_3d.yaml` for a full example.

---

## Frontier Sampling Modes

| Mode | Selects | Best for |
|------|---------|---------|
| `border` | Annular band between `border_outer_ld` and `border_inner_ld` (log-density) | Early iterations — expand outward gradually |
| `frontier` | Lowest max log-density across all Gaussians | Later iterations — fill uncovered regions |
| `auto` | `border` for iter < `auto_switch_iter`, then `frontier` | Fully automatic schedule (recommended) |

Typical values: `border_inner_ld = -0.5`, `border_outer_ld = -2.5`

---

## Evaluation Protocol

`pec_eval_expert_frontier.py` (Isaac Sim subprocess):
- Spawns F frontier designs as F **parallel** environments
- Runs `--num_episodes` episodes; curriculum manager adjusts obstacle height
- Records the **final** curriculum level (not max) per environment after all
  episodes complete — this is the expert's score for that design
- Prints `FRONTIER_RESULTS: [...]` to stdout for the orchestrator to parse

**PEC oracle** for a design = `max(score_E0, score_E1, ..., score_EK)`

---

## Final Evaluation (`pec_eval_final.py`)

Evaluates all K experts on an **unseen** set of designs:

```bash
python scripts/pec/pec_eval_final.py \
    --run_name     my_run \
    --designs_file logs/pec/my_run/test_1000_uniform.json \
    --num_episodes 16 --start_difficulty 22 --headless
```

Designs file format:
- 2D: `[{"id": 0, "GCR": 0.82, "spcf": 0.005}, ...]`
- 3D: `[{"id": 0, "GCR": 0.82, "spcf": 0.005, "leg": 0.35}, ...]`

In 3D mode `pec_eval_final.py` generates the required USDs automatically before
launching any Isaac Sim subprocess.

Outputs to `logs/pec/<run_name>/final_eval/<designs_stem>/`:
- `expert_<k>_results.json` — full per-design result payload
- `summary.json` — aggregated scores matrix + PEC oracle

---

## Gaussian Refit Mathematics

- Assignment: argmax over experts by score; ties → lower expert ID wins
- Mean: MLE (mean of all assigned designs)
- Covariance: diagonal MLE with variance floor:
  `var_floor = (min_var_scale × range_d)²`  (default `min_var_scale = 0.01`)
- ddof = 1 (Bessel correction) unless only one design assigned (ddof = 0)
- Coverage: MC sampling with `exp(-2)` ≈ 0.135 threshold (~2σ radius),
  10 000 samples, seed 0

---

## Visualisation Conventions

**2D mode** — single heatmap:
- X-axis: spcf, Y-axis: GCR
- `origin="lower"` — do **not** apply `np.flipud`
- Heatmap alpha fades from log-density; no hard binary coverage mask

**3D mode** — single 3D axes (mpl_toolkits.mplot3d):
- Axis mapping: X = spcf, Y = leg, Z = GCR
- Each expert: 1σ solid wireframe ellipsoid + 2σ dashed wireframe + centre marker + design scatter
- Frontier overlay as 3D star scatter (when `--itr` is given)

`--itr N` reads from `state["history"][N]`, not current `state["experts"]`
→ visualise **after** refit to see the state active during iteration N's eval

---

## Log Directory Layout

```
logs/pec/<run_name>/
├── pec_state.json
├── pec_config.yaml                  # effective config saved by pec_run.py
├── frontier_evals/
│   └── iter_<N>/
│       ├── candidates.json
│       ├── scores.json
│       ├── assignments.json
│       └── expert_<k>_results.json
├── final_eval/
│   └── <designs_stem>/
│       ├── expert_<k>_results.json
│       └── summary.json
├── expert_<k>/
│   └── samples/iter_<N>/
│       ├── gcr.npy
│       ├── spcf.npy
│       ├── leg.npy            # 3D only
│       └── usd_order.json     # 3D only — ordered USD paths for num_envs envs
│   └── usds/                  # 3D only — generated USD files
│       ├── leg_0.3000/leg_0.3000.usd
│       ├── urdf/
│       │   ├── leg_0.3000.urdf
│       │   └── meshes/*.STL
│       └── morphology_registry.json
└── plots/
    ├── initial_state.png
    ├── iter_<N>_frontier.png
    └── after_iter_<N>_current.png
```

Training checkpoints live at:
```
logs/rsl_rl/<experiment_name>/<run_name>/expert_<k>/iter_<N>/model_best.pt
```
