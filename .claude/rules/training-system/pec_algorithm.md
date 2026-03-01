# Progressive Expert Coverage (PEC) — Deep Reference

PEC trains K specialist PPO policies, each owning a **Gaussian region** of the
GCR × spcf morphology design space.  Routing at deployment is purely geometric
(nearest Gaussian centre).  No learned gate is used.

Design-space dimensions:
- **GCR** (Gravity Compensation Ratio): `[0.75, 0.89]`
- **spcf** (Spring Coefficient): `[0.001, 0.010]`

Kinematic parameters (femur/tibia lengths) are **not** partitioned by PEC.

---

## Algorithm Structure

```
INIT     pec_init.py
           seed K Gaussians, assign N_init designs per expert
           write pec_state.json

LOOP  (orchestrated by pec_run.py)
  STEP 1  pec_train_expert.py  ×K
            sample GCR/spcf from expert's Gaussian → .npy files
            call train.py subprocess (warm-start from checkpoint)
            parse EXP_DIR: from stdout → store checkpoint in state

  STEP 2  pec_evaluate_frontier.py
            sample F frontier/border candidates
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

```jsonc
{
  "run_name":     "my_run",
  "iteration":    3,               // completed PEC iterations
  "usd_rel_path": "morphologies/.../robot.usd",
  "design_space": { "GCR": [0.75, 0.89], "spcf": [0.001, 0.010] },
  "experts": [
    {
      "id": 0,
      "mu": [0.79, 0.0035],
      "sigma": [[var_gcr, 0.0], [0.0, var_spcf]],   // diagonal covariance
      "designs": [[gcr, spcf], ...],                 // ALL assigned designs (append-only)
      "trained": true,
      "checkpoint": "/abs/path/to/model_best.pt",
      "last_trained_pec_iter": 2                     // canonical "done" marker
    }
  ],
  "history": [                                       // one snapshot per iteration
    {
      "iteration": 2,
      "experts_snapshot": [{ "id": 0, "mu": [...], "sigma": [...],
                             "checkpoint": "...", "n_designs": 42 }]
    }
  ]
}
```

**Key invariants:**
- `designs` stores `[GCR, spcf]` pairs (not dicts) — access as `d[0]`, `d[1]`
- `checkpoint` is always an **absolute** path to `model_best.pt`
- `history` is append-only; `pec_refit_gaussians.py` writes the snapshot
  **before** overwriting `mu`/`sigma`
- `last_trained_pec_iter` is the canonical marker for "expert trained this iteration"
  (replaces brittle path-pattern matching)
- The initial N_init designs stay in `expert["designs"]` forever — intentional
  anchor preventing experts from migrating far from their seed region

---

## Script Roles and CLI Conventions

| Script | Step | Key args |
|--------|------|----------|
| `pec_run.py` | Orchestrator | `--run_name`, `--config`, `--max_pec_iterations`, `--K`, `--max_iterations`, `--max_iterations_iter0` |
| `pec_init.py` | 0 — Bootstrap | `--run_name`, `--K`, `--GCR_range`, `--spcf_range`, `--sigma_scale`, `--centers`, `--usd_rel_path` |
| `pec_train_expert.py` | 1 — Train | `--run_name`, `--expert_id`, `--max_iterations`, `--num_envs`, `--dl` |
| `pec_evaluate_frontier.py` | 2 — Orchestrate eval | `--run_name`, `--F`, `--sampling_mode`, `--num_episodes`, `--start_difficulty` |
| `pec_eval_expert_frontier.py` | 2 — Isaac subprocess | `--checkpoint_path`, `--frontier_file`, `--output` |
| `pec_refit_gaussians.py` | 3 — Refit | `--run_name`, `--min_var_scale`, `--min_designs` |
| `pec_eval_final.py` | Final eval | `--run_name`, `--designs_file`, `--num_episodes`, `--start_difficulty` |
| `pec_visualize.py` | Any — Plot | `--run_name`, `--itr`, `--no_frontier`, `--no_2sigma` |

All orchestrator scripts inject `BALLU_USD_REL_PATH` into Isaac Sim subprocesses
from `state["usd_rel_path"]`.  **Never pass it manually.**

---

## Gaussian Initialisation

```
std_d = sigma_scale × range_d / sqrt(K)
```

| sigma_scale | ~σ as % of range | Overlap | Use when |
|------------|-----------------|---------|----------|
| 0.10 | 7% | ~zero | Very narrow specialists (default) |
| 0.15 | 11% | small | Moderate separation |
| 0.20 | 14% | moderate | Broad initial regions |

`--centers` overrides the automatic grid placement with explicit `[GCR, spcf]`
pairs (length K).

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

## Per-Environment GCR/spcf Parameterisation

PEC requires a different (GCR, spcf) per parallel environment:

1. `pec_train_expert.py` writes per-env samples to `.npy` files
2. Passes `--GCR_samples_file` / `--spcf_samples_file` to `train.py`
3. `train.py` forwards them to `gym.make()` as `GCR_values=` / `spcf_values=` kwargs
4. `morphology_vector_priv()` in `observations.py` reads them with priority:
   `GCR_values` > `GCR_range` > `GCR` (scalar)

Do **not** revert to uniform `GCR_range` sampling when per-env values are needed.

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

Designs file format: `[{"id": 0, "GCR": 0.82, "spcf": 0.005}, ...]`

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

- **X-axis: spcf**, **Y-axis: GCR** (always, in all plots)
- `pec_visualize.py` uses `origin="lower"` — do **not** apply `np.flipud`
- Heatmap alpha fades from log-density; no hard binary coverage mask
- `--itr N` reads from `state["history"][N]`, not current `state["experts"]`
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
│       └── spcf.npy
└── plots/
    ├── initial_state.png
    ├── iter_<N>_frontier.png
    └── after_iter_<N>_current.png
```

Training checkpoints live at:
```
logs/rsl_rl/<experiment_name>/<run_name>/expert_<k>/iter_<N>/model_best.pt
```
