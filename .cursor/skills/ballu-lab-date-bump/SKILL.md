---
name: ballu-lab-date-bump
description: Bumps the canonical BALLU "lab date" (MM.DD.YYYY) across training and morphology paths. Use when the user asks to update the lab date, next lab date, weekly lab folder, morphologies date folder, or experiment_name lab_* strings.
---

# BALLU lab date bump

## When to run

The user wants the **current** lab session date updated (typically weekly). Format is always **`MM.DD.YYYY`** (e.g. `04.07.2026`).

If they did not give the new date, ask once for it in that format.

## Source of truth for the *old* date

Read the previous value from:

`ballu_isclb_extension/source/ballu_isaac_extension/ballu_isaac_extension/morphology/constants.py` → `NEXT_LAB_DATE = "MM.DD.YYYY"`

Call that string `OLD`. The user provides `NEW`.

## Files to update (replace `OLD` → `NEW`)

| Location | What changes |
|----------|----------------|
| `morphology/constants.py` | `NEXT_LAB_DATE` value |
| `tasks/ballu_locomotion/agents/rsl_rl_ppo_cfg.py` | Every `experiment_name = "lab_<date>"` (same count as today: typically five) |
| `scripts/morphology_utils/explore_morphology.py` | `morphologies/<date>/` in `BALLU_USD_REL_PATH` (two places) |
| `scripts/morphology_utils/explore_morphology_walking.py` | same |
| `scripts/morphology_utils/explore_morphology_cmaes.py` | same |

**Already wired to `NEXT_LAB_DATE`** (no hardcoded date string once constants are updated): `morphology/robot_generator.py`, `scripts/morphology_utils/generate_morphology_library.py`.

## Verification

1. `rg -F 'OLD'` inside `ballu_isclb_extension/` — should return **no** matches after the bump.
2. Optionally `rg -F 'NEW'` on the paths above to confirm.

## Other matches from search

If `rg` finds `OLD` under `scripts/analysis/`, docs, or comments pointing at **historical** log runs, **do not** change those unless the user explicitly asked to rewrite old paths. The weekly workflow is only the canonical list above.

## Filesystem reminder

On disk, USD/URDF output uses folders named like `morphologies/<NEW>/`. Tell the user they must **create, copy, or regenerate** assets under the new folder name if they still rely on the old dated tree.
