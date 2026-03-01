# Claude Code Rules — BALLU Project

Topic-specific deep-dive references for Claude.  Start with `CLAUDE.md` at
the project root; come here for details.

```
.claude/rules/
├── training-system/
│   ├── pec_algorithm.md        PEC algorithm, state schema, script roles, crash handling
│   └── training_workflow.md    train.py flags, BALLU physics, PPO loop, curriculum
├── task-development/
│   ├── task-dev.md             Creating new Isaac Lab tasks, registration, MDP patterns
│   └── templates/
│       ├── observation_template.py
│       └── reward_template.py
└── morphology-system/
    ├── morphology_pipeline.md  Morphology params, USD pipeline, heterogeneous training
    └── examples/
        └── morphology_examples.py
```

## When to consult each file

| Working on… | Read |
|-------------|------|
| PEC pipeline (`scripts/pec/`) | `training-system/pec_algorithm.md` |
| `train.py`, PPO config, curriculum | `training-system/training_workflow.md` |
| New task / env config / MDP terms | `task-development/task-dev.md` |
| Morphology generation, USD files | `morphology-system/morphology_pipeline.md` |
