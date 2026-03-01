# BALLU Morphology System — Deep Reference

System for defining, generating, and exploring BALLU's physical design space.

---

## Parameter Space

### Geometry

| Parameter | Default | Range |
|-----------|---------|-------|
| `femur_length` | 0.365 m | (0.2, 0.6) |
| `tibia_length` | 0.385 m | (0.2, 0.6) |
| `limb_radius` | 0.005 m | — |
| `hip_width` | 0.116 m | — |
| `balloon_radius` | 0.32 m | (0.2, 0.5) |
| `balloon_height` | 0.70 m | (0.4, 1.0) |

### Mass

| Parameter | Default | Range |
|-----------|---------|-------|
| `pelvis_mass` | 0.021 kg | — |
| `femur_mass` | 0.009 kg | — |
| `tibia_mass` | 0.044 kg | — |
| `balloon_mass` | 0.159 kg | (0.05, 0.40) |
| **Total** | ~0.30 kg | — |

### Derived PEC Design Parameters

These two scalar quantities are the RL design axes used by PEC:

- **GCR** (Gravity Compensation Ratio): `[0.75, 0.89]`
  Ratio of balloon buoyancy to total robot weight.
- **spcf** (Spring Coefficient): `[0.001, 0.010]`
  Stiffness of the spring in the four-bar linkage.

Kinematic parameters (femur/tibia lengths) are **not** partitioned by PEC.

---

## USD File Convention

Each morphology is stored as a USD file.  The active USD is resolved at runtime
via:

```bash
export BALLU_USD_REL_PATH="morphologies/hetero_library_hvyBloon_lab01.20.26/hetero_0001_fl0.312_tl0.398/hetero_0001_fl0.312_tl0.398.usd"
```

In PEC runs this is set automatically from `pec_state["usd_rel_path"]` and
injected into every Isaac Sim subprocess.  **Never override it manually** when
running PEC scripts.

### Morphology library layout

```
morphologies/
└── hetero_library_<version>/
    └── hetero_<id>_fl<femur>_tl<tibia>/
        ├── hetero_<id>_fl<femur>_tl<tibia>.usd
        └── morphology.json
```

---

## Creating Morphologies

### From Python

```python
from ballu_morphology_config import BalluMorphology, create_morphology_variant

# Default morphology
morph = BalluMorphology.default()

# Custom by femur-to-limb ratio
morph = create_morphology_variant(
    morphology_id="fl_ratio_0.40",
    femur_to_limb_ratio=0.40,
)

# Validate before generating
is_valid, errors = morph.validate()
if not is_valid:
    print(errors)

# Save / load
morph.to_json("morphologies/my_morphology.json")
morph = BalluMorphology.from_json("morphologies/my_morphology.json")
```

### URDF → USD Pipeline

```bash
# 1. Generate URDF
python scripts/morphology_utils/generate_urdf.py \
    --morphology_id my_morph \
    --femur_to_limb_ratio 0.40

# 2. Convert to USD (requires Isaac Sim)
python morphology/convert_urdf.py \
    output.urdf output.usd \
    --merge-joints --headless
```

---

## Batch Generation

```python
from ballu_morphology_config import create_morphology_variant

morphologies = []
for ratio in [0.35, 0.40, 0.45, 0.50, 0.55, 0.60]:
    morph = create_morphology_variant(
        morphology_id=f"fl_ratio_{ratio:.2f}",
        femur_to_limb_ratio=ratio,
    )
    morphologies.append(morph)

generate_library(morphologies, output_dir="morphology_library")
```

---

## Runtime Loading in Task Configs

```python
from ballu_isaac_extension.ballu_assets.morphology_loader import MorphologyLoaderCfg

@configclass
class HeteroSceneCfg(InteractiveSceneCfg):
    robot = MorphologyLoaderCfg(
        morphology_dir="morphology_library",
        sample_strategy="random",    # or "sequential", "uniform"
        per_env_morphology=True,     # different morphology per env
        prim_path="{ENV_REGEX_NS}/Robot"
    )
```

---

## Morphology Exploration Strategies

### Uniform Sampling

```python
rng = random.Random(42)
designs = [
    {"id": i, "GCR": round(rng.uniform(0.75, 0.89), 6),
               "spcf": round(rng.uniform(0.001, 0.010), 6)}
    for i in range(1000)
]
```

### Grid Search

```python
for femur_ratio in [0.35, 0.40, 0.45, 0.50]:
    for leg_length in [0.70, 0.75, 0.80, 0.85]:
        morph = create_morphology_variant(
            morphology_id=f"fl_{femur_ratio}_len_{leg_length}",
            femur_to_limb_ratio=femur_ratio,
            total_leg_length=leg_length,
        )
```

### CMA-ES Optimisation

```python
from scripts.morphology_utils.explore_morphology_cmaes import optimize_morphology

best_morph = optimize_morphology(
    task="Isaac-Vel-BALLU-1-obstacle",
    initial_params=[0.365, 0.385, 0.159],
    num_iterations=100,
)
```

---

## Heterogeneous Training (Universal Controllers)

Train a single policy across multiple morphologies simultaneously:

```bash
python scripts/rsl_rl/train.py \
    --task Isc-BALLU-hetero-general \
    --num_envs 4096 \
    --max_iterations 5000
```

Include morphology parameters in observations:

```python
@configclass
class HeteroObsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        velocity_commands   = ObsTerm(...)
        joint_pos           = ObsTerm(...)
        morphology_params   = ObsTerm(
            func=mdp.morphology_vector_priv,
            params={"normalize": True}
        )
```

`morphology_vector_priv()` reads GCR/spcf with priority:
`GCR_values` (per-env list) > `GCR_range` > `GCR` (scalar)

---

## Validation Rules

```
femur_length, tibia_length > 0
total_leg_length ∈ [0.1, 2.0]
femur_to_limb_ratio ∈ [0.1, 0.9]
all masses > 0
balloon_mass_ratio ∈ [0.1, 0.9]
lower < upper for all joint limits
initial joint positions ∈ [lower, upper]
```

---

## Best Practices

1. Always validate before generating URDF/USD
2. Use meaningful IDs that encode key parameters (`hetero_0001_fl0.312_tl0.398`)
3. Version-control morphology JSON files (not USD binaries)
4. Test each new morphology in simulation before large-scale training
5. Track performance metrics per morphology to guide exploration

---

## Troubleshooting

| Error | Fix |
|-------|-----|
| `Validation failed — total_leg_length out of range` | Check parameter ranges in `MorphologyParameterRanges` |
| `Cannot generate URDF` | Verify all parameters pass validation |
| `Failed to import URDF` | Check URDF is well-formed; use `--merge-joints` |
| Isaac Sim cannot find USD | Verify `BALLU_USD_REL_PATH` is set and path exists relative to `ballu_isclb_extension/` |

---

## Additional Resources

- Parameter ranges: `scripts/morphology_utils/ballu_morphology_config.py`
- Generation tools: `scripts/morphology_utils/`
- Examples: [examples/morphology_examples.py](examples/morphology_examples.py)
- PEC design-space partitioning: [../training-system/pec_algorithm.md](../training-system/pec_algorithm.md)
