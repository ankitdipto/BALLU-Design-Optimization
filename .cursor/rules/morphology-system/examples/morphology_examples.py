"""Example morphology usage patterns for BALLU."""

from ballu_morphology_config import (
    BalluMorphology,
    GeometryParams,
    MassParams,
    create_morphology_variant,
    MorphologyParameterRanges,
)


def example_default_morphology():
    """Create default morphology from original URDF."""
    morph = BalluMorphology.default()
    print(morph)
    
    # Validate
    is_valid, errors = morph.validate()
    if is_valid:
        print("✓ Valid morphology")
    else:
        print("✗ Validation errors:", errors)
    
    # Get derived properties
    props = morph.get_derived_properties()
    print(f"Total leg length: {props['total_leg_length']:.3f}m")
    print(f"Femur-to-limb ratio: {props['femur_to_limb_ratio']:.3f}")


def example_custom_morphology():
    """Create custom morphology with specific parameters."""
    morph = BalluMorphology(
        morphology_id="long_legs_v1",
        description="BALLU with longer legs for obstacles",
        geometry=GeometryParams(
            femur_length=0.45,
            tibia_length=0.45,
        ),
        mass=MassParams(
            balloon_mass=0.20,  # More buoyancy
        )
    )
    
    # Validate
    is_valid, errors = morph.validate()
    print(f"Valid: {is_valid}")
    
    return morph


def example_morphology_variants():
    """Create morphology variants using convenience function."""
    
    # By femur-to-limb ratio
    morph1 = create_morphology_variant(
        morphology_id="fl_ratio_0.40",
        femur_to_limb_ratio=0.40,
    )
    
    # By total leg length
    morph2 = create_morphology_variant(
        morphology_id="long_legs_0.9m",
        total_leg_length=0.9,
    )
    
    # By balloon size
    morph3 = create_morphology_variant(
        morphology_id="big_balloon",
        balloon_radius=0.4,
        balloon_height=0.9,
        balloon_mass=0.25,
    )
    
    return [morph1, morph2, morph3]


def example_save_load():
    """Save and load morphologies."""
    # Create
    morph = create_morphology_variant(
        morphology_id="test_morph",
        total_leg_length=0.85,
    )
    
    # Save to JSON
    morph.to_json("test_morphology.json")
    
    # Load from JSON
    loaded_morph = BalluMorphology.from_json("test_morphology.json")
    
    print(f"Loaded: {loaded_morph.morphology_id}")


def example_parameter_ranges():
    """Work with parameter ranges."""
    ranges = MorphologyParameterRanges()
    
    # View ranges
    print(f"Femur length range: {ranges.femur_length}")  # (min, max, default)
    
    # Sample random values
    femur_len = ranges.sample_uniform("femur_length")
    tibia_len = ranges.sample_uniform("tibia_length")
    
    # Create morphology from sampled params
    morph = create_morphology_variant(
        morphology_id="random_sample",
        femur_length=femur_len,
        tibia_length=tibia_len,
    )
    
    return morph


def example_batch_generation():
    """Generate batch of morphologies."""
    morphologies = []
    
    for ratio in [0.35, 0.40, 0.45, 0.50, 0.55, 0.60]:
        morph = create_morphology_variant(
            morphology_id=f"fl_ratio_{ratio:.2f}",
            femur_to_limb_ratio=ratio,
        )
        morphologies.append(morph)
    
    # Save all
    for morph in morphologies:
        morph.to_json(f"morphologies/{morph.morphology_id}.json")
    
    print(f"Generated {len(morphologies)} morphologies")
    return morphologies


def example_task_specific():
    """Create task-specific morphologies."""
    
    # High obstacle clearance
    obstacle_morph = create_morphology_variant(
        morphology_id="high_obstacles",
        total_leg_length=0.9,
        femur_to_limb_ratio=0.55,
        balloon_mass=0.22,
    )
    
    # Speed optimization
    speed_morph = create_morphology_variant(
        morphology_id="speed_optimized",
        total_leg_length=0.8,
        limb_radius=0.003,
        tibia_mass=0.03,
    )
    
    # Stability focused
    stable_morph = create_morphology_variant(
        morphology_id="stable",
        hip_width=0.14,
        balloon_mass=0.20,
    )
    
    return [obstacle_morph, speed_morph, stable_morph]


def example_exploration_grid():
    """Grid search over parameter space."""
    morphologies = []
    
    femur_ratios = [0.35, 0.40, 0.45, 0.50, 0.55]
    leg_lengths = [0.7, 0.75, 0.8, 0.85, 0.9]
    
    for fr in femur_ratios:
        for ll in leg_lengths:
            morph = create_morphology_variant(
                morphology_id=f"fl_{fr:.2f}_len_{ll:.2f}",
                femur_to_limb_ratio=fr,
                total_leg_length=ll,
            )
            
            # Validate before adding
            if morph.validate()[0]:
                morphologies.append(morph)
    
    print(f"Generated {len(morphologies)} valid morphologies")
    return morphologies


if __name__ == "__main__":
    # Run examples
    print("=== Default Morphology ===")
    example_default_morphology()
    
    print("\n=== Custom Morphology ===")
    example_custom_morphology()
    
    print("\n=== Morphology Variants ===")
    example_morphology_variants()
    
    print("\n=== Batch Generation ===")
    example_batch_generation()
    
    print("\n=== Task-Specific ===")
    example_task_specific()
