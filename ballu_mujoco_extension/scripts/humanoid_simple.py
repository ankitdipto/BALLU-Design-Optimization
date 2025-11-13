#!/usr/bin/env python3
"""
Simple humanoid robot simulation that saves to video file.
This is a simpler alternative to the web streaming version.
"""

import os
os.environ['MUJOCO_GL'] = 'egl'  # Use EGL for headless rendering

import numpy as np
import mujoco
import imageio
from pathlib import Path


class HumanoidSimulation:
    """Manages the humanoid simulation with various control modes."""
    
    def __init__(self, model_path):
        """Initialize the simulation."""
        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Create renderer
        self.renderer = mujoco.Renderer(self.model, height=720, width=1280)
        
        # Reset simulation
        mujoco.mj_resetData(self.model, self.data)
        
        # Simulation parameters
        self.fps = 30
        self.frame_duration = 1.0 / self.fps
        self.steps_per_frame = int(self.frame_duration / self.model.opt.timestep)
        
        # Control parameters
        self.time = 0.0
        
        print(f"✅ Humanoid model loaded successfully!")
        print(f"  - Number of bodies: {self.model.nbody}")
        print(f"  - Number of joints: {self.model.njnt}")
        print(f"  - Number of actuators: {self.model.nu}")
        print(f"  - Timestep: {self.model.opt.timestep} s")
        print(f"  - Steps per frame: {self.steps_per_frame}")
        
        # Print actuator names for reference
        print(f"\n📋 Available actuators:")
        for i in range(self.model.nu):
            actuator_id = i
            actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            print(f"  [{i:2d}] {actuator_name}")
    
    def apply_walking_control(self):
        """Apply a simple walking gait pattern."""
        t = self.time
        
        # Simple sinusoidal walking pattern
        freq = 1.0  # Hz
        phase = 2 * np.pi * freq * t
        
        # Right leg
        self.data.ctrl[5] = 0.3 * np.sin(phase)  # hip_y_right
        self.data.ctrl[6] = 0.2 * np.abs(np.sin(phase))  # knee_right (always positive for walking)
        
        # Left leg (opposite phase)
        self.data.ctrl[11] = 0.3 * np.sin(phase + np.pi)  # hip_y_left
        self.data.ctrl[12] = 0.2 * np.abs(np.sin(phase + np.pi))  # knee_left
        
        # Arms (counter-balance)
        self.data.ctrl[15] = 0.2 * np.sin(phase + np.pi)  # shoulder1_right
        self.data.ctrl[18] = 0.2 * np.sin(phase)  # shoulder1_left
        
        # Keep torso stable
        self.data.ctrl[0] = 0.0  # abdomen_z
        self.data.ctrl[1] = 0.0  # abdomen_y
        self.data.ctrl[2] = 0.0  # abdomen_x
    
    def apply_squat_control(self):
        """Apply squatting motion."""
        t = self.time
        squat_depth = 0.5 * (1 + np.sin(2 * np.pi * 0.3 * t))
        
        # Bend both knees
        self.data.ctrl[6] = -squat_depth * 0.8  # knee_right
        self.data.ctrl[12] = -squat_depth * 0.8  # knee_left
        
        # Adjust hips
        self.data.ctrl[5] = -squat_depth * 0.5  # hip_y_right
        self.data.ctrl[11] = -squat_depth * 0.5  # hip_y_left
        
        # Lean forward slightly
        self.data.ctrl[1] = -squat_depth * 0.3  # abdomen_y
    
    def apply_dance_control(self):
        """Apply a fun dancing motion."""
        t = self.time
        
        # Legs - side to side
        self.data.ctrl[3] = 0.3 * np.sin(2 * np.pi * 0.5 * t)  # hip_x_right
        self.data.ctrl[9] = 0.3 * np.sin(2 * np.pi * 0.5 * t)  # hip_x_left
        
        # Arms - wave motion
        self.data.ctrl[15] = 0.5 * np.sin(2 * np.pi * 0.8 * t)  # shoulder1_right
        self.data.ctrl[16] = 0.5 * np.sin(2 * np.pi * 0.8 * t + np.pi/4)  # shoulder2_right
        self.data.ctrl[18] = 0.5 * np.sin(2 * np.pi * 0.8 * t + np.pi)  # shoulder1_left
        self.data.ctrl[19] = 0.5 * np.sin(2 * np.pi * 0.8 * t + 3*np.pi/4)  # shoulder2_left
        
        # Torso twist
        self.data.ctrl[0] = 0.3 * np.sin(2 * np.pi * 0.4 * t)  # abdomen_z
    
    def apply_balance_control(self):
        """Simple balance control to keep humanoid upright."""
        # Get torso orientation
        torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        
        # Simple PD control on joints to maintain upright posture
        for i in range(self.model.nu):
            # Small damping to prevent falling
            self.data.ctrl[i] = -0.1 * self.data.qvel[i + 6]  # Skip free joint (first 6 DOF)
    
    def step(self, control_mode="walking"):
        """Perform one simulation step and return the rendered frame."""
        # Apply control based on mode
        if control_mode == "walking":
            self.apply_walking_control()
        elif control_mode == "squat":
            self.apply_squat_control()
        elif control_mode == "dance":
            self.apply_dance_control()
        elif control_mode == "balance":
            self.apply_balance_control()
        else:  # standing
            self.data.ctrl[:] = 0.0
        
        # Step simulation
        for _ in range(self.steps_per_frame):
            mujoco.mj_step(self.model, self.data)
        
        # Update time
        self.time += self.frame_duration
        
        # Render frame
        self.renderer.update_scene(self.data, camera="back")
        pixels = self.renderer.render()
        
        return pixels


def run_simulation(duration=30.0, control_mode="walking", output_name=None):
    """
    Run humanoid simulation and save as video.
    
    Args:
        duration: Simulation duration in seconds
        control_mode: Control mode (walking, squat, dance, balance, standing)
        output_name: Output filename (default: humanoid_{mode}.mp4)
    """
    # Load model
    model_path = Path(__file__).parent.parent / 'models' / 'humanoid.xml'
    sim = HumanoidSimulation(str(model_path))
    
    # Calculate frames
    n_frames = int(duration * sim.fps)
    
    print(f"\n{'='*70}")
    print(f"🎬 Running simulation: {control_mode.upper()} mode")
    print(f"{'='*70}")
    print(f"  Duration: {duration} s")
    print(f"  FPS: {sim.fps}")
    print(f"  Total frames: {n_frames}")
    print(f"{'='*70}\n")
    
    # Run simulation
    frames = []
    for i in range(n_frames):
        frame = sim.step(control_mode=control_mode)
        frames.append(frame)
        
        if (i + 1) % 30 == 0 or i == 0:
            # Print status every second
            torso_height = sim.data.qpos[2]  # Z position of root
            print(f"  Frame {i+1:4d}/{n_frames} | Time: {sim.time:5.1f}s | Torso height: {torso_height:.3f}m")
    
    # Save video
    output_dir = Path(__file__).parent.parent / 'outputs'
    output_dir.mkdir(exist_ok=True)
    
    if output_name is None:
        output_name = f'humanoid_{control_mode}.mp4'
    output_path = output_dir / output_name
    
    print(f"\n💾 Saving video to: {output_path}")
    imageio.mimsave(output_path, frames, fps=sim.fps, codec='libx264')
    
    print(f"\n{'='*70}")
    print("✅ Simulation complete!")
    print(f"{'='*70}")
    print(f"📹 Video: {output_path}")
    print(f"📊 Size: {output_path.stat().st_size / 1024 / 1024:.1f} MB")
    print(f"⏱️  Duration: {duration:.1f} seconds")
    print(f"🎞️  Frames: {len(frames)}")
    print(f"{'='*70}\n")


def run_multi_mode_simulation(duration_per_mode=10.0):
    """Run simulation cycling through different control modes."""
    modes = [
        ("balance", "Standing Balance"),
        ("walking", "Walking Gait"),
        ("squat", "Squatting"),
        ("dance", "Dancing")
    ]
    
    print("="*70)
    print("🤖 MuJoCo Humanoid Multi-Mode Simulation")
    print("="*70)
    print(f"\nThis will create {len(modes)} videos demonstrating different control modes.")
    print(f"Each video will be {duration_per_mode} seconds long.\n")
    
    for mode, description in modes:
        print(f"\n🎯 Mode: {description}")
        run_simulation(duration=duration_per_mode, control_mode=mode)
    
    print("\n" + "="*70)
    print("🎉 All simulations complete!")
    print("="*70)
    print(f"\nCheck the outputs/ directory for {len(modes)} video files.")
    print("You can download them via SCP or view them locally.\n")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        # Run specific mode
        mode = sys.argv[1]
        duration = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
        
        valid_modes = ["walking", "squat", "dance", "balance", "standing"]
        if mode not in valid_modes:
            print(f"❌ Invalid mode: {mode}")
            print(f"Valid modes: {', '.join(valid_modes)}")
            sys.exit(1)
        
        run_simulation(duration=duration, control_mode=mode)
    else:
        # Run all modes
        run_multi_mode_simulation(duration_per_mode=10.0)

