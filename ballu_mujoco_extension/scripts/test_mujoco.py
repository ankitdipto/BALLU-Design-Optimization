#!/usr/bin/env python3
"""
Test script for MuJoCo installation.
This script loads a cartpole model, runs a simulation with offscreen rendering,
and saves the result as a video file.
"""

import os
os.environ['MUJOCO_GL'] = 'egl'  # Use EGL for headless rendering

import numpy as np
import mujoco
import imageio

def test_mujoco_installation():
    """Test MuJoCo installation by running a simple simulation and saving a video."""
    
    # Get the project root directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Load the model
    model_path = os.path.join(project_root, 'models', 'cartpole.xml')
    print(f"Loading model from: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Create renderer for offscreen rendering
    renderer = mujoco.Renderer(model, height=480, width=640)
    
    # Simulation parameters
    duration = 5.0  # seconds
    fps = 30
    frames = []
    
    # Calculate timesteps
    frame_duration = 1.0 / fps
    n_frames = int(duration * fps)
    steps_per_frame = int(frame_duration / model.opt.timestep)
    
    print(f"Running simulation for {duration} seconds at {fps} FPS...")
    print(f"Total frames: {n_frames}")
    print(f"Steps per frame: {steps_per_frame}")
    
    # Reset simulation
    mujoco.mj_resetData(model, data)
    
    # Apply a simple control policy (oscillating force)
    for i in range(n_frames):
        # Simple sinusoidal control to make the cart move
        t = i * frame_duration
        data.ctrl[0] = np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz oscillation
        
        # Step simulation
        for _ in range(steps_per_frame):
            mujoco.mj_step(model, data)
        
        # Update renderer and capture frame
        renderer.update_scene(data, camera="tracking")
        pixels = renderer.render()
        frames.append(pixels.copy())
        
        if (i + 1) % 30 == 0:
            print(f"  Rendered frame {i+1}/{n_frames}")
    
    # Save video
    output_dir = os.path.join(project_root, 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'test_simulation.mp4')
    
    print(f"\nSaving video to: {output_path}")
    imageio.mimsave(output_path, frames, fps=fps, codec='libx264')
    
    print(f"\n{'='*60}")
    print("SUCCESS! MuJoCo installation test completed.")
    print(f"{'='*60}")
    print(f"Video saved to: {output_path}")
    print(f"Video duration: {duration} seconds")
    print(f"Video resolution: 640x480")
    print(f"Frame rate: {fps} FPS")
    print(f"\nSimulation details:")
    print(f"  Model timestep: {model.opt.timestep} s")
    print(f"  Total simulation steps: {n_frames * steps_per_frame}")
    print(f"  Cart position (final): {data.qpos[0]:.3f} m")
    print(f"  Pole angle (final): {data.qpos[1]:.3f} rad ({np.rad2deg(data.qpos[1]):.1f} deg)")
    print(f"{'='*60}")

if __name__ == "__main__":
    test_mujoco_installation()

