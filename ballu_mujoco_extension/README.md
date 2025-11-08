# MuJoCo Project

This project demonstrates MuJoCo physics simulation with offscreen rendering for headless servers.

## Environment Setup

### Conda Environment

The project uses a dedicated conda environment named `mujoco_v0`:

```bash
conda activate mujoco_v0
```

### Installed Packages

- **mujoco** (3.3.7): Main MuJoCo physics engine with Python bindings
- **mujoco-python-viewer**: Interactive viewer for MuJoCo simulations
- **numpy**: Numerical computing
- **imageio**: Image and video I/O
- **imageio-ffmpeg**: FFmpeg backend for video encoding
- **PyOpenGL**: OpenGL bindings for Python
- **PyOpenGL-accelerate**: Accelerated OpenGL operations

## Directory Structure

```
mujoco_project/
├── models/           # MuJoCo XML model files
│   └── cartpole.xml # Simple cartpole model for testing
├── scripts/          # Python scripts
│   └── test_mujoco.py # Test script with video recording
├── outputs/          # Generated videos and results
└── README.md         # This file
```

## Running the Test

The test script demonstrates:
- Loading a MuJoCo model from XML
- Running a physics simulation
- Offscreen rendering using EGL (for headless servers)
- Saving simulation as MP4 video

```bash
cd mujoco_project/scripts
conda activate mujoco_v0
python test_mujoco.py
```

### Test Output

The script creates a 5-second video at 30 FPS showing a cartpole system with sinusoidal control input. The video is saved to `outputs/test_simulation.mp4`.

## Key Features

### Headless Rendering

The project uses EGL for offscreen rendering, which works on remote servers without a display:

```python
import os
os.environ['MUJOCO_GL'] = 'egl'  # Set before importing mujoco
import mujoco
```

### Video Recording

Simulations are recorded frame-by-frame and saved as MP4 videos:

```python
renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data, camera="tracking")
pixels = renderer.render()
frames.append(pixels.copy())

# Save video
imageio.mimsave(output_path, frames, fps=fps, codec='libx264')
```

## Model: Cartpole

The included cartpole model features:
- A sliding cart on a rail (1D motion)
- A pole attached via a hinge joint
- Single motor actuator controlling cart position
- Two cameras: "fixed" and "tracking" (center-of-mass tracking)

## Creating Your Own Models

MuJoCo models are defined in XML format. Key elements:

1. **worldbody**: Contains all physical objects
2. **body**: Rigid body with mass and inertia
3. **geom**: Visual and collision geometry
4. **joint**: Degrees of freedom between bodies
5. **actuator**: Motors, cylinders, or muscles
6. **camera**: Viewpoints for rendering

Example:

```xml
<mujoco model="my_model">
  <worldbody>
    <body name="object" pos="0 0 1">
      <joint name="free" type="free"/>
      <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
```

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo GitHub](https://github.com/google-deepmind/mujoco)
- [Python Bindings Documentation](https://mujoco.readthedocs.io/en/stable/python.html)
- [XML Model Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)

## Troubleshooting

### Display Errors

If you see "DISPLAY environment variable is missing":
- Ensure `MUJOCO_GL='egl'` is set **before** importing mujoco
- Install mesa-utils: `sudo apt-get install mesa-utils`

### Video Encoding Errors

If video saving fails:
- Check that imageio-ffmpeg is installed: `pip install imageio-ffmpeg`
- Verify ffmpeg is available: `ffmpeg -version`

### OpenGL Errors

If you encounter OpenGL initialization errors:
- Try installing system OpenGL libraries: `sudo apt-get install libgl1-mesa-glx libegl1-mesa`
- For better performance, install: `pip install PyOpenGL-accelerate`

## Next Steps

1. Create custom models in `models/` directory
2. Develop control policies and reinforcement learning algorithms
3. Integrate with ML frameworks (PyTorch, JAX, TensorFlow)
4. Experiment with different camera angles and rendering options
5. Optimize simulation parameters for your use case

