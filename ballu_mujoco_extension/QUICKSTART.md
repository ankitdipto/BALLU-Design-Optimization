# MuJoCo Project - Quick Start Guide

## Installation

### Option 1: Using the Setup Script

```bash
cd mujoco_project
./setup_env.sh
```

### Option 2: Manual Installation

```bash
# Create conda environment
conda create -n mujoco_v0 python=3.10 -y
conda activate mujoco_v0

# Install dependencies
pip install -r requirements.txt
```

## Running Examples

### Test Installation

This runs a simple cartpole simulation to verify everything is working:

```bash
cd scripts
conda activate mujoco_v0
python test_mujoco.py
```

**Output:** `outputs/test_simulation.mp4` (5 seconds, cartpole with oscillating control)

### Pendulum Example

This demonstrates a more complex simulation with control logic:

```bash
cd scripts
conda activate mujoco_v0
python simple_example.py
```

**Outputs:**
- `outputs/pendulum_controlled.mp4` (8 seconds, swing-up controller)
- `outputs/pendulum_free.mp4` (5 seconds, free swing from bottom)

## Key Concepts

### 1. Headless Rendering

For remote servers without displays, set this **before** importing mujoco:

```python
import os
os.environ['MUJOCO_GL'] = 'egl'
import mujoco
```

### 2. Loading Models

From XML file:
```python
model = mujoco.MjModel.from_xml_path('path/to/model.xml')
data = mujoco.MjData(model)
```

From XML string:
```python
xml_string = "<mujoco>...</mujoco>"
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)
```

### 3. Running Simulation

```python
# Reset to initial state
mujoco.mj_resetData(model, data)

# Set initial conditions
data.qpos[0] = 1.0  # Position
data.qvel[0] = 0.0  # Velocity

# Apply controls
data.ctrl[0] = 0.5

# Step simulation forward
mujoco.mj_step(model, data)
```

### 4. Rendering and Recording

```python
# Create renderer
renderer = mujoco.Renderer(model, height=480, width=640)

frames = []
for i in range(num_frames):
    # ... simulation steps ...
    
    # Render frame
    renderer.update_scene(data, camera="camera_name")
    pixels = renderer.render()
    frames.append(pixels.copy())

# Save video
import imageio
imageio.mimsave('output.mp4', frames, fps=30, codec='libx264')
```

## Creating Your Own Model

### Basic Structure

```xml
<mujoco model="my_robot">
  <!-- Simulation settings -->
  <option timestep="0.01" gravity="0 0 -9.81"/>
  
  <!-- Visual settings -->
  <visual>
    <global offwidth="640" offheight="480"/>
  </visual>
  
  <!-- Materials and textures -->
  <asset>
    <texture name="grid" type="2d" builtin="checker"/>
    <material name="grid" texture="grid"/>
  </asset>
  
  <!-- Physical objects -->
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1" material="grid"/>
    
    <!-- Your robot here -->
    <body name="robot" pos="0 0 1">
      <joint name="joint1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
    </body>
    
    <!-- Camera for rendering -->
    <camera name="main" pos="2 -2 1.5"/>
  </worldbody>
  
  <!-- Actuators -->
  <actuator>
    <motor joint="joint1" name="motor1"/>
  </actuator>
</mujoco>
```

### Common Geometry Types

- `box`: `size="width height depth"`
- `sphere`: `size="radius"`
- `cylinder`: `size="radius height"`
- `capsule`: `fromto="x1 y1 z1 x2 y2 z2" size="radius"`
- `plane`: `size="x y z"`

### Joint Types

- `free`: 6 DOF (3 translation + 3 rotation)
- `ball`: 3 DOF (rotation only)
- `slide`: 1 DOF (linear)
- `hinge`: 1 DOF (rotational)

## Debugging Tips

### Check Model Statistics

```python
print(f"Number of bodies: {model.nbody}")
print(f"Number of joints: {model.njnt}")
print(f"Number of DOFs: {model.nv}")
print(f"Number of actuators: {model.nu}")
print(f"Timestep: {model.opt.timestep}")
```

### Print State

```python
print(f"Positions: {data.qpos}")
print(f"Velocities: {data.qvel}")
print(f"Controls: {data.ctrl}")
print(f"Time: {data.time}")
```

### Visualize in Real-Time (with display)

```python
# Only works if DISPLAY is available
import mujoco_python_viewer

viewer = mujoco_python_viewer.MujocoViewer(model, data)
for _ in range(1000):
    mujoco.mj_step(model, data)
    viewer.render()
viewer.close()
```

## Performance Tips

1. **Adjust timestep**: Larger timestep = faster simulation (but less accurate)
   ```xml
   <option timestep="0.02"/>  <!-- Default is 0.002 -->
   ```

2. **Reduce render frequency**: Don't render every simulation step
   ```python
   steps_per_frame = 10
   for _ in range(steps_per_frame):
       mujoco.mj_step(model, data)
   # Then render once
   ```

3. **Lower resolution**: Use smaller render resolution
   ```python
   renderer = mujoco.Renderer(model, height=240, width=320)
   ```

4. **Disable shadows**: For faster rendering
   ```xml
   <visual>
     <quality shadowsize="0"/>
   </visual>
   ```

## Common Issues

**Error: "DISPLAY environment variable is missing"**
- Solution: Set `os.environ['MUJOCO_GL'] = 'egl'` before importing mujoco

**Error: "Camera does not exist"**
- Solution: Check camera name in XML, or use `camera=-1` for default

**Error: "OpenGL context not created"**
- Solution: Install mesa libraries: `sudo apt-get install libgl1-mesa-glx libegl1-mesa`

**Video file is corrupt**
- Solution: Ensure imageio-ffmpeg is installed: `pip install imageio-ffmpeg`

## Resources

- **Documentation**: https://mujoco.readthedocs.io/
- **Examples**: https://github.com/google-deepmind/mujoco/tree/main/python/mujoco/examples
- **Forum**: https://github.com/google-deepmind/mujoco/discussions
- **Model Zoo**: https://github.com/google-deepmind/mujoco_menagerie

## Next Steps

1. Modify the cartpole or pendulum models
2. Create your own robot model
3. Implement a reinforcement learning controller
4. Integrate with ML frameworks (PyTorch, JAX)
5. Explore MuJoCo's contact dynamics and friction models

