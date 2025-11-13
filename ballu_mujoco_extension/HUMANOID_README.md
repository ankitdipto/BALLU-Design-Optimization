# 🤖 Humanoid Robot Simulation with MuJoCo

A comprehensive MuJoCo-based humanoid simulation with web streaming capabilities for remote SSH access. This project provides real-time visualization and video export of physics-based humanoid control.

## ⚡ Quick Start (2 Minutes)

### Option 1: Interactive Menu (Easiest)

```bash
cd /home/asinha389/shared/BALLU_Project/ballu_mujoco_extension/scripts
./run_humanoid.sh
```

### Option 2: Web Streaming (Best for SSH)

```bash
cd /home/asinha389/shared/BALLU_Project/ballu_mujoco_extension/scripts
python humanoid_webstream.py
# Open http://YOUR_SERVER_IP:5000 in any browser
```

### Option 3: Quick Video Export

```bash
cd /home/asinha389/shared/BALLU_Project/ballu_mujoco_extension/scripts
python humanoid_simple.py          # All modes (10s each)
python humanoid_simple.py walking 30  # Specific mode (30s)
```

## 📦 Installation

### First Time Setup

```bash
# 1. Activate environment
conda activate mujoco_v0

# 2. Install dependencies
cd /home/asinha389/shared/BALLU_Project/ballu_mujoco_extension
pip install -r requirements.txt

# 3. Verify installation
python -c "import mujoco, flask, cv2; print('✅ All dependencies OK!')"
```

### Requirements

**Core:**
- Python 3.10+
- mujoco >= 3.3.7
- numpy >= 1.24.0
- imageio >= 2.30.0

**Web Streaming:**
- flask >= 3.0.0
- opencv-python >= 4.8.0

## 📁 Project Structure

```
ballu_mujoco_extension/
├── models/
│   └── humanoid.xml              # MuJoCo humanoid model (21 actuators, 27 DOF)
├── scripts/
│   ├── humanoid_webstream.py     # Real-time web streaming
│   ├── humanoid_simple.py        # Video export
│   └── run_humanoid.sh           # Interactive launcher
├── outputs/                       # Generated videos
└── HUMANOID_README.md            # This file
```

## 🎮 Control Modes

The simulation supports 5 different control modes:

| Mode | Description | Stability |
|------|-------------|-----------|
| **Balance** 🧍 | PD control to maintain upright posture | Most stable |
| **Walking** 🚶 | Sinusoidal gait with alternating legs & arm swings | Moderate |
| **Squat** 🏋️ | Periodic squatting with synchronized knee bending | Moderate |
| **Dance** 💃 | Complex multi-frequency movements (legs, arms, torso) | Dynamic |
| **Standing** 🗿 | Minimal actuation, passive pose | Stable |

**Usage:**
```bash
python humanoid_simple.py balance 20   # 20 seconds of balance mode
python humanoid_simple.py walking 30   # 30 seconds of walking
python humanoid_simple.py dance 15     # 15 seconds of dancing
```

## 🌟 Features

### Web Streaming
- ✅ Real-time 30 FPS MJPEG stream
- ✅ Beautiful responsive web interface
- ✅ Works on desktop, tablet, and mobile
- ✅ No X11 forwarding needed (EGL rendering)
- ✅ Download recordings directly from browser
- ✅ Automatic mode switching every 10 seconds
- ✅ Network accessible from any device

### Video Export
- ✅ High-quality MP4 output (H.264 codec)
- ✅ Customizable duration and resolution
- ✅ Multiple camera angles (back, side, egocentric)
- ✅ Batch processing of multiple modes
- ✅ Frame-by-frame control

### Technical Specifications
- **Rendering**: EGL (headless, GPU-accelerated)
- **Resolution**: 1280x720 (720p, configurable)
- **Frame Rate**: 30 FPS (configurable)
- **Physics**: 0.005s timestep (200 Hz)
- **Model**: 17 bodies, 22 joints, 21 actuators
- **Latency**: < 100ms for web streaming

## 🌐 Remote Access Methods

### Method 1: SSH Port Forwarding (Recommended)

On your **local machine**:
```bash
ssh -L 5000:localhost:5000 user@remote_server
```

Run web streaming on server, then access `http://localhost:5000` locally.

### Method 2: Direct Network Access

On the **server**:
```bash
hostname -I  # Get server IP
python humanoid_webstream.py
```

Access from any device on the network: `http://SERVER_IP:5000`

### Method 3: Download Videos via SCP

```bash
# On your local machine
scp user@server:/home/asinha389/shared/BALLU_Project/ballu_mujoco_extension/outputs/*.mp4 ./
```

## 🔧 Customization

### Change Simulation Duration

```bash
python humanoid_simple.py walking 60  # 60 seconds
```

### Modify Control Parameters

Edit `humanoid_simple.py` or `humanoid_webstream.py`:

```python
def apply_walking_control(self):
    freq = 1.0  # Adjust walking frequency (Hz)
    self.data.ctrl[5] = 0.3 * np.sin(phase)  # Adjust amplitude
```

### Change Camera View

```python
# Available cameras: "back", "side", "egocentric"
self.renderer.update_scene(self.data, camera="side")
```

### Adjust Video Quality

```python
# Higher resolution
self.renderer = mujoco.Renderer(self.model, height=1080, width=1920)

# Higher quality encoding
imageio.mimsave(output_path, frames, fps=60, quality=10)
```

### Change Web Server Port

```python
# In humanoid_webstream.py
app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
```

### Modify Mode Duration (Web Streaming)

```python
# In humanoid_webstream.py
mode_duration = 15.0  # seconds per mode (default: 10.0)
```

## 📊 Model Details

### Actuator Mapping

| Index | Actuator | Joint | Function |
|-------|----------|-------|----------|
| 0-2 | abdomen_z/y/x | Torso | Bending/twisting |
| 3-8 | hip/knee/ankle_right | Right leg | Leg movement |
| 9-14 | hip/knee/ankle_left | Left leg | Leg movement |
| 15-17 | shoulder/elbow_right | Right arm | Arm movement |
| 18-20 | shoulder/elbow_left | Left arm | Arm movement |

### Model Features
- Full humanoid with torso, head, arms, and legs
- Realistic joint limits and damping
- Tendon constraints (hamstrings connecting hip and knee)
- Multiple camera views for different perspectives
- Contact exclusions to prevent self-collision

## 🐛 Troubleshooting

### EGL Errors

```bash
# Install EGL libraries
sudo apt-get install libegl1-mesa libegl1-mesa-dev

# Verify EGL is available
python -c "import os; os.environ['MUJOCO_GL']='egl'; import mujoco; print('EGL OK')"
```

### Flask Not Found

```bash
pip install flask opencv-python
```

### Port 5000 Already in Use

Edit `humanoid_webstream.py` and change the port:
```python
app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
```

### Video Encoding Errors

```bash
# Install ffmpeg
sudo apt-get install ffmpeg
pip install imageio-ffmpeg

# Verify
ffmpeg -version
```

### Humanoid Falls Over

This is normal behavior with simple open-loop controllers. Try:
```bash
python humanoid_simple.py balance 20  # Most stable mode
```

To improve stability:
- Reduce control amplitudes in the control functions
- Increase damping in `models/humanoid.xml`
- Implement feedback control (PD controllers)

### Memory Issues

For long simulations, limit the recording buffer:
```python
# In humanoid_webstream.py
self.max_recording_frames = 150  # 5 seconds at 30 FPS
```

### MUJOCO_GL Error

```bash
export MUJOCO_GL=egl
python humanoid_simple.py
```

## 🎓 Advanced Usage

### Custom Control Trajectories

```python
from humanoid_simple import HumanoidSimulation
import numpy as np

# Create simulation
sim = HumanoidSimulation('models/humanoid.xml')
frames = []

# Custom control loop
for t in np.linspace(0, 10, 300):  # 10 seconds, 30 FPS
    # Your custom control logic
    sim.data.ctrl[5] = 0.5 * np.sin(2 * np.pi * t)
    frame = sim.step(control_mode="standing")
    frames.append(frame)

# Save video
import imageio
imageio.mimsave('custom_trajectory.mp4', frames, fps=30)
```

### Multi-Camera Recording

```python
cameras = ["back", "side", "egocentric"]
all_frames = {cam: [] for cam in cameras}

for i in range(n_frames):
    sim.step()
    for cam in cameras:
        sim.renderer.update_scene(sim.data, camera=cam)
        all_frames[cam].append(sim.renderer.render())

# Save each view
for cam, frames in all_frames.items():
    imageio.mimsave(f'humanoid_{cam}.mp4', frames, fps=30)
```

### Integration with Reinforcement Learning

```python
class HumanoidEnv:
    def __init__(self):
        self.sim = HumanoidSimulation(model_path)
    
    def step(self, action):
        self.sim.data.ctrl[:] = action
        frame = self.sim.step()
        
        # Compute reward
        height = self.sim.data.qpos[2]
        velocity = self.sim.data.qvel[0]
        reward = height + 0.5 * velocity
        
        # Check termination
        done = height < 0.5
        
        return self._get_obs(), reward, done, {}
    
    def _get_obs(self):
        return np.concatenate([
            self.sim.data.qpos,
            self.sim.data.qvel
        ])
```

## 📈 Performance Tips

1. **Lower Resolution**: Use 640x480 for faster rendering
   ```python
   self.renderer = mujoco.Renderer(self.model, height=480, width=640)
   ```

2. **Reduce FPS**: 15-20 FPS is often sufficient
   ```python
   self.fps = 20
   ```

3. **Increase Timestep**: Less accurate but faster (use cautiously)
   ```xml
   <!-- In humanoid.xml -->
   <option timestep="0.01"/>
   ```

4. **Disable Recording**: Comment out frame storage for web streaming
   ```python
   # self.frames.append(pixels.copy())
   ```

## 📝 Control Strategies

The current implementation uses **feedforward control** (open-loop sinusoidal patterns). Other strategies include:

1. **Feedback Control**: PD controllers using joint positions/velocities
2. **Trajectory Optimization**: Pre-computed optimal trajectories
3. **Model Predictive Control**: Real-time optimization
4. **Reinforcement Learning**: Learned policies (PPO, SAC, etc.)

## 🎯 Use Cases

- **Research**: Test control algorithms, validate theories
- **Education**: Teach robotics, physics simulation
- **Development**: Debug motion planning, visualize behavior
- **Demonstrations**: Create professional videos, live demos
- **Remote Work**: Access simulations over SSH without GUI

## 📞 Support & Resources

- **MuJoCo Documentation**: https://mujoco.readthedocs.io/
- **Python Bindings**: https://mujoco.readthedocs.io/en/stable/python.html
- **XML Reference**: https://mujoco.readthedocs.io/en/stable/XMLreference.html
- **BALLU Project**: See main project README

## 🤝 Contributing

Contributions welcome! Areas of interest:
- Better control algorithms (feedback, optimal control)
- Additional visualization features
- Performance optimizations
- Interactive web controls
- Multi-robot simulations
- Bug fixes and documentation

## 📝 Citation

If you use this code in your research, please cite MuJoCo:

```bibtex
@inproceedings{todorov2012mujoco,
  title={MuJoCo: A physics engine for model-based control},
  author={Todorov, Emanuel and Erez, Tom and Tassa, Yuval},
  booktitle={2012 IEEE/RSJ International Conference on Intelligent Robots and Systems},
  pages={5026--5033},
  year={2012},
  organization={IEEE}
}
```

## 📊 Output Files

All videos are saved to:
```
/home/asinha389/shared/BALLU_Project/ballu_mujoco_extension/outputs/
```

Generated files:
- `humanoid_balance.mp4` - Balance mode
- `humanoid_walking.mp4` - Walking mode
- `humanoid_squat.mp4` - Squat mode
- `humanoid_dance.mp4` - Dance mode
- `humanoid_recording.mp4` - Web interface recording
- `humanoid_final.mp4` - Final recording on server shutdown

## 🙏 Acknowledgments

- MuJoCo physics engine by DeepMind
- Humanoid model from MuJoCo model zoo
- Flask web framework
- OpenCV for image processing

---

**Happy Simulating! 🚀**

*Made with ❤️ for the BALLU Project | Robotics Research & Education*

