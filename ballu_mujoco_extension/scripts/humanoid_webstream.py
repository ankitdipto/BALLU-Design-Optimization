#!/usr/bin/env python3
"""
Humanoid robot simulation with real-time web streaming.
This script creates a MuJoCo humanoid simulation and streams it via Flask web server.
Perfect for remote SSH access without X11 forwarding.
"""

import os
os.environ['MUJOCO_GL'] = 'egl'  # Use EGL for headless rendering

import numpy as np
import mujoco
import imageio
from pathlib import Path
import threading
import time
from flask import Flask, Response, render_template_string
import cv2
from io import BytesIO
import base64

# Initialize Flask app
app = Flask(__name__)

# Global variables for simulation state
current_frame = None
simulation_running = False
frame_lock = threading.Lock()


# HTML template for the web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>MuJoCo Humanoid Simulation</title>
    <style>
        body {
            margin: 0;
            padding: 20px;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        h1 {
            text-align: center;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .subtitle {
            text-align: center;
            margin-bottom: 30px;
            opacity: 0.9;
            font-size: 1.1em;
        }
        .video-container {
            background: rgba(0, 0, 0, 0.3);
            border-radius: 15px;
            padding: 20px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
            margin-bottom: 20px;
        }
        #stream {
            width: 100%;
            height: auto;
            border-radius: 10px;
            box-shadow: 0 4px 16px rgba(0,0,0,0.4);
        }
        .info-panel {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            padding: 20px;
            backdrop-filter: blur(10px);
            margin-bottom: 20px;
        }
        .info-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        .info-item {
            background: rgba(255, 255, 255, 0.1);
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }
        .info-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-bottom: 5px;
        }
        .info-value {
            font-size: 1.5em;
            font-weight: bold;
        }
        .controls {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            padding: 20px;
            backdrop-filter: blur(10px);
        }
        .button {
            background: rgba(255, 255, 255, 0.2);
            border: 2px solid rgba(255, 255, 255, 0.3);
            color: white;
            padding: 12px 24px;
            border-radius: 8px;
            cursor: pointer;
            font-size: 1em;
            margin: 5px;
            transition: all 0.3s;
        }
        .button:hover {
            background: rgba(255, 255, 255, 0.3);
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.2);
        }
        .status {
            display: inline-block;
            padding: 8px 16px;
            border-radius: 20px;
            background: rgba(76, 175, 80, 0.3);
            border: 2px solid rgba(76, 175, 80, 0.5);
            margin-left: 10px;
        }
        .footer {
            text-align: center;
            margin-top: 30px;
            opacity: 0.7;
            font-size: 0.9em;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 MuJoCo Humanoid Simulation</h1>
        <p class="subtitle">Real-time physics simulation with EGL rendering | Remote SSH Compatible</p>
        
        <div class="video-container">
            <img id="stream" src="{{ url_for('video_feed') }}" alt="Simulation Stream">
        </div>
        
        <div class="info-panel">
            <h2>📊 Simulation Information</h2>
            <div class="info-grid">
                <div class="info-item">
                    <div class="info-label">Status</div>
                    <div class="info-value">
                        <span class="status">● LIVE</span>
                    </div>
                </div>
                <div class="info-item">
                    <div class="info-label">Model</div>
                    <div class="info-value">Humanoid</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Renderer</div>
                    <div class="info-value">EGL</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Frame Rate</div>
                    <div class="info-value">30 FPS</div>
                </div>
            </div>
        </div>
        
        <div class="controls">
            <h2>🎮 Controls</h2>
            <p>The humanoid is performing autonomous movements with physics-based control.</p>
            <button class="button" onclick="location.reload()">🔄 Refresh Stream</button>
            <button class="button" onclick="window.open('/download', '_blank')">💾 Download Recording</button>
        </div>
        
        <div class="footer">
            <p>Powered by MuJoCo Physics Engine | Flask Web Server</p>
            <p>Access this stream from any device on your network</p>
        </div>
    </div>
    
    <script>
        // Auto-refresh every 30 seconds to keep connection alive
        setInterval(function() {
            var img = document.getElementById('stream');
            var src = img.src;
            img.src = '';
            img.src = src;
        }, 30000);
    </script>
</body>
</html>
"""


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
        
        # Control mode
        self.control_mode = "walking"  # walking, standing, squat, dance
        self.time = 0.0
        
        # Recording
        self.frames = []
        self.max_recording_frames = 300  # 10 seconds at 30 FPS
        
        print(f"Humanoid model loaded successfully!")
        print(f"  - Number of bodies: {self.model.nbody}")
        print(f"  - Number of joints: {self.model.njnt}")
        print(f"  - Number of actuators: {self.model.nu}")
        print(f"  - Timestep: {self.model.opt.timestep} s")
        print(f"  - Steps per frame: {self.steps_per_frame}")
        
    def apply_walking_control(self):
        """Apply a simple walking gait pattern."""
        t = self.time
        
        # Simple sinusoidal walking pattern
        freq = 1.0  # Hz
        phase = 2 * np.pi * freq * t
        
        # Get actuator indices (simplified - using first few actuators)
        # In a real implementation, you'd map these to specific joints
        
        # Hip movements (alternating legs)
        if self.model.nu > 6:
            # Right leg
            self.data.ctrl[5] = 0.3 * np.sin(phase)  # hip_y_right
            self.data.ctrl[6] = 0.2 * np.sin(phase + np.pi)  # knee_right
            
            # Left leg
            self.data.ctrl[11] = 0.3 * np.sin(phase + np.pi)  # hip_y_left
            self.data.ctrl[12] = 0.2 * np.sin(phase)  # knee_left
            
            # Arms (counter-balance)
            if self.model.nu > 17:
                self.data.ctrl[17] = 0.2 * np.sin(phase + np.pi)  # right arm
                self.data.ctrl[20] = 0.2 * np.sin(phase)  # left arm
    
    def apply_squat_control(self):
        """Apply squatting motion."""
        t = self.time
        squat_depth = 0.5 * (1 + np.sin(2 * np.pi * 0.3 * t))
        
        if self.model.nu > 12:
            # Bend both knees
            self.data.ctrl[6] = -squat_depth * 0.8  # knee_right
            self.data.ctrl[12] = -squat_depth * 0.8  # knee_left
            
            # Adjust hips
            self.data.ctrl[5] = -squat_depth * 0.5  # hip_y_right
            self.data.ctrl[11] = -squat_depth * 0.5  # hip_y_left
    
    def apply_dance_control(self):
        """Apply a fun dancing motion."""
        t = self.time
        
        # Multiple frequency components for complex motion
        if self.model.nu > 20:
            # Legs
            self.data.ctrl[5] = 0.3 * np.sin(2 * np.pi * 0.5 * t)
            self.data.ctrl[11] = 0.3 * np.sin(2 * np.pi * 0.5 * t + np.pi/2)
            
            # Arms - wave motion
            self.data.ctrl[15] = 0.5 * np.sin(2 * np.pi * 0.8 * t)  # shoulder_right
            self.data.ctrl[16] = 0.5 * np.sin(2 * np.pi * 0.8 * t + np.pi/4)
            self.data.ctrl[18] = 0.5 * np.sin(2 * np.pi * 0.8 * t + np.pi)  # shoulder_left
            self.data.ctrl[19] = 0.5 * np.sin(2 * np.pi * 0.8 * t + np.pi/4)
            
            # Torso twist
            self.data.ctrl[0] = 0.3 * np.sin(2 * np.pi * 0.4 * t)  # abdomen_z
    
    def step(self):
        """Perform one simulation step and return the rendered frame."""
        # Apply control based on mode
        if self.control_mode == "walking":
            self.apply_walking_control()
        elif self.control_mode == "squat":
            self.apply_squat_control()
        elif self.control_mode == "dance":
            self.apply_dance_control()
        else:  # standing
            # Minimal control to maintain balance
            self.data.ctrl[:] = 0.0
        
        # Step simulation
        for _ in range(self.steps_per_frame):
            mujoco.mj_step(self.model, self.data)
        
        # Update time
        self.time += self.frame_duration
        
        # Render frame
        self.renderer.update_scene(self.data, camera="back")
        pixels = self.renderer.render()
        
        # Store frame for recording (circular buffer)
        if len(self.frames) >= self.max_recording_frames:
            self.frames.pop(0)
        self.frames.append(pixels.copy())
        
        return pixels
    
    def save_recording(self, output_path):
        """Save recorded frames as video."""
        if len(self.frames) > 0:
            print(f"Saving recording to {output_path}...")
            imageio.mimsave(output_path, self.frames, fps=self.fps, codec='libx264')
            print(f"Recording saved! ({len(self.frames)} frames)")
            return True
        return False


# Global simulation instance
sim = None


def simulation_thread():
    """Background thread that runs the simulation."""
    global current_frame, simulation_running, sim
    
    # Load humanoid model
    model_path = Path(__file__).parent.parent / 'models' / 'humanoid.xml'
    sim = HumanoidSimulation(str(model_path))
    
    # Cycle through different control modes
    modes = ["walking", "squat", "dance", "standing"]
    mode_duration = 10.0  # seconds per mode
    mode_start_time = time.time()
    current_mode_idx = 0
    
    simulation_running = True
    print("\n🚀 Simulation thread started!")
    print(f"Initial control mode: {sim.control_mode}")
    
    while simulation_running:
        try:
            # Check if it's time to switch modes
            elapsed = time.time() - mode_start_time
            if elapsed > mode_duration:
                current_mode_idx = (current_mode_idx + 1) % len(modes)
                sim.control_mode = modes[current_mode_idx]
                mode_start_time = time.time()
                print(f"\n🔄 Switching to control mode: {sim.control_mode}")
            
            # Step simulation and get frame
            frame = sim.step()
            
            # Update global frame
            with frame_lock:
                current_frame = frame
            
            # Control frame rate
            time.sleep(1.0 / sim.fps)
            
        except Exception as e:
            print(f"Error in simulation thread: {e}")
            import traceback
            traceback.print_exc()
            break
    
    print("Simulation thread stopped.")


def generate_frames():
    """Generator function for video streaming."""
    global current_frame
    
    while True:
        with frame_lock:
            if current_frame is not None:
                frame = current_frame.copy()
            else:
                # Create a placeholder frame
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(frame, "Starting simulation...", (400, 360),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 85])
        frame_bytes = buffer.tobytes()
        
        # Yield frame in multipart format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS


@app.route('/')
def index():
    """Serve the main page."""
    return render_template_string(HTML_TEMPLATE)


@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/download')
def download():
    """Download the recorded video."""
    global sim
    
    if sim is not None:
        output_dir = Path(__file__).parent.parent / 'outputs'
        output_dir.mkdir(exist_ok=True)
        output_path = output_dir / 'humanoid_recording.mp4'
        
        if sim.save_recording(str(output_path)):
            return f"Recording saved to: {output_path}"
        else:
            return "No frames recorded yet."
    else:
        return "Simulation not initialized."


def main():
    """Main function to start the web server and simulation."""
    print("="*70)
    print("🤖 MuJoCo Humanoid Web Streaming Simulation")
    print("="*70)
    print("\nThis server will stream the humanoid simulation to your web browser.")
    print("Perfect for remote SSH access without X11 forwarding!")
    print("\n" + "="*70)
    
    # Start simulation in background thread
    sim_thread = threading.Thread(target=simulation_thread, daemon=True)
    sim_thread.start()
    
    # Give simulation time to initialize
    time.sleep(2)
    
    # Get network information
    import socket
    hostname = socket.gethostname()
    try:
        local_ip = socket.gethostbyname(hostname)
    except:
        local_ip = "localhost"
    
    print("\n✅ Server starting...")
    print(f"\n📡 Access the simulation at:")
    print(f"   • Local:   http://localhost:5000")
    print(f"   • Network: http://{local_ip}:5000")
    print("\n💡 Tips:")
    print("   • Open the URL in any web browser")
    print("   • Works on mobile devices too!")
    print("   • Press Ctrl+C to stop the server")
    print("\n" + "="*70 + "\n")
    
    # Start Flask server
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down...")
        simulation_running = False
        
        # Save final recording
        if sim is not None:
            output_dir = Path(__file__).parent.parent / 'outputs'
            output_dir.mkdir(exist_ok=True)
            output_path = output_dir / 'humanoid_final.mp4'
            sim.save_recording(str(output_path))
        
        print("✅ Server stopped. Goodbye!")


if __name__ == "__main__":
    main()

