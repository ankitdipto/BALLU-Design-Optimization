# BALLU Robot Isaac Lab Extension Project

This repository contains the complete BALLU (Buoyancy-Assisted Legged Locomotion Unit) robot research project, which integrates Isaac Lab extensions for morphology optimization and locomotion training using reinforcement learning.

## 🚀 Project Overview

The BALLU project is focused on optimizing the morphology of the BALLU robot to achieve stable and dynamic locomotion.

## 📁 Repository Structure

```
BALLU_Project/
├── ballu_isclb_extension/          # Main Isaac Lab extension
│   ├── source/ballu_isaac_extension/  # Extension source code
│   ├── scripts/                    # Training and visualization scripts
│   └── config/                     # Extension configuration
├── isaac_lab/                      # Isaac Lab submodule (forked)
├── rsl_rl_lib/                     # RSL-RL submodule (forked)
└── .gitmodules                    # Git submodule configuration
```

## 🏗️ Installation & Setup

### Prerequisites
- Python 3.10
- NVIDIA Isaac Sim 4.5.0

### Step 1: Clone the Repository with Submodules

Since this project uses git submodules, you need to clone with special flags:

```bash
# Method 1: Clone with submodules (recommended)
git clone --recurse-submodules <repository_url> BALLU_Project
cd BALLU_Project

# Method 2: If already cloned without submodules
git clone <repository_url> BALLU_Project
cd BALLU_Project
git submodule init
git submodule update --recursive

# Method 3: Update existing clone
git submodule update --init --recursive
```

### Step 2: Verify Submodule Status

Check that all submodules are properly initialized:

```bash
git submodule status
```

You should see both `isaac_lab` and `rsl_rl_lib` submodules listed as initialized.

### Step 3: Install Dependencies

WIP

## 🎯 Quick Start

### Training a Policy

```bash
cd ballu_isclb_extension
python scripts/rsl_rl/train.py --task Isc-Vel-BALLU-encoder --num_envs 16 --seed 0 --max_iterations 2000
```

### Visualizing Results

```bash
cd ballu_isclb_extension
python scripts/rsl_rl/play.py --task Isc-Vel-BALLU-encoder --load_run <run_name> --checkpoint <model_name> --num_envs 4 --video
```

## 📚 About the BALLU Project

The BALLU (Buoyancy-Assisted Legged Locomotion Unit) robot represents an innovative approach to legged locomotion that leverages buoyancy assistance for enhanced stability and efficiency. This research project focuses on:

- **Morphology Optimization**: Systematically testing different robot configurations to find optimal designs
- **Sensor Integration**: Implementing various sensor suites (IMUs, cameras) for state estimation
- **Reinforcement Learning**: Training robust locomotion policies using PPO algorithm
- **Real-world Transfer**: Bridging the simulation-to-reality gap for physical robot deployment

## 📄 License

This project is part of the BALLU robot research initiative. Please refer to individual submodule repositories for specific licensing information.
