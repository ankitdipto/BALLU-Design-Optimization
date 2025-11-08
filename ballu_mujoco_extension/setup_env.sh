#!/bin/bash
# Setup script for MuJoCo project environment
# This script creates the conda environment and installs all dependencies

set -e  # Exit on error

echo "============================================================"
echo "MuJoCo Project Environment Setup"
echo "============================================================"

# Environment name
ENV_NAME="mujoco_v0"

# Check if conda is available
if ! command -v conda &> /dev/null; then
    echo "ERROR: conda not found. Please install Anaconda or Miniconda first."
    exit 1
fi

echo ""
echo "Creating conda environment: $ENV_NAME"
echo "Python version: 3.10"
echo ""

# Create conda environment
conda create -n $ENV_NAME python=3.10 -y

echo ""
echo "Activating environment..."
echo ""

# Activate environment
eval "$(conda shell.bash hook)"
conda activate $ENV_NAME

echo "Installing Python packages..."
echo ""

# Install packages from requirements.txt
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
else
    echo "Warning: requirements.txt not found. Installing packages directly..."
    pip install mujoco mujoco-python-viewer numpy imageio imageio-ffmpeg pyopengl PyOpenGL-accelerate pyyaml
fi

echo ""
echo "============================================================"
echo "Setup Complete!"
echo "============================================================"
echo ""
echo "To activate the environment, run:"
echo "  conda activate $ENV_NAME"
echo ""
echo "To test the installation, run:"
echo "  cd scripts"
echo "  python test_mujoco.py"
echo ""
echo "For more examples:"
echo "  python simple_example.py"
echo ""
echo "============================================================"

