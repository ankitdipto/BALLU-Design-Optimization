#!/bin/bash
# Launcher script for humanoid simulations

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Banner
echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║                                                                    ║"
echo "║           🤖  MuJoCo Humanoid Simulation Launcher  🤖             ║"
echo "║                                                                    ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check if conda environment is activated
if [[ -z "$CONDA_DEFAULT_ENV" ]]; then
    echo -e "${YELLOW}⚠️  Warning: No conda environment detected${NC}"
    echo -e "${YELLOW}   Consider running: conda activate mujoco_v0${NC}"
    echo ""
fi

# Function to show menu
show_menu() {
    echo -e "${BLUE}Please select an option:${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} 🌐 Web Streaming (Real-time browser view)"
    echo -e "  ${GREEN}2)${NC} 🎬 Simple Export (All modes, 10s each)"
    echo -e "  ${GREEN}3)${NC} 🚶 Walking Only (20 seconds)"
    echo -e "  ${GREEN}4)${NC} 🏋️  Squat Only (20 seconds)"
    echo -e "  ${GREEN}5)${NC} 💃 Dance Only (20 seconds)"
    echo -e "  ${GREEN}6)${NC} 🧍 Balance Only (20 seconds)"
    echo -e "  ${GREEN}7)${NC} 📚 View Documentation"
    echo -e "  ${GREEN}8)${NC} 🔧 Install Dependencies"
    echo -e "  ${GREEN}9)${NC} ❌ Exit"
    echo ""
    echo -ne "${CYAN}Enter your choice [1-9]: ${NC}"
}

# Function to check dependencies
check_dependencies() {
    echo -e "${BLUE}Checking dependencies...${NC}"
    
    # Check Python
    if ! command -v python &> /dev/null; then
        echo -e "${RED}❌ Python not found${NC}"
        return 1
    fi
    
    # Check required packages
    python -c "import mujoco" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ MuJoCo not installed${NC}"
        return 1
    fi
    
    python -c "import numpy" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ NumPy not installed${NC}"
        return 1
    fi
    
    python -c "import imageio" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ imageio not installed${NC}"
        return 1
    fi
    
    echo -e "${GREEN}✅ Core dependencies OK${NC}"
    
    # Check optional dependencies
    python -c "import flask" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo -e "${YELLOW}⚠️  Flask not installed (needed for web streaming)${NC}"
    else
        echo -e "${GREEN}✅ Flask OK${NC}"
    fi
    
    python -c "import cv2" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo -e "${YELLOW}⚠️  OpenCV not installed (needed for web streaming)${NC}"
    else
        echo -e "${GREEN}✅ OpenCV OK${NC}"
    fi
    
    return 0
}

# Function to install dependencies
install_dependencies() {
    echo -e "${BLUE}Installing dependencies...${NC}"
    echo ""
    
    cd "$SCRIPT_DIR/.."
    
    echo -e "${CYAN}Installing from requirements.txt...${NC}"
    pip install -r requirements.txt
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Dependencies installed successfully${NC}"
    else
        echo -e "${RED}❌ Installation failed${NC}"
    fi
    
    echo ""
    read -p "Press Enter to continue..."
}

# Function to run web streaming
run_webstream() {
    echo -e "${MAGENTA}"
    echo "════════════════════════════════════════════════════════════════════"
    echo "  Starting Web Streaming Server..."
    echo "════════════════════════════════════════════════════════════════════"
    echo -e "${NC}"
    
    # Check if Flask is available
    python -c "import flask" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ Flask not installed!${NC}"
        echo -e "${YELLOW}Please install it first: pip install flask opencv-python${NC}"
        echo ""
        read -p "Press Enter to continue..."
        return
    fi
    
    python humanoid_webstream.py
}

# Function to run simple export
run_simple() {
    echo -e "${MAGENTA}"
    echo "════════════════════════════════════════════════════════════════════"
    echo "  Running Simple Export (All Modes)..."
    echo "════════════════════════════════════════════════════════════════════"
    echo -e "${NC}"
    
    python humanoid_simple.py
    
    echo ""
    echo -e "${GREEN}✅ Videos saved to: ../outputs/${NC}"
    echo ""
    read -p "Press Enter to continue..."
}

# Function to run specific mode
run_mode() {
    local mode=$1
    local duration=${2:-20}
    
    echo -e "${MAGENTA}"
    echo "════════════════════════════════════════════════════════════════════"
    echo "  Running ${mode^^} mode for ${duration} seconds..."
    echo "════════════════════════════════════════════════════════════════════"
    echo -e "${NC}"
    
    python humanoid_simple.py "$mode" "$duration"
    
    echo ""
    echo -e "${GREEN}✅ Video saved to: ../outputs/humanoid_${mode}.mp4${NC}"
    echo ""
    read -p "Press Enter to continue..."
}

# Function to view documentation
view_docs() {
    clear
    if [ -f "$SCRIPT_DIR/../HUMANOID_SIMULATION.md" ]; then
        less "$SCRIPT_DIR/../HUMANOID_SIMULATION.md"
    else
        echo -e "${RED}❌ Documentation not found${NC}"
        echo ""
        read -p "Press Enter to continue..."
    fi
}

# Main loop
while true; do
    clear
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════════════╗"
    echo "║                                                                    ║"
    echo "║           🤖  MuJoCo Humanoid Simulation Launcher  🤖             ║"
    echo "║                                                                    ║"
    echo "╚════════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo ""
    
    show_menu
    read choice
    
    case $choice in
        1)
            run_webstream
            ;;
        2)
            run_simple
            ;;
        3)
            run_mode "walking" 20
            ;;
        4)
            run_mode "squat" 20
            ;;
        5)
            run_mode "dance" 20
            ;;
        6)
            run_mode "balance" 20
            ;;
        7)
            view_docs
            ;;
        8)
            install_dependencies
            ;;
        9)
            echo -e "${GREEN}Goodbye! 👋${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid option. Please try again.${NC}"
            sleep 2
            ;;
    esac
done

