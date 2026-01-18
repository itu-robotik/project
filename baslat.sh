#!/bin/bash

# ITU Robotics Combined Workspace - Unified Launch Script
# v2.0 - Full Integration (Nav2 + SLAM + Gemini)

echo "🚀 ITU Robotics System Starting..."
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Workspace Directories
WS_DIR="/home/metin/itu_robotics_ws/itu_robotics_combined_ws"
INSTALL_SETUP="$WS_DIR/install/setup.bash"
MAP_DIR="$WS_DIR/maps"
mkdir -p "$MAP_DIR"

# Source Workspace
if [ -f "$INSTALL_SETUP" ]; then
    source "$INSTALL_SETUP"
else
    echo -e "${RED}❌ setup.bash not found! Please BUILD first.${NC}"
fi

# Load Secrets
if [ -f ~/itu_robotics_ws/itu_project_ws/secrets.env ]; then
    source ~/itu_robotics_ws/itu_project_ws/secrets.env
fi

# Set Gazebo Resource Path for models (poster boards, etc.)
export GZ_SIM_RESOURCE_PATH="$WS_DIR/install/simulation_pkg/share/simulation_pkg/models:${GZ_SIM_RESOURCE_PATH:-}"

# Log Management
LOG_DIR="$WS_DIR/log"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/session_$(date +%Y%m%d_%H%M%S).txt"

# Functions
function build_project() {
    echo -e "${YELLOW}🔨 Building Project...${NC}"
    cd "$WS_DIR"
    colcon build --symlink-install
    source install/setup.bash
    echo -e "${GREEN}✅ Build Complete!${NC}"
    echo ""
}

function save_map() {
    read -p "Enter map name (default: cafeteria_map): " map_name
    map_name=${map_name:-"cafeteria_map"}
    echo -e "${YELLOW}💾 Saving map to $MAP_DIR/$map_name...${NC}"
    ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$map_name"
    echo -e "${GREEN}✅ Map saved!${NC}"
}

# Main Menu
while true; do
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}  ITU Robotics Control Center${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
    echo "1) 🏗️  Build Workspace"
    echo "2) 🗺️  START AUTO-MAPPING (SLAM + EXPLORER)"
    echo "   -> Launches Robot, SLAM, Logger AND Auto-Explorer."
    echo "   -> Robot will explore and map by itself! 🍹"
    echo "3) 💾 SAVE MAP"
    echo "   -> Saves the current SLAM map to disk."
    echo "4) 🤖 START PATROL (AUTONOMOUS)"
    echo "   -> Launches Robot, Nav2, Perception, and Patrol Logic."
    echo "   -> Uses the saved map."
    echo "5) 🧠 Launch Planner Only (Debug)"
    echo "6) ❌ Exit"
    echo ""
    read -p "Select option (1-6): " choice

    case $choice in
        1)
            build_project
            ;;
        2)
            echo -e "${GREEN}🗺️  Launching Mapping Mode...${NC}"
            echo "Logs will be saved to $LOG_FILE"
            ros2 launch simulation_pkg mapping.launch.py 2>&1 | tee -a "$LOG_FILE" &
            echo "Launched! Use 'Teleop' in another terminal to drive."
            ;;
        3)
            save_map
            ;;
        4)
            read -p "Enter map name to use (default: cafeteria_map.yaml): " map_name
            map_name=${map_name:-"cafeteria_map.yaml"}
            full_map_path="$MAP_DIR/$map_name"
            
            if [ ! -f "$full_map_path" ]; then
                echo -e "${RED}❌ Map file not found: $full_map_path${NC}"
                echo "Please Map and Save first (Option 2 & 3)."
            else
                echo -e "${GREEN}🤖 Launching Autonomous Patrol...${NC}"
                ros2 launch simulation_pkg patrol.launch.py map:="$full_map_path" 2>&1 | tee -a "$LOG_FILE" &
            fi
            ;;
        5)
            echo -e "${GREEN}🧠 Launching Planner Node...${NC}"
            ros2 run simulation_pkg planner_node.py --ros-args --log-level planner_node:=DEBUG 2>&1 | tee -a "$LOG_FILE" &
            ;;
        6)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid option!"
            ;;
    esac
    echo ""
    read -p "Press Enter to continue..."
done
