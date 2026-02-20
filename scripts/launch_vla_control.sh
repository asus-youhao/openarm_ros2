#!/bin/bash
#
# Launch Script for OpenArm VLA Control System
# =============================================
# This script launches all necessary components for the GR00T N1.5 VLA control system.
#
# Usage:
#   ./launch_vla_control.sh [options]
#
# Options:
#   --hardware    Launch robot hardware (default: true)
#   --state       Launch state publisher (default: true)
#   --action      Launch action chunk controller (default: true)
#   --build       Build workspace before launch (default: false)
#   --help        Show this help message
#
# Requirements:
#   - ROS2 Humble/Iron installed
#   - Workspace built at ~/openarm_ros2
#   - Hardware connected (CAN interface, Serial port)

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default options
LAUNCH_HARDWARE=true
LAUNCH_STATE_PUBLISHER=true
LAUNCH_ACTION_CONTROLLER=true
BUILD_WORKSPACE=false

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --hardware)
            LAUNCH_HARDWARE=true
            shift
            ;;
        --no-hardware)
            LAUNCH_HARDWARE=false
            shift
            ;;
        --state)
            LAUNCH_STATE_PUBLISHER=true
            shift
            ;;
        --no-state)
            LAUNCH_STATE_PUBLISHER=false
            shift
            ;;
        --action)
            LAUNCH_ACTION_CONTROLLER=true
            shift
            ;;
        --no-action)
            LAUNCH_ACTION_CONTROLLER=false
            shift
            ;;
        --build)
            BUILD_WORKSPACE=true
            shift
            ;;
        --help|-h)
            echo "OpenArm VLA Control System Launch Script"
            echo ""
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --hardware      Launch robot hardware (default: true)"
            echo "  --no-hardware   Skip hardware launch"
            echo "  --state         Launch state publisher (default: true)"
            echo "  --no-state      Skip state publisher"
            echo "  --action        Launch action controller (default: true)"
            echo "  --no-action     Skip action controller"
            echo "  --build         Build workspace before launch"
            echo "  --help, -h      Show this help message"
            echo ""
            echo "Example:"
            echo "  $0 --build                    # Build and launch everything"
            echo "  $0 --no-hardware              # Launch without hardware (simulation)"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Array to store background process PIDs
declare -a PIDS=()

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Shutting down...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
    exit 0
}

trap cleanup SIGINT SIGTERM

# Print banner
echo -e "${BLUE}"
echo "=============================================="
echo "   OpenArm VLA Control System Launcher"
echo "   GR00T N1.5 Integration"
echo "=============================================="
echo -e "${NC}"

# Build workspace if requested
if [ "$BUILD_WORKSPACE" = true ]; then
    echo -e "${YELLOW}Building workspace...${NC}"
    cd "$WORKSPACE_DIR"
    colcon build --packages-select openarm_hardware
    echo -e "${GREEN}Build complete!${NC}"
fi

# Source workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
source "$WORKSPACE_DIR/install/setup.bash"

# Launch hardware
if [ "$LAUNCH_HARDWARE" = true ]; then
    echo -e "${GREEN}[1/3] Launching robot hardware...${NC}"
    ros2 launch openarm_bringup openarm.bimanual.launch.py &
    PIDS+=($!)
    echo -e "  Hardware PID: ${PIDS[-1]}"
    sleep 5  # Wait for hardware to initialize
fi

# Launch state publisher
if [ "$LAUNCH_STATE_PUBLISHER" = true ]; then
    echo -e "${GREEN}[2/3] Launching GR00T state publisher (50Hz)...${NC}"
    python3 "$SCRIPT_DIR/groot_state_publisher.py" &
    PIDS+=($!)
    echo -e "  State Publisher PID: ${PIDS[-1]}"
    sleep 1
fi

# Launch action chunk controller
if [ "$LAUNCH_ACTION_CONTROLLER" = true ]; then
    echo -e "${GREEN}[3/3] Launching action chunk controller...${NC}"
    python3 "$SCRIPT_DIR/action_chunk_controller.py" &
    PIDS+=($!)
    echo -e "  Action Controller PID: ${PIDS[-1]}"
    sleep 1
fi

# Print status
echo -e "${BLUE}"
echo "=============================================="
echo "   System Status"
echo "==============================================${NC}"
echo -e "  Hardware:           $([ "$LAUNCH_HARDWARE" = true ] && echo -e "${GREEN}Running${NC}" || echo -e "${YELLOW}Skipped${NC}")"
echo -e "  State Publisher:    $([ "$LAUNCH_STATE_PUBLISHER" = true ] && echo -e "${GREEN}Running${NC}" || echo -e "${YELLOW}Skipped${NC}")"
echo -e "  Action Controller:  $([ "$LAUNCH_ACTION_CONTROLLER" = true ] && echo -e "${GREEN}Running${NC}" || echo -e "${YELLOW}Skipped${NC}")"
echo ""
echo -e "  Topics:"
echo -e "    /joint_states           - Raw joint states (100Hz)"
echo -e "    /groot/joint_states     - Filtered states for GR00T (50Hz)"
echo -e "    /groot/state_health     - Health statistics"
echo -e "    /action_chunk           - Action chunk input (from GR00T)"
echo ""
echo -e "${YELLOW}Press Ctrl+C to shutdown all components${NC}"
echo ""

# Wait for all processes
wait