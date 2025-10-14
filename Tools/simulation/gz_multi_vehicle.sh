#!/bin/bash
# Multi-vehicle Gazebo (gz) simulation launcher for PX4
# Spawns multiple gz_x500 drones with configurable spacing
# Each instance runs in its own terminal window
# Usage: ./gz_multi_vehicle.sh [num_vehicles] [spacing]
#   num_vehicles: Number of drones to spawn (default: 2, max: 10)
#   spacing: Distance between drones in meters (default: 0.5)

set -e

# Configuration
num_vehicles=${1:-2}
spacing=${2:-1}
MAX_VEHICLES=10

# Get script directory and PX4 root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PX4_DIR="$SCRIPT_DIR/../.."

# Build directory
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"

# Check for required tools
if ! command -v terminator &> /dev/null; then
    echo "ERROR: terminator is required"
    echo "Please install terminator:"
    echo "  sudo apt install terminator"
    exit 1
fi

if ! command -v tmux &> /dev/null; then
    echo "ERROR: tmux is required for grid layout"
    echo "Please install tmux:"
    echo "  sudo apt install tmux"
    exit 1
fi

echo "Using terminator with tmux grid layout"

# Validate inputs
if ! [[ "$num_vehicles" =~ ^[0-9]+$ ]] || [ "$num_vehicles" -lt 1 ]; then
    echo "ERROR: Number of vehicles must be a positive integer"
    echo "Usage: $0 [num_vehicles] [spacing]"
    exit 1
fi

if [ "$num_vehicles" -gt "$MAX_VEHICLES" ]; then
    echo "ERROR: Number of vehicles cannot exceed $MAX_VEHICLES"
    echo "Usage: $0 [num_vehicles] [spacing]"
    exit 1
fi

if ! [[ "$spacing" =~ ^[0-9]+\.?[0-9]*$ ]]; then
    echo "ERROR: Spacing must be a positive number"
    echo "Usage: $0 [num_vehicles] [spacing]"
    exit 1
fi

# Check if PX4 is built
if [ ! -f "${BUILD_DIR}/bin/px4" ]; then
    echo "ERROR: PX4 SITL not built. Please run 'make px4_sitl' first"
    exit 1
fi

echo "=================================================="
echo "PX4 Multi-Vehicle Gazebo Launcher"
echo "=================================================="
echo "Number of vehicles: $num_vehicles"
echo "Spacing: ${spacing}m"
echo "Model: gz_x500"
echo "PX4 Directory: $PX4_DIR"
echo "=================================================="
echo ""

# Kill any existing PX4 instances (exact match only, won't kill this script)
echo "Cleaning up any existing PX4 instances..."
pkill -x px4 || true

# Kill any existing Gazebo instances (use exact match to avoid killing this script)
echo "Cleaning up any existing Gazebo instances..."
pkill -x gz || true

echo ""
echo "Launching vehicles..."
echo ""

# Prepare all vehicle instances
for i in $(seq 0 $((num_vehicles - 1))); do
    # Create working directory for this instance
    working_dir="${BUILD_DIR}/instance_${i}"
    mkdir -p "$working_dir"

    # Create symlink to gz_env.sh if it doesn't exist
    if [ ! -f "$working_dir/gz_env.sh" ]; then
        ln -sf "${BUILD_DIR}/rootfs/gz_env.sh" "$working_dir/gz_env.sh"
    fi

    # Calculate Y position (spacing along Y-axis)
    y_pos=$(echo "$i * $spacing" | bc -l)
    mav_sys_id=$((i + 1))

    echo "----------------------------------------"
    echo "Preparing Vehicle $i"
    echo "  Working directory: $working_dir"
    echo "  Position: (0, $y_pos, 0)"
    echo "  MAV_SYS_ID: $mav_sys_id"
    echo "  Model name: x500_${i}"
    echo "----------------------------------------"
done

echo ""

# Create tmux session with grid layout
SESSION_NAME="px4_multi_$(date +%s)"

echo "Creating tmux session with $num_vehicles panes in grid layout..."

# Start tmux session with first pane
working_dir_0="${BUILD_DIR}/instance_0"
tmux new-session -d -s "$SESSION_NAME" -c "$working_dir_0"

# Create additional panes (one for each vehicle after the first)
for i in $(seq 1 $((num_vehicles - 1))); do
    tmux split-window -t "$SESSION_NAME:0" -c "${BUILD_DIR}/instance_${i}"
    tmux select-layout -t "$SESSION_NAME:0" tiled
done

# Send commands to each pane
for i in $(seq 0 $((num_vehicles - 1))); do
    working_dir="${BUILD_DIR}/instance_${i}"
    y_pos=$(echo "$i * $spacing" | bc -l)
    mav_sys_id=$((i + 1))

    # Send commands to pane
    tmux send-keys -t "$SESSION_NAME:0.$i" "cd '$working_dir'" C-m
    tmux send-keys -t "$SESSION_NAME:0.$i" "echo '========================================='" C-m
    tmux send-keys -t "$SESSION_NAME:0.$i" "echo 'PX4 Vehicle $i (MAV_SYS_ID=$mav_sys_id)'" C-m
    tmux send-keys -t "$SESSION_NAME:0.$i" "echo '========================================='" C-m

    if [ $i -eq 0 ]; then
        tmux send-keys -t "$SESSION_NAME:0.$i" "echo 'Mode: Primary (launching Gazebo)'" C-m
        tmux send-keys -t "$SESSION_NAME:0.$i" "export PX4_SYS_AUTOSTART=4001" C-m
        tmux send-keys -t "$SESSION_NAME:0.$i" "export PX4_SIM_MODEL=gz_x500" C-m
        tmux send-keys -t "$SESSION_NAME:0.$i" "'$BUILD_DIR/bin/px4' -i $i -d '$BUILD_DIR/etc'" C-m
    else
        tmux send-keys -t "$SESSION_NAME:0.$i" "echo 'Mode: Secondary (connecting to Gazebo)'" C-m
        tmux send-keys -t "$SESSION_NAME:0.$i" "echo 'Position: (0, $y_pos, 0)'" C-m
        # For secondary instances, wait for Gazebo to start
        if [ $i -eq 1 ]; then
            tmux send-keys -t "$SESSION_NAME:0.$i" "echo 'Waiting for Gazebo to start...'" C-m
            tmux send-keys -t "$SESSION_NAME:0.$i" "sleep 15" C-m
        else
            tmux send-keys -t "$SESSION_NAME:0.$i" "sleep $((15 + (i-1) * 2))" C-m
        fi
        tmux send-keys -t "$SESSION_NAME:0.$i" "export PX4_SYS_AUTOSTART=4001" C-m
        tmux send-keys -t "$SESSION_NAME:0.$i" "export PX4_SIM_MODEL=gz_x500" C-m
        tmux send-keys -t "$SESSION_NAME:0.$i" "export PX4_GZ_MODEL_POSE='0,$y_pos'" C-m
        tmux send-keys -t "$SESSION_NAME:0.$i" "export PX4_GZ_STANDALONE=1" C-m
        tmux send-keys -t "$SESSION_NAME:0.$i" "'$BUILD_DIR/bin/px4' -i $i -d '$BUILD_DIR/etc'" C-m
    fi
done

echo ""
echo "All vehicles configured in tmux session: $SESSION_NAME"
echo "Attaching to tmux session..."
echo ""
echo "Tmux Commands:"
echo "  - Detach from tmux: Ctrl+B then D"
echo "  - Kill session: tmux kill-session -t $SESSION_NAME"
echo "  - Navigate panes: Ctrl+B then arrow keys"
echo "  - Zoom pane: Ctrl+B then Z (toggle)"
echo ""

# Attach to the session
exec tmux attach-session -t "$SESSION_NAME"

echo ""
echo "=================================================="
echo "All vehicles launched successfully!"
echo "=================================================="
echo ""
echo "Vehicle Summary:"
echo "----------------"
for i in $(seq 0 $((num_vehicles - 1))); do
    y_pos=$(echo "$i * $spacing" | bc -l)
    mav_sys_id=$((i + 1))
    echo "Vehicle $i:"
    echo "  - MAV_SYS_ID: $mav_sys_id"
    echo "  - Model name: x500_${i}"
    echo "  - Position: (0, $y_pos, 0)"
    echo "  - ROS 2 namespace: px4_${mav_sys_id}"
    echo "  - Terminal: PX4 Vehicle $i (MAV_SYS_ID=$mav_sys_id)"
    echo ""
done

echo "Tips:"
echo "-----"
echo "- Each vehicle runs in its own tmux pane in the grid"
echo "- QGroundControl should auto-connect to all vehicles"
echo "- For ROS 2, start MicroXRCEAgent: MicroXRCEAgent udp4 -p 8888"
echo "- Navigate panes: Ctrl+B then arrow keys"
echo "- Zoom pane: Ctrl+B then Z"
echo "- To stop all: Ctrl+C in each pane or pkill -x px4"
echo ""
echo "=================================================="
