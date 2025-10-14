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

# Detect available terminal emulator (Ubuntu only)
TERMINAL=""
if command -v terminator &> /dev/null; then
    TERMINAL="terminator"
elif command -v gnome-terminal &> /dev/null; then
    TERMINAL="gnome-terminal"
else
    echo "ERROR: No compatible terminal emulator found"
    echo "Please install either terminator or gnome-terminal"
    echo "  sudo apt install terminator"
    echo "  or"
    echo "  sudo apt install gnome-terminal"
    exit 1
fi

echo "Using terminal: $TERMINAL"

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

# # Kill any existing PX4 instances
# echo "Cleaning up any existing PX4 instances..."
# pkill -x px4 || true
# sleep 1

# # Kill any existing Gazebo instances
# echo "Cleaning up any existing Gazebo instances..."
# pkill -x ruby || true  # Gazebo uses ruby
# pkill gz || true
# sleep 2

echo ""
echo "Launching vehicles..."
echo ""

# Function to launch terminal based on type
launch_terminal() {
    local title="$1"
    local instance="$2"
    local working_dir="$3"
    local y_pos="$4"

    case "$TERMINAL" in
        terminator)
            terminator --title="$title" -x bash -c "
                cd '$working_dir'
                echo '=========================================='
                echo '$title'
                echo '=========================================='
                echo 'Working directory: $working_dir'
                echo 'Instance: $instance'
                if [ $instance -eq 0 ]; then
                    echo 'Mode: Primary (launching Gazebo)'
                    export PX4_SYS_AUTOSTART=4001
                    export PX4_SIM_MODEL=gz_x500
                    '$BUILD_DIR/bin/px4' -i $instance -d '$BUILD_DIR/etc'
                else
                    echo 'Mode: Secondary (connecting to Gazebo)'
                    echo 'Position: (0, $y_pos, 0)'
                    export PX4_SYS_AUTOSTART=4001
                    export PX4_SIM_MODEL=gz_x500
                    export PX4_GZ_MODEL_POSE='0,$y_pos'
                    export PX4_GZ_STANDALONE=1
                    '$BUILD_DIR/bin/px4' -i $instance -d '$BUILD_DIR/etc'
                fi
                echo ''
                echo 'PX4 exited. Press Enter to close this terminal...'
                read
            " &
            ;;
        gnome-terminal)
            gnome-terminal --title="$title" -- bash -c "
                cd '$working_dir'
                echo '=========================================='
                echo '$title'
                echo '=========================================='
                echo 'Working directory: $working_dir'
                echo 'Instance: $instance'
                if [ $instance -eq 0 ]; then
                    echo 'Mode: Primary (launching Gazebo)'
                    export PX4_SYS_AUTOSTART=4001
                    export PX4_SIM_MODEL=gz_x500
                    '$BUILD_DIR/bin/px4' -i $instance -d '$BUILD_DIR/etc'
                else
                    echo 'Mode: Secondary (connecting to Gazebo)'
                    echo 'Position: (0, $y_pos, 0)'
                    export PX4_SYS_AUTOSTART=4001
                    export PX4_SIM_MODEL=gz_x500
                    export PX4_GZ_MODEL_POSE='0,$y_pos'
                    export PX4_GZ_STANDALONE=1
                    '$BUILD_DIR/bin/px4' -i $instance -d '$BUILD_DIR/etc'
                fi
                echo ''
                echo 'PX4 exited. Press Enter to close this terminal...'
                read
            " &
            ;;
    esac
}

# Launch each vehicle instance
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

    # MAV_SYS_ID will be instance + 1
    mav_sys_id=$((i + 1))

    echo "----------------------------------------"
    echo "Launching Vehicle $i"
    echo "  Working directory: $working_dir"
    echo "  Position: (0, $y_pos, 0)"
    echo "  MAV_SYS_ID: $mav_sys_id"
    echo "  Model name: x500_${i}"

    if [ $i -eq 0 ]; then
        echo "  Mode: Primary (launching Gazebo)"
    else
        echo "  Mode: Secondary (connecting to Gazebo)"
    fi

    # Launch in new terminal
    title="PX4 Vehicle $i (MAV_SYS_ID=$mav_sys_id)"
    launch_terminal "$title" "$i" "$working_dir" "$y_pos"

    echo "  Status: Terminal launched"
    echo "----------------------------------------"
    echo ""

    # Wait for first instance to start Gazebo before launching others
    if [ $i -eq 0 ]; then
        echo "Waiting for Gazebo to start (15 seconds)..."
        sleep 15
    fi
done

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
echo "- Each vehicle is running in its own terminal window"
echo "- QGroundControl should auto-connect to all vehicles"
echo "- For ROS 2, start MicroXRCEAgent: MicroXRCEAgent udp4 -p 8888"
echo "- To stop all vehicles, close the terminal windows or run: pkill -x px4"
echo ""
echo "=================================================="
