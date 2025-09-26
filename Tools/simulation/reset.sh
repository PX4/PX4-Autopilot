#!/bin/bash
# reset.sh
# Helper script to reset the SITL simulation.
# Usage: ./reset.sh <px4_source_root_dir>

set -e # Exit immediately if a command exits with a non-zero status.

if [ -z "$1" ]; then
    echo "Usage: $0 <px4_source_root_dir>"
    echo "ERROR: PX4 source directory argument required."
    exit 1
fi

PX4_SRC_DIR="$1"

# Function to find the original make command
get_make_command() {
    # Look for the make process that is running the simulation
    # We search for a process with 'make' and the build path in its arguments
    local build_path="${PX4_SRC_DIR}/build/px4_sitl_default"

    # Use ps to find the make command, filter out common false positives, and get the last one
    local make_cmd
    make_cmd=$(ps aux | grep -E "[m]ake.*${build_path}" | grep -v "grep" | tail -1 | awk '{for(i=11;i<=NF;++i)printf "%s ", $i; print ""}')

    # If we found a command, use it. Otherwise, fall back to the default.
    if [ -n "$make_cmd" ]; then
        echo "$make_cmd"
    else
        # Fallback default
        echo "make px4_sitl gazebo-classic"
    fi
}

# Function to perform the actual reset and restart
perform_reset() {
    # Navigate to the PX4 source directory
    cd "${PX4_SRC_DIR}" || { echo "ERROR: Could not cd to ${PX4_SRC_DIR}"; exit 1; }

    echo "Resetting simulation for SITL from $(pwd)..."

    # 1. Get the original make command before killing anything
    local make_cmd
    make_cmd=$(get_make_command)
    echo "Detected original command: $make_cmd"

    # 2. Kill the Gazebo Client (GUI)
    pkill -f "gzclient" || true

    # 3. Kill the Gazebo Server (physics engine).
    pkill -f "gzserver" || true

    # 4. Forcefully kill the PX4 process itself if it's still running
    pkill -f "bin/px4" || true

    # 5. Also kill the make process to clean up the old terminal
    pkill -f "make.*px4_sitl" || true

    # 6. Brief pause to let the system clean up processes
    sleep 2

    # 7. Restart the simulation in a NEW terminal window using the detected command
    echo "Starting a new simulation instance with: $make_cmd"

    # Determine the default terminal emulator
    if command -v gnome-terminal &> /dev/null; then
        TERMINAL_CMD="gnome-terminal -- bash -c"
    elif command -v konsole &> /dev/null; then
        TERMINAL_CMD="konsole -e bash -c"
    elif command -v xterm &> /dev/null; then
        TERMINAL_CMD="xterm -e bash -c"
    else
        echo "ERROR: No known terminal emulator found (gnome-terminal, konsole, xterm)."
        exit 1
    fi

    # Execute the command in a new terminal window
    $TERMINAL_CMD "cd $PX4_SRC_DIR && $make_cmd; exec bash" &
}

# Call the function
perform_reset
