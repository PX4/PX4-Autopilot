#!/bin/bash

# Check if both arguments are provided
if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: $0 <FILE_NAME> <DIRECTORY>"
    exit 1
fi

FILE_NAME=$1
DIRECTORY=$2

# Get the directory of the script to write the error file in the same location as the script
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ERROR_FILE="$SCRIPT_DIR/errors.txt"
DONE_FILE="$SCRIPT_DIR/px4_done"

# Check if the provided directory exists
if [ ! -d "$DIRECTORY" ]; then
    echo "Error: Directory '$DIRECTORY' does not exist."
    exit 1
fi

# Remove old done file before starting
rm -f "$DONE_FILE"

# Run the command in a new terminal
TERMINAL_CMD="make px4_sitl gz_${FILE_NAME} | tee >(while read -r line; do
    if echo \"\$line\" | grep -q \"INFO  \[px4\] Startup script returned successfully\"; then
        echo \"Startup script returned successfully. Stopping processing.\"
        echo 'done' > \"$DONE_FILE\"  # Mark completion
    elif echo \"\$line\" | grep -q \"ERROR\"; then
        echo \"\$line\" >> \"$ERROR_FILE\"
    fi
done)"

# Start a new terminal and run the command
if command -v gnome-terminal &>/dev/null; then
    gnome-terminal -- bash -c "cd '$DIRECTORY' && $TERMINAL_CMD; exec bash"
else
    echo "No supported terminal emulator found."
    exit 1
fi
