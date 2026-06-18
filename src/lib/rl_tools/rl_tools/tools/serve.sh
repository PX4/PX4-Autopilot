#!/usr/bin/env bash
# This script does three things:
# 1. Downloads the JavaScript dependencies for the ExTrack UI
# 2. Runs a simple script that creates an index (simple list of files) of the experiments
# 3. Starts a HTTP server to serve the main folder (where there is front-end code that uses the index of files to detect experiments and display them)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
bash $PARENT_DIR/static/extrack_ui/download_dependencies.sh
bash -c "while true; do $SCRIPT_DIR/index_experiments.sh $PARENT_DIR/experiments; sleep 10; done" &
LOOP_PID=$!

# Cleanup function to kill the background process
cleanup() {
    echo 'Shutting down...'
    kill $LOOP_PID 2>/dev/null || true
    exit "${1:-0}"
}

# Set up traps for both interrupts and any exit (including errors)
trap "cleanup 0" SIGINT
trap "cleanup 1" ERR EXIT

python3 -m http.server -d "$PARENT_DIR"
