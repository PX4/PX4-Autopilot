#!/usr/bin/env bash

# Exit immediately if any command fails
set -e

# --- Configuration Section ---
# Path to the PX4 root directory.
# Assuming the script is run from the PX4 root directory.
# If not, please modify this line, e.g.: PX4_ROOT_DIR="/path/to/your/PX4-Autopilot"
PX4_ROOT_DIR="."

# Default parameters, can be overridden via command-line arguments
RPI_USER="pi"                     # Raspberry Pi username
RPI_IP=""                         # Raspberry Pi IP address
REMOTE_TARGET_DIR="/home/pi/px4_build" # Target directory on Raspberry Pi for the build output (e.g., bin, etc folders will go here)
PX4_BUILD_TARGET=""               # PX4 build target, e.g., px4_fmu-v5_default

# --- Function Definitions ---

# Print usage instructions
usage() {
  echo "Usage: $0 -t <px4_build_target> -i <rpi_ip> [-u <rpi_user>] [-d <remote_target_dir>]"
  echo ""
  echo "Arguments:"
  echo "  -t <px4_build_target>    : Required. PX4 build target (e.g., px4_fmu-v5_default)."
  echo "  -i <rpi_ip>            : Required. Raspberry Pi's IP address."
  echo "  -u <rpi_user>          : Optional. Raspberry Pi username (default: ${RPI_USER})."
  echo "  -d <remote_target_dir> : Optional. Remote directory on Raspberry Pi (default: ${REMOTE_TARGET_DIR})."
  echo ""
  echo "Example:"
  echo "  $0 -t px4_fmu-v5_default -i 192.168.1.100"
  echo "  $0 -t px4_fmu-v5_default -i 192.168.1.100 -u customuser -d /opt/my_px4_firmware_dir"
  exit 1
}

# --- Parse Command Line Arguments ---
while getopts "t:i:u:d:" opt; do
  case "${opt}" in
    t)
      PX4_BUILD_TARGET="${OPTARG}"
      ;;
    i)
      RPI_IP="${OPTARG}"
      ;;
    u)
      RPI_USER="${OPTARG}"
      ;;
    d)
      REMOTE_TARGET_DIR="${OPTARG}"
      ;;
    *)
      usage
      ;;
  esac # This was the line with the typo: esoc -> esac
done
shift $((OPTIND-1))

# --- Parameter Validation ---
if [ -z "${PX4_BUILD_TARGET}" ] || [ -z "${RPI_IP}" ]; then
  echo "Error: Missing required arguments (-t or -i)."
  usage
fi

# Construct the base directory for PX4 build outputs (e.g., build/px4_fmu-v5_default)
PX4_BUILD_OUTPUT_DIR="${PX4_ROOT_DIR}/build/${PX4_BUILD_TARGET}"

# Check if the base build output directory exists
if [ ! -d "${PX4_BUILD_OUTPUT_DIR}" ]; then
  echo "Error: Local PX4 build output directory does not exist or is invalid: ${PX4_BUILD_OUTPUT_DIR}"
  echo "Please ensure PX4 has been successfully compiled for target '${PX4_BUILD_TARGET}'."
  exit 1
fi

# Check if 'bin' and 'etc' subdirectories exist
if [ ! -d "${PX4_BUILD_OUTPUT_DIR}/bin" ]; then
    echo "Warning: '${PX4_BUILD_OUTPUT_DIR}/bin' directory not found. Skipping bin folder transfer."
fi
if [ ! -d "${PX4_BUILD_OUTPUT_DIR}/etc" ]; then
    echo "Warning: '${PX4_BUILD_OUTPUT_DIR}/etc' directory not found. Skipping etc folder transfer."
fi

echo "--- Starting PX4 file deployment to Raspberry Pi (e.g., pi@navio) ---"
echo "Local PX4 Root Dir:       ${PX4_ROOT_DIR}"
echo "Local PX4 Build Output:   ${PX4_BUILD_OUTPUT_DIR}"
echo "Raspberry Pi User@IP:     ${RPI_USER}@${RPI_IP}"
echo "Raspberry Pi Target Dir:  ${REMOTE_TARGET_DIR}"
echo "-------------------------------------------------------------------"

# --- Create remote directory (if it doesn't exist) ---
echo "1. Attempting to create target directory on Raspberry Pi..."
# Target host: pi@navio
ssh "${RPI_USER}@${RPI_IP}" "mkdir -p ${REMOTE_TARGET_DIR}"
if [ $? -eq 0 ]; then
  echo "   Remote directory '${REMOTE_TARGET_DIR}' confirmed or created."
else
  echo "Error: Could not create or access target directory on Raspberry Pi. Please check SSH connection and permissions."
  exit 1
fi

# --- Use scp to transfer 'bin' and 'etc' folders ---
echo "2. Transferring 'bin' and 'etc' folders using scp..."

# Build the source list for scp
declare -a SOURCE_DIRS
if [ -d "${PX4_BUILD_OUTPUT_DIR}/bin" ]; then
    SOURCE_DIRS+=("${PX4_BUILD_OUTPUT_DIR}/bin")
fi
if [ -d "${PX4_BUILD_OUTPUT_DIR}/etc" ]; then
    SOURCE_DIRS+=("${PX4_BUILD_OUTPUT_DIR}/etc")
fi

if [ ${#SOURCE_DIRS[@]} -eq 0 ]; then
    echo "Error: Neither 'bin' nor 'etc' directories found in '${PX4_BUILD_OUTPUT_DIR}'. Nothing to transfer."
    exit 1
fi

# -r: Recursively copy directories
# -P: Specify SSH port (if not default 22) - not used here, implies default
# -v: Display verbose transfer information (optional, for debugging)
# This will copy the 'bin' and 'etc' folders themselves into the REMOTE_TARGET_DIR
scp -r "${SOURCE_DIRS[@]}" "${RPI_USER}@${RPI_IP}:${REMOTE_TARGET_DIR}/"

if [ $? -eq 0 ]; then
  echo "--- File transfer successful! ---"
  echo "PX4 'bin' and 'etc' folders have been successfully uploaded to '${REMOTE_TARGET_DIR}' on Raspberry Pi."
  echo "Now, you might need to navigate to this directory on Raspberry Pi and execute relevant files, e.g., start PX4."
else
  echo "Error: File transfer failed. Please check the following:"
  echo "  - Is Raspberry Pi's network connection active?"
  echo "  - Is SSH service running on Raspberry Pi?"
  echo "  - Is SSH authentication (password or key) correct for ${RPI_USER}@${RPI_IP}?"
  echo "  - Are the permissions of the target directory on Raspberry Pi allowing writes?"
  exit 1
fi
