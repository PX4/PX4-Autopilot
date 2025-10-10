#!/bin/bash

mkdir artifacts
mkdir -p artifacts/cannode

# Function to check if a build is a CAN node
is_cannode() {
  local build_path=$1
  local boardconfig="$build_path/boardconfig"

  if [ -f "$boardconfig" ]; then
    # Check if CONFIG_BOARD_ROMFSROOT is set to "cannode"
    if grep -q '^CONFIG_BOARD_ROMFSROOT="cannode"' "$boardconfig"; then
      return 0
    fi
  fi
  return 1
}

# First pass: identify and package CAN nodes
for build_dir_path in build/*/ ; do
  # Remove trailing slash
  build_dir_path=${build_dir_path::${#build_dir_path}-1}
  build_dir=${build_dir_path#*/}

  if is_cannode "$build_dir_path"; then
    # Find the .uavcan.bin file
    uavcan_bin=$(find "$build_dir_path" -maxdepth 1 -name "*.uavcan.bin" -type f)

    if [ -n "$uavcan_bin" ]; then
      # Extract the target name (e.g., ark_can-flow from ark_can-flow_default)
      target_name=${build_dir%_default}

      # Create a directory for this CAN node
      mkdir -p "artifacts/cannode/$target_name"

      # Copy the .uavcan.bin file
      cp "$uavcan_bin" "artifacts/cannode/$target_name/"

      echo "Packaged CAN node: $target_name"
    fi
  fi
done

# Second pass: package regular firmware binaries (excluding CAN nodes)
for build_dir_path in build/*/ ; do
  build_dir_path=${build_dir_path::${#build_dir_path}-1}

  # Skip CAN nodes
  if is_cannode "$build_dir_path"; then
    continue
  fi

  # Copy .px4 files
  find "$build_dir_path" -maxdepth 1 -name "*.px4" -type f -exec cp {} artifacts/ \;

  # Copy .elf files
  find "$build_dir_path" -maxdepth 1 -name "*.elf" -type f -exec cp {} artifacts/ \;
done

# Third pass: package metadata for each non-CAN node build directory
for build_dir_path in build/*/ ; do
  build_dir_path=${build_dir_path::${#build_dir_path}-1}
  build_dir=${build_dir_path#*/}

  # Skip CAN node builds for metadata packaging (they only need the .uavcan.bin)
  if is_cannode "$build_dir_path"; then
    continue
  fi

  mkdir artifacts/$build_dir
  find artifacts/ -maxdepth 1 -type f -name "*$build_dir*"
  # Airframe
  cp $build_dir_path/airframes.xml artifacts/$build_dir/
  # Parameters
  cp $build_dir_path/parameters.xml artifacts/$build_dir/
  cp $build_dir_path/parameters.json artifacts/$build_dir/
  cp $build_dir_path/parameters.json.xz artifacts/$build_dir/
  # Actuators
  cp $build_dir_path/actuators.json artifacts/$build_dir/
  cp $build_dir_path/actuators.json.xz artifacts/$build_dir/
  # Events
  cp $build_dir_path/events/all_events.json.xz artifacts/$build_dir/
  # ROS 2 msgs
  cp $build_dir_path/events/all_events.json.xz artifacts/$build_dir/
  # Module Docs
  ls -la artifacts/$build_dir
  echo "----------"
done

if [ -d artifacts/px4_sitl_default ]; then
  # general metadata
  mkdir artifacts/_general/
  cp artifacts/px4_sitl_default/airframes.xml artifacts/_general/
  # Airframe
  cp artifacts/px4_sitl_default/airframes.xml artifacts/_general/
  # Parameters
  cp artifacts/px4_sitl_default/parameters.xml artifacts/_general/
  cp artifacts/px4_sitl_default/parameters.json artifacts/_general/
  cp artifacts/px4_sitl_default/parameters.json.xz artifacts/_general/
  # Actuators
  cp artifacts/px4_sitl_default/actuators.json artifacts/_general/
  cp artifacts/px4_sitl_default/actuators.json.xz artifacts/_general/
  # Events
  cp artifacts/px4_sitl_default/events/all_events.json.xz artifacts/_general/
  # ROS 2 msgs
  cp artifacts/px4_sitl_default/events/all_events.json.xz artifacts/_general/
  # Module Docs
  ls -la artifacts/_general/
fi
