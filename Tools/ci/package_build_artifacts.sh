#!/bin/bash

# Enable globstar for recursive globbing
shopt -s globstar

mkdir artifacts

# Copy px4 files for regular flight controllers
cp **/**/*.px4 artifacts/ 2>/dev/null || true

# Copy uavcan.bin files for CAN nodes to named folders
mkdir -p artifacts/can_nodes
for uavcan_bin in **/**/*.uavcan.bin; do
  if [ -f "$uavcan_bin" ]; then
    # Extract build directory name (e.g., "ark_can-flow_default" from "build/ark_can-flow_default/...")
    build_dir=$(echo "$uavcan_bin" | sed 's|build/\([^/]*\)/.*|\1|')
    
    # Create subdirectory for this CAN node
    can_node_dir="artifacts/can_nodes/$build_dir"
    mkdir -p "$can_node_dir"
    
    # Copy the uavcan.bin file with original name
    cp "$uavcan_bin" "$can_node_dir/"
    
    echo "Packaged CAN node firmware: $uavcan_bin -> $can_node_dir/"
  fi
done

# Also check for CAN node binaries in deploy directories
for deploy_bin in **/deploy/*.bin; do
  if [ -f "$deploy_bin" ]; then
    # Extract build directory name from deploy path
    build_dir=$(echo "$deploy_bin" | sed 's|build/\([^/]*\)/.*|\1|')
    
    # Create subdirectory for this CAN node
    can_node_dir="artifacts/can_nodes/$build_dir"
    mkdir -p "$can_node_dir"
    
    # Copy the bin file
    cp "$deploy_bin" "$can_node_dir/"
    
    echo "Packaged CAN node firmware from deploy: $deploy_bin -> $can_node_dir/"
  fi
done

cp **/**/*.elf artifacts/ 2>/dev/null || true
for build_dir_path in build/*/ ; do
  build_dir_path=${build_dir_path::${#build_dir_path}-1}
  build_dir=${build_dir_path#*/}
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
