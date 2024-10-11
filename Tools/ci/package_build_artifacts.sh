#!/bin/bash

mkdir artifacts
cp **/**/*.px4 artifacts/
cp **/**/*.elf artifacts/
for build_dir_path in build/*/ ; do
  build_dir=${build_dir_path#*/}
  build_dir=${build_dir::${#build_dir}-1}
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
