#!/usr/bin/env bash

mkdir artifacts

# Flash images that are not the .px4 envelope:
# - CAN node application APDescriptor (.uavcan.bin)
# - Bootloaders (*_bootloader and *_canbootloader): raw .bin for SWD
#   *_bootloader_* variants (e.g. bootloader_secureboot) are omitted;
#   those images are baked with in-tree test keys.
for uavcan_bin in build/*/*.uavcan.bin; do
  [ -f "$uavcan_bin" ] || continue
  build_dir=$(basename "$(dirname "$uavcan_bin")")
  cp "$uavcan_bin" "artifacts/${build_dir}.uavcan.bin"
done

for bl_dir in build/*_bootloader build/*_canbootloader; do
  [ -d "$bl_dir" ] || continue
  build_dir=$(basename "$bl_dir")
  bl_bin="$bl_dir/${build_dir}.bin"
  [ -f "$bl_bin" ] || continue
  cp "$bl_bin" "artifacts/${build_dir}.bin"
done

for px4_file in build/*/*.px4; do
  [ -f "$px4_file" ] || continue
  build_dir=$(basename "$(dirname "$px4_file")")
  # CAN node application: .uavcan.bin is the flashable image
  if [ -f "artifacts/${build_dir}.uavcan.bin" ]; then
    continue
  fi
  # Bootloader .px4 is unused: SWD uses the raw .bin, and USB flashing would
  # write it into the application slot.
  if [[ "$build_dir" == *bootloader* ]]; then
    continue
  fi
  cp "$px4_file" artifacts/
done

cp **/**/*.elf artifacts/ 2>/dev/null || true
cp **/**/*.deb artifacts/ 2>/dev/null || true
for build_dir_path in build/*/ ; do
  build_dir_path=${build_dir_path::${#build_dir_path}-1}
  build_dir=${build_dir_path#*/}
  # GitHub Releases attach artifacts/**/*.sbom.spdx.json; bootloader SBOMs
  # are not a recovery artifact.
  case "$build_dir" in
    *_bootloader|*_bootloader_*|*_canbootloader) continue ;;
  esac
  mkdir -p artifacts/$build_dir
  find artifacts/ -maxdepth 1 -type f -name "*$build_dir*"
  # Airframe (NuttX: build root, SITL: docs/ subdirectory)
  airframes_src=""
  if [ -f "$build_dir_path/airframes.xml" ]; then
    airframes_src="$build_dir_path/airframes.xml"
  elif [ -f "$build_dir_path/docs/airframes.xml" ]; then
    airframes_src="$build_dir_path/docs/airframes.xml"
  fi
  if [ -n "$airframes_src" ]; then
    cp "$airframes_src" "artifacts/$build_dir/"
  fi
  # Parameters
  cp $build_dir_path/parameters.xml artifacts/$build_dir/ 2>/dev/null || true
  cp $build_dir_path/parameters.json artifacts/$build_dir/ 2>/dev/null || true
  cp $build_dir_path/parameters.json.xz artifacts/$build_dir/ 2>/dev/null || true
  # Actuators
  cp $build_dir_path/actuators.json artifacts/$build_dir/ 2>/dev/null || true
  cp $build_dir_path/actuators.json.xz artifacts/$build_dir/ 2>/dev/null || true
  # Events
  mkdir -p artifacts/$build_dir/events/
  cp $build_dir_path/events/all_events.json.xz artifacts/$build_dir/events/ 2>/dev/null || true
  # Also copy to top level: firmware advertises the metadata URI without the events/ subdirectory
  # (see src/lib/component_information/CMakeLists.txt comp_metadata_events_uri_board)
  cp $build_dir_path/events/all_events.json.xz artifacts/$build_dir/ 2>/dev/null || true
  # SBOM
  cp $build_dir_path/*.sbom.spdx.json artifacts/$build_dir/ 2>/dev/null || true
  ls -la artifacts/$build_dir
  echo "----------"
done

if [ -d artifacts/px4_sitl_default ]; then
  # general metadata (used by Flight Review and other downstream consumers)
  mkdir -p artifacts/_general/
  # Airframe
  if [ -f artifacts/px4_sitl_default/airframes.xml ]; then
    cp artifacts/px4_sitl_default/airframes.xml artifacts/_general/
  else
    echo "Error: expected 'artifacts/px4_sitl_default/airframes.xml' not found." >&2
    exit 1
  fi
  # Parameters
  cp artifacts/px4_sitl_default/parameters.xml artifacts/_general/
  cp artifacts/px4_sitl_default/parameters.json artifacts/_general/
  cp artifacts/px4_sitl_default/parameters.json.xz artifacts/_general/
  # Actuators
  cp artifacts/px4_sitl_default/actuators.json artifacts/_general/
  cp artifacts/px4_sitl_default/actuators.json.xz artifacts/_general/
  # Events
  if [ -f artifacts/px4_sitl_default/events/all_events.json.xz ]; then
    cp artifacts/px4_sitl_default/events/all_events.json.xz artifacts/_general/
  else
    echo "Error: expected 'artifacts/px4_sitl_default/events/all_events.json.xz' not found." >&2
    exit 1
  fi
  ls -la artifacts/_general/
fi
