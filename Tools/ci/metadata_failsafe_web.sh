#!/usr/bin/env bash
#
# metadata_failsafe_web.sh — build and sync failsafe webapp metadata files
#
# Usage:
#   Tools/ci/metadata_failsafe_web.sh [--test-only] [--debug]
#
# Options:
#   --test-only   Run build and comparison; exit 1 if diffs found, without copying files
#   --debug       Show full build output and debug info for file comparisons and EMSDK install
#
set -euo pipefail
shopt -s nullglob

# Parse flags
test_only=false
debug=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --test-only) test_only=true; shift ;;
    --debug)     debug=true; shift ;;
    *) echo "Usage: $0 [--test-only] [--debug]"; exit 2 ;;
  esac
done

# Paths and commands
build_cmd="make failsafe_web"
src_dir="build/px4_sitl_default_failsafe_web"
dest_dir="docs/public/config/failsafe"

# Ensure Emscripten SDK is available
if ! command -v emcc >/dev/null 2>&1; then
  echo "🔧 Emscripten not found. Ensuring EMSDK is installed."
  # Clone SDK only if not already present
  if [ ! -d "_emscripten_sdk" ]; then
    if [ "$debug" = true ]; then
      git clone https://github.com/emscripten-core/emsdk.git _emscripten_sdk
    else
      git clone https://github.com/emscripten-core/emsdk.git _emscripten_sdk > /dev/null 2>&1
    fi
  fi
  pushd _emscripten_sdk >/dev/null
  if [ "$debug" = true ]; then
    ./emsdk install latest
    ./emsdk activate latest
  else
    ./emsdk install latest > /dev/null 2>&1
    ./emsdk activate latest > /dev/null 2>&1
  fi
  popd >/dev/null
  # Load environment into current shell
  if [ "$debug" = true ]; then
    # shellcheck source=/dev/null
    . ./_emscripten_sdk/emsdk_env.sh
  else
    # shellcheck source=/dev/null
    . ./_emscripten_sdk/emsdk_env.sh > /dev/null 2>&1
  fi
fi

# Build step
if [ "$debug" = true ]; then
  echo "🔧 Running build: $build_cmd"
  $build_cmd
else
  echo "🔧 Running build"
  $build_cmd > /dev/null 2>&1
fi

# Gather built files
src_files=("$src_dir"/*.{js,wasm,json})
if [ ${#src_files[@]} -eq 0 ]; then
  echo "❌ No generated files found in $src_dir. Build failed or path wrong."
  exit 1
fi

# Prepare destination
echo "🔍 Checking failsafe web metadata"
mkdir -p "$dest_dir"

changed=()
for src in "${src_files[@]}"; do
  name=$(basename "$src")
  dst="$dest_dir/$name"

  if [[ ! -f "$dst" ]]; then
    [ "$debug" = true ] && echo "DEBUG: missing $dst"
    changed+=("$name")
  elif ! cmp -s "$src" "$dst"; then
    [ "$debug" = true ] && echo "DEBUG: cmp -s '$src' '$dst'; echo \$?"
    changed+=("$name")
  fi
done

if [ ${#changed[@]} -eq 0 ]; then
  echo "✅ All failsafe web metadata files are in sync."
  exit 0
fi

echo "⚠️ Detected updates in:"
for f in "${changed[@]}"; do echo "  - $f"; done

if [ "$test_only" = true ]; then
  echo "🚨 Failsafe web metadata needs update; rerun without --test-only to apply."
  exit 1
fi

echo "📂 Copying updated files"
for f in "${changed[@]}"; do cp -v "$src_dir/$f" "$dest_dir/$f"; done

echo "🚨 Failsafe web metadata updated; please commit changes."
exit 1
