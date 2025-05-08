#!/usr/bin/env bash
#
# metadata_parameters.sh — generate and sync PX4 parameter reference documentation
#
# Usage:
#   Tools/ci/metadata_parameters.sh [--test-only] [--debug]
#
# Options:
#   --test-only   Run make target and comparison; exit 1 if diffs found, without copying file
#   --debug       Show full make output and debug info for comparison
#
set -euo pipefail

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

# Paths and make target
make_target="parameters_metadata"
src_file="build/px4_sitl_default/docs/parameters.md"
dest_file="docs/en/advanced_config/parameter_reference.md"

# Run make target
if [ "$debug" = true ]; then
  echo "🔧 Running 'make $make_target' (verbose)"
  make $make_target
else
  echo "🔧 Running 'make $make_target'"
  make $make_target > /dev/null 2>&1
fi

# Verify build output
if [[ ! -f "$src_file" ]]; then
  echo "❌ Generated file not found: $src_file"
  exit 1
fi

echo "🔍 Comparing parameter docs"

# Compare files
if cmp -s "$src_file" "$dest_file"; then
  echo "✅ Parameter reference is up to date."
  exit 0
else
  if [ "$debug" = true ]; then
    echo "DEBUG: cmp -s '$src_file' '$dest_file'; echo \$?"
  fi
  echo "⚠️ Parameter reference needs updating."
  if [ "$test_only" = true ]; then
    exit 1
  fi
  # Copy over updated file
  echo "📂 Copying updated parameter_reference.md"
  cp -v "$src_file" "$dest_file"
  echo "🚨 Parameter docs updated; commit the change:"
  echo "    git status -s $dest_file"
  echo "    git add $dest_file"
  echo "    git commit -m 'docs: update parameter reference metadata'"
  exit 1
fi
