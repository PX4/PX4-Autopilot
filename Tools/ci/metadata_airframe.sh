#!/usr/bin/env bash
#
# metadata_airframe.sh — generate and sync PX4 airframe reference documentation
#
# Usage:
#   Tools/ci/metadata_airframe.sh [--test-only] [--debug]
#
# Options:
#   --test-only   Run make target and comparison; exit 1 if diffs found, without copying file
#   --debug       Show full make output and debug info for comparison
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

# Paths and make target
make_target="airframe_metadata"
src_file="build/px4_sitl_default/docs/airframes.md"
dest_file="docs/en/airframes/airframe_reference.md"

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

echo "🔍 Comparing airframe reference docs"

# Compare files
if cmp -s "$src_file" "$dest_file"; then
  echo "✅ Airframe reference is up to date."
  exit 0
else
  if [ "$debug" = true ]; then
    echo "DEBUG: cmp -s '$src_file' '$dest_file'; echo \$?"
  fi
  echo "⚠️ Airframe reference needs updating."
  if [ "$test_only" = true ]; then
    exit 1
  fi
  # Copy over updated file
  echo "📂 Copying updated airframe_reference.md"
  cp -v "$src_file" "$dest_file"
  echo "🚨 Airframe docs updated; commit the change:"
  echo "    git status -s $dest_file"
  echo "    git add $dest_file"
  echo "    git commit -m 'docs: update airframe reference metadata'"
  exit 1
fi
