#!/usr/bin/env bash
#
# metadata_modules.sh — generate and sync PX4 module reference documentation
#
# Usage:
#   Tools/ci/metadata_modules.sh [--test-only] [--debug]
#
# Options:
#   --test-only   Run make target and comparison; exit 1 if diffs found, without copying files
#   --debug       Show full make output and debug info for file comparisons
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
make_target="module_documentation"
src_dir="build/px4_sitl_default/docs/modules"
dest_dir="docs/en/modules"

# Run make target
if [ "$debug" = true ]; then
  echo "🔧 Running 'make $make_target' (verbose)"
  make $make_target
else
  echo "🔧 Running 'make $make_target'"
  make $make_target > /dev/null 2>&1
fi

# Verify build output
src_files=("$src_dir"/*)
if [ ${#src_files[@]} -eq 0 ]; then
  echo "❌ No generated module docs found in $src_dir. Build failed or path wrong."
  exit 1
fi

echo "🔍 Checking module reference docs in $dest_dir"
mkdir -p "$dest_dir"

changed=()
for src in "${src_files[@]}"; do
  name=$(basename "$src")
  dst="$dest_dir/$name"

  if [[ ! -e "$dst" ]]; then
    [ "$debug" = true ] && echo "DEBUG: missing $dst"
    changed+=("$name")
  elif ! cmp -s "$src" "$dst"; then
    [ "$debug" = true ] && echo "DEBUG: cmp -s '$src' '$dst'; echo \$?"
    changed+=("$name")
  fi
done

if [ ${#changed[@]} -eq 0 ]; then
  echo "✅ All module reference docs are up to date."
  exit 0
fi

echo "⚠️ Detected updates in module docs:"
for f in "${changed[@]}"; do echo "  - $f"; done

if [ "$test_only" = true ]; then
  echo "🚨 Module reference docs need updating; rerun without --test-only to apply."
  exit 1
fi

echo "📂 Copying updated module docs to $dest_dir"
for f in "${changed[@]}"; do cp -rv "$src_dir/$f" "$dest_dir/$f"; done

echo "🚨 Module reference docs updated; please commit changes:"
echo "    git status -s $dest_dir"
echo "    git add $dest_dir/*"
echo "    git commit -m 'docs: update module reference metadata'"
exit 1
