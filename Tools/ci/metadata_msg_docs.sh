#!/usr/bin/env bash
#
# metadata_msg_docs.sh — generate and sync uORB message reference documentation
#
# Usage:
#   Tools/ci/metadata_msg_docs.sh [--test-only] [--debug]
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
make_target="msg_docs"
src_dir="build/msg_docs"
dest_dir="docs/en/msg_docs"

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
  echo "❌ No files found in $src_dir. Build target '$make_target' failed or path is wrong."
  exit 1
fi

echo "🔍 Checking uORB message docs in $dest_dir"
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
  echo "✅ All uORB message docs are up to date."
  exit 0
fi

echo "⚠️ Detected updates in the following docs:"
for f in "${changed[@]}"; do echo "  - $f"; done

if [ "$test_only" = true ]; then
  echo "🚨 uORB message docs need updating! Rerun without --test-only to apply changes."
  exit 1
fi

echo "📂 Copying updated doc files to $dest_dir"
for f in "${changed[@]}"; do cp -v "$src_dir/$f" "$dest_dir/$f"; done

echo "🚨 uORB message docs updated; please commit changes:"
echo "    git status -s $dest_dir"
echo "    git add $dest_dir/*"
echo "    git commit -m 'docs: update uORB message reference docs'"
exit 1
