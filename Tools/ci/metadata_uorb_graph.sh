#!/usr/bin/env bash
#
# update_uorb_graphs.sh — generate, compare, and sync uORB graph JSONs
#
# Usage:
#   ./scripts/update_uorb_graphs.sh [--test-only] [--debug]
#
# Options:
#   --test-only   Run generation and comparison only; exit 1 if diffs found, without copying files
#   --debug       Echo debug info for missing or differing files and show full make output
#
# Examples:
#   # CI mode: fail if docs need updates
#   ./scripts/update_uorb_graphs.sh --test-only
#
#   # Developer mode: regenerate and sync JSONs
#   ./scripts/update_uorb_graphs.sh
#
set -euo pipefail

shopt -s nullglob

# Parse flags
test_only=false
debug=false
while [[ $# -gt 0 ]]; do
	case "$1" in
		--test-only)
			test_only=true
			shift
			;;
		--debug)
			debug=true
			shift
			;;
		*)
			echo "Usage: $0 [--test-only] [--debug]"
			exit 2
			;;
	esac
done

# Paths
graph_dir="Tools/uorb_graph"
dest_dir="docs/public/middleware"

# Generate uORB graphs (conditionally silent)
if [ "$debug" = true ]; then
	echo "🔧 Generating uORB message graphs (verbose output)"
	make uorb_graphs
else
	echo "🔧 Generating uORB message graphs"
	make uorb_graphs > /dev/null 2>&1
fi

# Verify generation
src_files=("$graph_dir"/*.json)
if [ ${#src_files[@]} -eq 0 ]; then
	echo "❌ No JSON files found in $graph_dir. Generation failed or path is wrong."
	exit 1
fi

echo "🔍 Checking for updated uORB graph JSONs"
mkdir -p "$dest_dir"

changed=()
for src in "${src_files[@]}"; do
	name=$(basename "$src")
	dst="$dest_dir/$name"

	if [[ ! -f "$dst" ]]; then
		[ "$debug" = true ] && echo "DEBUG: $dst missing"
		changed+=("$name")

	elif ! cmp -s "$src" "$dst"; then
		[ "$debug" = true ] && echo "DEBUG: cmp -s '$src' '$dst'; echo \$?"
		changed+=("$name")
	fi
done

if [ ${#changed[@]} -eq 0 ]; then
	echo "✅ All uORB graph JSONs are already in sync."
	exit 0
fi

echo "⚠️ Detected updates in the following files:"
for name in "${changed[@]}"; do
	echo "	- $name"
done

if [ "$test_only" = true ]; then
	echo
	echo "🚨 uORB graph docs need updating! Rerun without --test-only to apply changes."
	exit 1
fi

echo
echo "📂 Copying updated files over"
for name in "${changed[@]}"; do
	cp -v "$graph_dir/$name" "$dest_dir/$name"
done

echo
echo "🚨 uORB graph docs need updating! Review above, then run:"
echo "	git status -s $dest_dir/"
echo "	git add $dest_dir/*.json"
echo "	git commit -m 'docs: metadata: update uORB graph JSONs'"
exit 1
