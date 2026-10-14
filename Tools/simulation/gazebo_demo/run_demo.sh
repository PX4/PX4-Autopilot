#!/usr/bin/env bash
set -e

DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$DEMO_DIR/../../.." && pwd)"

if [[ -f "$REPO_ROOT/.venv/bin/activate" ]]; then
	source "$REPO_ROOT/.venv/bin/activate"
fi

exec python3 "$DEMO_DIR/demo.py" "$@"
