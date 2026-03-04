#!/usr/bin/env bash
#
# test_metadata_sync.sh - Test metadata_sync.sh locally using Docker
#
# Usage:
#   Tools/ci/test_metadata_sync.sh [OPTIONS] [TYPES...]
#
# Options:
#   --shell       Drop into interactive shell instead of running sync
#   --verbose     Pass --verbose to metadata_sync.sh
#   --skip-build  Skip SITL build (use existing build artifacts)
#   --help        Show this help
#
# Types:
#   Same as metadata_sync.sh: parameters, airframes, modules, msg_docs, uorb_graphs, failsafe_web, all
#
# Examples:
#   # Test full regeneration
#   Tools/ci/test_metadata_sync.sh all
#
#   # Test just parameters (faster)
#   Tools/ci/test_metadata_sync.sh parameters
#
#   # Drop into shell for debugging
#   Tools/ci/test_metadata_sync.sh --shell
#
#   # Skip build if you already have artifacts
#   Tools/ci/test_metadata_sync.sh --skip-build --verbose all
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

DOCKER_IMAGE="px4io/px4-dev:v1.17.0-alpha1"
CONTAINER_NAME="px4-metadata-test-$$"

SHELL_MODE=false
VERBOSE=""
SKIP_BUILD=false
TYPES=()

show_help() {
    head -n 28 "$0" | tail -n +2 | sed 's/^# \?//'
    exit 0
}

cleanup() {
    echo "[test] Cleaning up container..."
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --shell)
                SHELL_MODE=true
                shift
                ;;
            --verbose)
                VERBOSE="--verbose"
                shift
                ;;
            --skip-build)
                SKIP_BUILD=true
                shift
                ;;
            --help|-h)
                show_help
                ;;
            -*)
                echo "Unknown option: $1" >&2
                exit 1
                ;;
            *)
                TYPES+=("$1")
                shift
                ;;
        esac
    done

    # Default to all types
    if [[ ${#TYPES[@]} -eq 0 ]]; then
        TYPES=("all")
    fi
}

main() {
    parse_args "$@"

    cd "$REPO_ROOT"

    echo "[test] Using Docker image: $DOCKER_IMAGE"
    echo "[test] Repository root: $REPO_ROOT"

    # Pull image if not present
    if ! docker image inspect "$DOCKER_IMAGE" >/dev/null 2>&1; then
        echo "[test] Pulling Docker image..."
        docker pull "$DOCKER_IMAGE"
    fi

    trap cleanup EXIT

    # Handle git worktrees: the .git file points to the main repo's .git directory
    # We need to mount that directory too so git works inside the container
    local git_mounts=()
    if [[ -f "$REPO_ROOT/.git" ]]; then
        # It's a worktree - read the gitdir path and mount it
        local gitdir
        gitdir=$(grep '^gitdir:' "$REPO_ROOT/.git" | cut -d' ' -f2)
        if [[ -n "$gitdir" ]]; then
            # Mount the gitdir at the same path so the .git file reference works
            git_mounts+=("-v" "$gitdir:$gitdir:ro")
            # Also need the main .git directory (parent of worktrees/)
            local main_git_dir
            main_git_dir=$(dirname "$(dirname "$gitdir")")
            git_mounts+=("-v" "$main_git_dir:$main_git_dir:ro")
            echo "[test] Detected git worktree, mounting git directories"
        fi
    fi

    if [[ "$SHELL_MODE" == "true" ]]; then
        echo "[test] Starting interactive shell..."
        echo "[test] Run: Tools/ci/metadata_sync.sh --generate --sync all"
        docker run -it --rm \
            --name "$CONTAINER_NAME" \
            -v "$REPO_ROOT:/src" \
            "${git_mounts[@]}" \
            -w /src \
            "$DOCKER_IMAGE" \
            /bin/bash
    else
        echo "[test] Running metadata sync for: ${TYPES[*]}"

        # Build the command
        local cmd=""

        if [[ "$SKIP_BUILD" == "false" ]]; then
            cmd="Tools/ci/metadata_sync.sh --generate --sync $VERBOSE ${TYPES[*]}"
        else
            cmd="Tools/ci/metadata_sync.sh --sync $VERBOSE ${TYPES[*]}"
        fi

        echo "[test] Command: $cmd"

        docker run --rm \
            --name "$CONTAINER_NAME" \
            -v "$REPO_ROOT:/src" \
            "${git_mounts[@]}" \
            -w /src \
            "$DOCKER_IMAGE" \
            /bin/bash -c "$cmd"

        echo ""
        echo "[test] Done! Check git status for changes:"
        echo "  git status -s docs/"
        echo ""
        echo "[test] To see what changed:"
        echo "  git diff docs/"
    fi
}

main "$@"
