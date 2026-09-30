#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
repo_dir="$(cd "$script_dir/../../.." && pwd -P)"
if [[ "${1:-}" == "--messages-only" ]]; then
    if [[ $# -ne 2 || -z "$2" ]]; then
        echo "Usage: $0 --messages-only PX4_MSGS_DIR" >&2
        exit 1
    fi
    messages_dir="$2"
else
    context_dir="${1:-docker-context}"
    messages_dir="$context_dir/px4_msgs"
    mkdir -p "$context_dir/setup"
    cp "$script_dir"/Dockerfile.* "$script_dir"/*entrypoint.sh "$script_dir/px4-network.sh" "$context_dir/"
    cp "$script_dir/ros2-install-dependencies.sh" "$repo_dir/Tools/ros2/ros2.repos" "$context_dir/"
    cp "$script_dir/ros2-agent.repos" "$context_dir/"
    cp "$repo_dir/Tools/setup/ubuntu.sh" "$repo_dir/Tools/setup/requirements.txt" "$context_dir/setup/"
fi

# Refresh generated definitions when reusing a context; keep downloaded .debs.
mkdir -p "$messages_dir/msg" "$messages_dir/srv"
for destination in "$messages_dir/msg" "$messages_dir/srv"; do
    destination="$(cd "$destination" && pwd -P)"
    case "$destination/" in
        "$repo_dir/msg/"*|"$repo_dir/srv/"*)
            echo "Refusing to overwrite PX4 source definitions: $destination" >&2
            exit 1
            ;;
    esac
done
rm -f "$messages_dir/msg/"*.msg "$messages_dir/srv/"*.srv
cp "$repo_dir"/msg/*.msg "$repo_dir"/msg/versioned/*.msg "$messages_dir/msg/"
cp "$repo_dir"/srv/*.srv "$messages_dir/srv/"
