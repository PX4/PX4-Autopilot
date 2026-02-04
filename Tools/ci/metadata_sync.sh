#!/usr/bin/env bash
#
# metadata_sync.sh - Unified metadata generation and synchronization for PX4 docs
#
# Usage:
#   Tools/ci/metadata_sync.sh [OPTIONS] [TYPES...]
#
# Types:
#   parameters    - Parameter reference (docs/en/advanced_config/parameter_reference.md)
#   airframes     - Airframe reference (docs/en/airframes/airframe_reference.md)
#   modules       - Module documentation (docs/en/modules/*.md)
#   msg_docs      - uORB message docs (docs/en/msg_docs/*.md + docs/en/middleware/dds_topics.md)
#   uorb_graphs   - uORB graph JSONs (docs/public/middleware/*.json)
#   failsafe_web  - Failsafe simulator (docs/public/config/failsafe/*.{js,wasm,json})
#   all           - All of the above (default)
#
# Options:
#   --generate    Build the make targets to generate fresh metadata
#   --sync        Copy generated files to docs/
#   --verbose     Show detailed output
#   --help        Show this help
#
# Exit codes:
#   0 - Success (files synced or already up-to-date)
#   1 - Error (build failed, missing files, etc.)
#
# Examples:
#   # Full regeneration and sync (orchestrator use case)
#   Tools/ci/metadata_sync.sh --generate --sync all
#
#   # Just sync specific type (assumes already built)
#   Tools/ci/metadata_sync.sh --sync parameters
#
#   # Generate only, don't copy
#   Tools/ci/metadata_sync.sh --generate uorb_graphs
#
set -euo pipefail
shopt -s nullglob

# ═══════════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════════

EMSCRIPTEN_VERSION="3.1.64"
EMSDK_DIR="${EMSDK_DIR:-_emscripten_sdk}"

# All available metadata types
ALL_TYPES=(parameters airframes modules msg_docs uorb_graphs failsafe_web)

# ═══════════════════════════════════════════════════════════════════════════════
# Logging
# ═══════════════════════════════════════════════════════════════════════════════

VERBOSE=false

log() {
    echo "[metadata_sync] $*"
}

log_verbose() {
    if [[ "$VERBOSE" == "true" ]]; then
        echo "[metadata_sync] $*"
    fi
}

die() {
    echo "[metadata_sync] ERROR: $*" >&2
    exit 1
}

# ═══════════════════════════════════════════════════════════════════════════════
# Help
# ═══════════════════════════════════════════════════════════════════════════════

show_help() {
    head -n 35 "$0" | tail -n +2 | sed 's/^# \?//'
    exit 0
}

# ═══════════════════════════════════════════════════════════════════════════════
# Emscripten Setup
# ═══════════════════════════════════════════════════════════════════════════════

ensure_emscripten() {
    if command -v emcc >/dev/null 2>&1; then
        log_verbose "Emscripten already available: $(emcc --version | head -1)"
        return 0
    fi

    log "Setting up Emscripten ${EMSCRIPTEN_VERSION}..."

    if [[ ! -d "$EMSDK_DIR" ]]; then
        log_verbose "Cloning emsdk to $EMSDK_DIR"
        if [[ "$VERBOSE" == "true" ]]; then
            git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_DIR"
        else
            git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_DIR" >/dev/null 2>&1
        fi
    fi

    pushd "$EMSDK_DIR" >/dev/null
    if [[ "$VERBOSE" == "true" ]]; then
        ./emsdk install "$EMSCRIPTEN_VERSION"
        ./emsdk activate "$EMSCRIPTEN_VERSION"
    else
        ./emsdk install "$EMSCRIPTEN_VERSION" >/dev/null 2>&1
        ./emsdk activate "$EMSCRIPTEN_VERSION" >/dev/null 2>&1
    fi
    popd >/dev/null

    # shellcheck source=/dev/null
    source "${EMSDK_DIR}/emsdk_env.sh" >/dev/null 2>&1

    log_verbose "Emscripten ready: $(emcc --version | head -1)"
}

# ═══════════════════════════════════════════════════════════════════════════════
# Generation Functions
# ═══════════════════════════════════════════════════════════════════════════════

generate_parameters() {
    log "Generating parameters metadata..."
    if [[ "$VERBOSE" == "true" ]]; then
        make parameters_metadata
    else
        make parameters_metadata >/dev/null 2>&1
    fi
}

generate_airframes() {
    log "Generating airframes metadata..."
    if [[ "$VERBOSE" == "true" ]]; then
        make airframe_metadata
    else
        make airframe_metadata >/dev/null 2>&1
    fi
}

generate_modules() {
    log "Generating modules documentation..."
    if [[ "$VERBOSE" == "true" ]]; then
        make module_documentation
    else
        make module_documentation >/dev/null 2>&1
    fi
}

generate_msg_docs() {
    log "Generating message documentation..."
    if [[ "$VERBOSE" == "true" ]]; then
        make msg_docs
    else
        make msg_docs >/dev/null 2>&1
    fi
}

generate_uorb_graphs() {
    log "Generating uORB graphs..."
    if [[ "$VERBOSE" == "true" ]]; then
        make uorb_graphs
    else
        make uorb_graphs >/dev/null 2>&1
    fi
}

generate_failsafe_web() {
    ensure_emscripten
    log "Generating failsafe web..."
    if [[ "$VERBOSE" == "true" ]]; then
        make failsafe_web
    else
        make failsafe_web >/dev/null 2>&1
    fi
}

# ═══════════════════════════════════════════════════════════════════════════════
# Sync Functions
# ═══════════════════════════════════════════════════════════════════════════════

sync_parameters() {
    local src="build/px4_sitl_default/docs/parameters.md"
    local dest="docs/en/advanced_config/parameter_reference.md"

    log "Syncing parameters..."

    if [[ ! -f "$src" ]]; then
        die "Source file not found: $src (did you run --generate?)"
    fi

    mkdir -p "$(dirname "$dest")"
    cp "$src" "$dest"
    log_verbose "  $src -> $dest"
}

sync_airframes() {
    local src="build/px4_sitl_default/docs/airframes.md"
    local dest="docs/en/airframes/airframe_reference.md"

    log "Syncing airframes..."

    if [[ ! -f "$src" ]]; then
        die "Source file not found: $src (did you run --generate?)"
    fi

    mkdir -p "$(dirname "$dest")"
    cp "$src" "$dest"
    log_verbose "  $src -> $dest"
}

sync_modules() {
    local src_dir="build/px4_sitl_default/docs/modules"
    local dest_dir="docs/en/modules"

    log "Syncing modules..."

    if [[ ! -d "$src_dir" ]]; then
        die "Source directory not found: $src_dir (did you run --generate?)"
    fi

    local src_files=("$src_dir"/*.md)
    if [[ ${#src_files[@]} -eq 0 ]]; then
        die "No .md files found in $src_dir"
    fi

    mkdir -p "$dest_dir"

    for src in "${src_files[@]}"; do
        local name
        name=$(basename "$src")
        cp "$src" "$dest_dir/$name"
        log_verbose "  $src -> $dest_dir/$name"
    done
}

sync_msg_docs() {
    local src_dir="build/px4_sitl_default/msg_docs"
    local dest_dir="docs/en/msg_docs"
    local middleware_dir="docs/en/middleware"

    log "Syncing message docs..."

    if [[ ! -d "$src_dir" ]]; then
        die "Source directory not found: $src_dir (did you run --generate?)"
    fi

    local src_files=("$src_dir"/*.md)
    if [[ ${#src_files[@]} -eq 0 ]]; then
        die "No .md files found in $src_dir"
    fi

    mkdir -p "$dest_dir"
    mkdir -p "$middleware_dir"

    for src in "${src_files[@]}"; do
        local name
        name=$(basename "$src")

        # dds_topics.md goes to middleware dir
        if [[ "$name" == "dds_topics.md" ]]; then
            cp "$src" "$middleware_dir/$name"
            log_verbose "  $src -> $middleware_dir/$name"
        else
            cp "$src" "$dest_dir/$name"
            log_verbose "  $src -> $dest_dir/$name"
        fi
    done
}

sync_uorb_graphs() {
    local src_dir="Tools/uorb_graph"
    local dest_dir="docs/public/middleware"

    log "Syncing uORB graphs..."

    local src_files=("$src_dir"/*.json)
    if [[ ${#src_files[@]} -eq 0 ]]; then
        die "No .json files found in $src_dir (did you run --generate?)"
    fi

    mkdir -p "$dest_dir"

    for src in "${src_files[@]}"; do
        local name
        name=$(basename "$src")
        cp "$src" "$dest_dir/$name"
        log_verbose "  $src -> $dest_dir/$name"
    done
}

sync_failsafe_web() {
    local src_dir="build/px4_sitl_default_failsafe_web"
    local dest_dir="docs/public/config/failsafe"

    log "Syncing failsafe web..."

    if [[ ! -d "$src_dir" ]]; then
        die "Source directory not found: $src_dir (did you run --generate?)"
    fi

    # Gather js, wasm, json files
    local src_files=()
    for ext in js wasm json; do
        src_files+=("$src_dir"/*."$ext")
    done

    if [[ ${#src_files[@]} -eq 0 ]]; then
        die "No .js/.wasm/.json files found in $src_dir"
    fi

    mkdir -p "$dest_dir"

    for src in "${src_files[@]}"; do
        local name
        name=$(basename "$src")
        cp "$src" "$dest_dir/$name"
        log_verbose "  $src -> $dest_dir/$name"
    done
}

# ═══════════════════════════════════════════════════════════════════════════════
# Main Logic
# ═══════════════════════════════════════════════════════════════════════════════

DO_GENERATE=false
DO_SYNC=false
SELECTED_TYPES=()

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --generate)
                DO_GENERATE=true
                shift
                ;;
            --sync)
                DO_SYNC=true
                shift
                ;;
            --verbose)
                VERBOSE=true
                shift
                ;;
            --help|-h)
                show_help
                ;;
            -*)
                die "Unknown option: $1"
                ;;
            *)
                # It's a type
                SELECTED_TYPES+=("$1")
                shift
                ;;
        esac
    done

    # Default to all types if none specified
    if [[ ${#SELECTED_TYPES[@]} -eq 0 ]]; then
        SELECTED_TYPES=("all")
    fi

    # Expand "all" to all types
    local expanded_types=()
    for t in "${SELECTED_TYPES[@]}"; do
        if [[ "$t" == "all" ]]; then
            expanded_types+=("${ALL_TYPES[@]}")
        else
            expanded_types+=("$t")
        fi
    done
    SELECTED_TYPES=("${expanded_types[@]}")

    # Validate types
    for t in "${SELECTED_TYPES[@]}"; do
        local valid=false
        for valid_type in "${ALL_TYPES[@]}"; do
            if [[ "$t" == "$valid_type" ]]; then
                valid=true
                break
            fi
        done
        if [[ "$valid" == "false" ]]; then
            die "Unknown type: $t (valid: ${ALL_TYPES[*]})"
        fi
    done

    # Must specify at least one action
    if [[ "$DO_GENERATE" == "false" && "$DO_SYNC" == "false" ]]; then
        die "Must specify at least one of: --generate, --sync"
    fi
}

main() {
    parse_args "$@"

    log "Selected types: ${SELECTED_TYPES[*]}"
    [[ "$DO_GENERATE" == "true" ]] && log "Actions: generate"
    [[ "$DO_SYNC" == "true" ]] && log "Actions: sync"

    # Remove duplicates from SELECTED_TYPES
    local -A seen
    local unique_types=()
    for t in "${SELECTED_TYPES[@]}"; do
        if [[ -z "${seen[$t]:-}" ]]; then
            seen[$t]=1
            unique_types+=("$t")
        fi
    done
    SELECTED_TYPES=("${unique_types[@]}")

    # Generate phase
    if [[ "$DO_GENERATE" == "true" ]]; then
        log "=== Generation Phase ==="
        for t in "${SELECTED_TYPES[@]}"; do
            "generate_$t"
        done
    fi

    # Sync phase
    if [[ "$DO_SYNC" == "true" ]]; then
        log "=== Sync Phase ==="
        for t in "${SELECTED_TYPES[@]}"; do
            "sync_$t"
        done
    fi

    log "Done."
    exit 0
}

main "$@"
