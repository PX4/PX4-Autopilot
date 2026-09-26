#!/usr/bin/env bash

# Check out the Homebrew repos the macOS CI install reads, at the commits
# recorded next to this script. `brew install` otherwise follows the live
# package list and ignores a checkout.

set -euo pipefail

DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export HOMEBREW_NO_AUTO_UPDATE=1
export HOMEBREW_NO_INSTALL_FROM_API=1

if ! command -v brew >/dev/null 2>&1; then
	echo "macos_ci_pins.sh: brew is not installed" >&2
	exit 1
fi

first_field() {
	local file="$1"
	local n="$2"
	grep -v -e '^#' -e '^[[:space:]]*$' "$file" | head -n 1 | awk -v n="$n" '{print $n}'
}

pin_sha() {
	local name="$1"
	grep -E "^${name}[[:space:]]+" "${DIR}/homebrew-pins.txt" | awk '{print $2}'
}

fetch_and_checkout() {
	local path="$1"
	local sha="$2"

	if [[ -z "$sha" ]]; then
		echo "macos_ci_pins.sh: no commit for ${path}" >&2
		exit 1
	fi

	echo "macos_ci_pins.sh: ${path} ${sha}"
	git -C "$path" fetch -q --depth 1 origin "$sha"
	git -C "$path" checkout -q -f --detach "$sha"
}

# The Homebrew install itself is already a git repo on the runner. Do not
# recreate that directory: it is the prefix every package lives under.
checkout_brew() {
	local sha="$1"

	if [[ ! -d "${BREW_REPO}/.git" ]]; then
		echo "macos_ci_pins.sh: ${BREW_REPO} is not a git repo" >&2
		exit 1
	fi
	fetch_and_checkout "$BREW_REPO" "$sha"
}

clone_at() {
	local path="$1"
	local url="$2"
	local sha="$3"

	if [[ ! -d "${path}/.git" ]]; then
		rm -rf "$path"
		mkdir -p "$(dirname "$path")"
		git init -q "$path"
		git -C "$path" remote add origin "$url"
	fi
	fetch_and_checkout "$path" "$sha"
}

BREW_REPO=$(brew --repo)
TAPS="${BREW_REPO}/Library/Taps"
CORE_SHA=$(first_field "${DIR}/protobuf-pin.txt" 1)
GZ_SHA=$(first_field "${DIR}/gz-tap-pin.txt" 1)

checkout_brew "$(pin_sha brew)"
clone_at "${TAPS}/homebrew/homebrew-core" "https://github.com/Homebrew/homebrew-core" "$CORE_SHA"
clone_at "${TAPS}/osrf/homebrew-simulation" "https://github.com/osrf/homebrew-simulation" "$GZ_SHA"
clone_at "${TAPS}/osx-cross/homebrew-arm" "https://github.com/osx-cross/homebrew-arm" "$(pin_sha osx-cross/arm)"
clone_at "${TAPS}/px4/homebrew-px4" "https://github.com/PX4/homebrew-px4" "$(pin_sha px4/px4)"
