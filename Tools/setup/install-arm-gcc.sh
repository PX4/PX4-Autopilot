#! /usr/bin/env bash

## Download, verify, and extract the ARM GNU embedded toolchain
## (arm-none-eabi-gcc) for the current host into the workspace.
##
## Used by the pixi `nuttx` feature so cross-compiling NuttX targets does
## not depend on Homebrew, apt, or a system install. Idempotent; re-running
## with an already-installed version is a no-op.
##
## Usage: install-arm-gcc.sh [VERSION]   (default: 13.3.rel1)

set -euo pipefail

VERSION="${1:-13.3.rel1}"

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
ROOT_DIR="$(git -C "$DIR" rev-parse --show-toplevel 2>/dev/null || echo "$DIR/../..")"
TOOLCHAIN_DIR="$ROOT_DIR/.toolchains"
CACHE_DIR="$TOOLCHAIN_DIR/cache"

host_os=$(uname -s)
host_arch=$(uname -m)

case "$host_os/$host_arch" in
	Linux/x86_64)   variant="x86_64-arm-none-eabi" ;;
	Linux/aarch64)  variant="aarch64-arm-none-eabi" ;;
	Darwin/x86_64)  variant="darwin-x86_64-arm-none-eabi" ;;
	Darwin/arm64)   variant="darwin-arm64-arm-none-eabi" ;;
	*)
		echo "[install-arm-gcc] Unsupported host: $host_os/$host_arch" >&2
		exit 1
		;;
esac

# Known-good SHA256 sums per supported version + variant.
sha256_for() {
	local version="$1" variant="$2"
	case "$version/$variant" in
		13.3.rel1/x86_64-arm-none-eabi)        echo "95c011cee430e64dd6087c75c800f04b9c49832cc1000127a92a97f9c8d83af4" ;;
		13.3.rel1/aarch64-arm-none-eabi)       echo "c8824bffd057afce2259f7618254e840715f33523a3d4e4294f471208f976764" ;;
		13.3.rel1/darwin-x86_64-arm-none-eabi) echo "1ab00742d1ed0926e6f227df39d767f8efab46f5250505c29cb81f548222d794" ;;
		13.3.rel1/darwin-arm64-arm-none-eabi)  echo "fb6921db95d345dc7e5e487dd43b745e3a5b4d5c0c7ca4f707347148760317b4" ;;
		*) return 1 ;;
	esac
}

if ! sha256="$(sha256_for "$VERSION" "$variant")"; then
	echo "[install-arm-gcc] No known SHA256 for $VERSION on $variant" >&2
	echo "[install-arm-gcc] Supported versions: 13.3.rel1" >&2
	exit 1
fi

basename="arm-gnu-toolchain-${VERSION}-${variant}"
extracted_dir="$TOOLCHAIN_DIR/$basename"
symlink="$TOOLCHAIN_DIR/arm-gnu-toolchain"
gcc_bin="$extracted_dir/bin/arm-none-eabi-gcc"

if [[ -x "$gcc_bin" && "$(readlink "$symlink" 2>/dev/null || true)" == "$basename" ]]; then
	exit 0
fi

mkdir -p "$CACHE_DIR"
tarball="$CACHE_DIR/${basename}.tar.xz"
url="https://developer.arm.com/-/media/Files/downloads/gnu/${VERSION}/binrel/${basename}.tar.xz"

verify_sha256() {
	local file="$1"
	if command -v shasum >/dev/null 2>&1; then
		shasum -a 256 "$file" | awk '{print $1}'
	elif command -v sha256sum >/dev/null 2>&1; then
		sha256sum "$file" | awk '{print $1}'
	else
		echo "[install-arm-gcc] Neither shasum nor sha256sum available" >&2
		return 1
	fi
}

need_download="false"
if [[ ! -f "$tarball" ]]; then
	need_download="true"
elif [[ "$(verify_sha256 "$tarball")" != "$sha256" ]]; then
	need_download="true"
fi

if [[ "$need_download" == "true" ]]; then
	echo "[install-arm-gcc] Downloading ARM GNU toolchain ${VERSION} for ${variant}..." >&2
	curl -fL --progress-bar -o "$tarball" "$url"
fi

actual="$(verify_sha256 "$tarball")"
if [[ "$actual" != "$sha256" ]]; then
	echo "[install-arm-gcc] SHA256 mismatch for $tarball" >&2
	echo "  expected: $sha256" >&2
	echo "  actual:   $actual" >&2
	exit 1
fi

if [[ ! -x "$gcc_bin" ]]; then
	echo "[install-arm-gcc] Extracting to $extracted_dir..." >&2
	rm -rf "$extracted_dir"
	tar -C "$TOOLCHAIN_DIR" -xJf "$tarball"
fi

ln -sfn "$basename" "$symlink"

echo "[install-arm-gcc] Installed: $("$gcc_bin" --version | head -1)" >&2
