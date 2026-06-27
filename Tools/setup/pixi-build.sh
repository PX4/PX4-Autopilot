#! /usr/bin/env bash

## Wrapper for the pixi `build` task.
##
## NuttX firmware targets need the conda host-compiler env (CC, CXX,
## CMAKE_ARGS, *FLAGS) cleared so PX4's
## cmake/toolchains/Toolchain-arm-none-eabi.cmake can resolve the cross
## tools (arm-none-eabi-gcc, -gcc-ar, -ranlib, ...) by name. Without
## this, conda's CMAKE_ARGS pre-sets -DCMAKE_AR=<host-ar>, the
## toolchain file's `find_program(CMAKE_AR arm-none-eabi-gcc-ar)` short-
## circuits, and the cross link silently produces empty archives.
##
## Host (POSIX/SITL) targets need the conda env intact so CMake picks up
## arm64-apple-darwin20.0.0-clang(++) instead of falling back to Apple's
## /usr/bin/c++, which trips -Wdouble-promotion in abseil headers.
##
## We dispatch by reading the target's .px4board file and looking for
## `CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"`.

set -e

target="${1:-}"

# Resolve a make target to its .px4board file the way the build itself does
# (cmake/px4_config.cmake): every boards/<vendor>/<model>/<label>.px4board
# maps to the target name <vendor>_<model>_<label> with path separators
# replaced by '_', and the '_default' label may be omitted on the command
# line. We enumerate the board files and match the generated name instead of
# splitting the target on '_', because vendor and model names themselves
# contain underscores (e.g. boards/saam/saampixv1_1).
px4board_for_target() {
	local t="$1" f name
	[[ -z "$t" ]] && return 1
	while IFS= read -r f; do
		name="${f#boards/}"
		name="${name%.px4board}"
		name="${name//\//_}"
		if [[ "$name" == "$t" || "$name" == "${t}_default" ]]; then
			printf '%s\n' "$f"
			return 0
		fi
	done < <(find boards -mindepth 3 -maxdepth 3 -name '*.px4board')
	return 1
}

read_toolchain() {
	sed -n 's/^CONFIG_BOARD_TOOLCHAIN="\(.*\)"/\1/p' "$1"
}

# A target needs the ARM cross compiler if its resolved .px4board selects the
# arm-none-eabi toolchain. This mirrors how cmake/kconfig.cmake derives the
# toolchain: it merges default.px4board + <label>.px4board and reads
# CONFIG_BOARD_TOOLCHAIN (-> CMAKE_TOOLCHAIN_FILE Toolchain-<name>). Some
# variant labels (rover, mavlink-dev, ...) don't declare a toolchain and
# inherit it from default.px4board, so fall back to that when the variant
# file is silent.
is_nuttx_target() {
	local file toolchain default_file
	file="$(px4board_for_target "$1")" || return 1

	toolchain="$(read_toolchain "$file")"
	if [[ -z "$toolchain" ]]; then
		default_file="${file%/*}/default.px4board"
		[[ -f "$default_file" ]] && toolchain="$(read_toolchain "$default_file")"
	fi

	[[ "$toolchain" == "arm-none-eabi" ]]
}

if is_nuttx_target "$target"; then
	unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS LDFLAGS_LD
	unset DEBUG_CFLAGS DEBUG_CXXFLAGS DEBUG_CPPFLAGS DEBUG_LDFLAGS
	unset FFLAGS FCFLAGS DEBUG_FFLAGS DEBUG_FORTRANFLAGS
	unset CC CXX FC OBJC CPP LD AR AS RANLIB NM NMEDIT STRIP LIPO LIBTOOL
	unset INSTALL_NAME_TOOL OTOOL CHECKSYMS PAGESTUFF REDO_PREBINDING
	unset SEG_ADDR_TABLE SEG_HACK SEGEDIT SIZE STRINGS CLANG
	unset CC_FOR_BUILD CXX_FOR_BUILD FC_FOR_BUILD OBJC_FOR_BUILD CPP_FOR_BUILD
	unset CMAKE_ARGS MESON_ARGS
fi

exec make "$@"
