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

is_nuttx_target() {
	local t="$1"
	[[ -z "$t" ]] && return 1

	# PX4 accepts two target forms:
	#   <vendor>_<board>           → boards/<vendor>/<board>/
	#   <vendor>_<board>_<label>   → boards/<vendor>/<board>/  (with <label>.px4board)
	# Board names may contain '_' (e.g. saam/saampixv1_1), so try the
	# 3-segment form first and fall back to the 2-segment shorthand.
	# Toolchain is only declared in default.px4board; variant labels
	# (rover, mavlink-dev, ...) inherit it, so we always read default.
	local vendor="${t%%_*}"
	local rest="${t#*_}"
	local board_dir=""

	if [[ "$rest" == *_* ]]; then
		local board="${rest%_*}"
		[[ -d "boards/${vendor}/${board}" ]] && board_dir="boards/${vendor}/${board}"
	fi
	[[ -z "$board_dir" && -d "boards/${vendor}/${rest}" ]] && board_dir="boards/${vendor}/${rest}"
	[[ -z "$board_dir" ]] && return 1

	local file="${board_dir}/default.px4board"
	[[ -f "$file" ]] && grep -q 'CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"' "$file"
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
