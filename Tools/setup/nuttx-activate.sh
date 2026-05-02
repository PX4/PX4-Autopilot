#! /usr/bin/env bash

## Pixi activation hook for the `nuttx` feature: ensures the ARM GNU
## embedded toolchain is installed in `.toolchains/` and prepends it to
## PATH. Sourced (not executed) by `pixi shell` / `pixi run`.

NUTTX_ARM_GCC_VERSION="13.3.rel1"

if [[ -z "${PIXI_PROJECT_ROOT:-}" ]]; then
	return 0 2>/dev/null || exit 0
fi

if [[ ! -x "$PIXI_PROJECT_ROOT/.toolchains/arm-gnu-toolchain/bin/arm-none-eabi-gcc" ]]; then
	"$PIXI_PROJECT_ROOT/Tools/setup/install-arm-gcc.sh" "$NUTTX_ARM_GCC_VERSION"
fi

export PATH="$PIXI_PROJECT_ROOT/.toolchains/arm-gnu-toolchain/bin:$PATH"

# Note: conda-forge's `compilers` activation exports host-targeted CC,
# CXX, CMAKE_ARGS (-DCMAKE_AR=<host-ar>, ...), and *FLAGS. Those break
# NuttX cross-builds — PX4's Toolchain-arm-none-eabi.cmake calls
# `find_program(CMAKE_AR arm-none-eabi-gcc-ar)`, but conda's pre-set
# -DCMAKE_AR=<host-ar> short-circuits it, and the cross link silently
# produces empty archives.
#
# The host vars are NOT unset here, because envs that bundle this
# feature with host-only ones (`ci`, `full`) also run host gtest builds
# (`pixi run -e full test`), which need the conda compilers visible to
# CMake. The unsets are scoped per-build instead — see
# `Tools/setup/pixi-build.sh`, used by the `build` pixi task.
