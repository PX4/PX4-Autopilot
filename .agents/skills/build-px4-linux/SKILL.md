---
name: build-px4-linux
description: Build PX4 firmware or SITL natively on a Linux host, in the current checkout, without flashing hardware.
---

# Build PX4 on Linux

## Inputs

- A make target, for example `px4_fmu-v6x_default` or `px4_sitl`. Ask for it
  if missing; validate it against `boards/`.

Build the current checkout as-is, worktree or not. Do not create a worktree
or switch branches to build.

## Prepare

1. `arm-none-eabi-gcc --version` must succeed for NuttX targets. If it does
   not, the host needs `Tools/setup/ubuntu.sh` (Ubuntu) — tell the user rather
   than installing it, or build in the container:
   `docker run --rm -w "$PWD" --user "$(id -u):$(id -g)" -v "$PWD:$PWD" px4io/px4-dev:v1.17.0 make <target>`.
   `Tools/docker_run.sh` passes `-it` and fails without a TTY.
2. The build initializes submodules itself; do not overwrite locally modified
   ones.

## Build

```bash
set -o pipefail
make <target> 2>&1 | tail -n 40
```

Report failures with the output; never treat an existing artifact as
evidence of success. After changing a defconfig or Kconfig option, or
switching toolchains, `rm -rf build/<target>` first: a stale build dir
recompiles nothing and links mixed objects.

Do not run a NuttX build concurrently with `make tests` or SITL in the same
tree.

## Report

The artifact is `build/<target>/<target>.px4` (NuttX) or
`build/px4_sitl_default/bin/px4` (SITL). Report the ref, commit, dirty state,
target, and artifact path. Building is not flashing; never upload without a
separate request.
