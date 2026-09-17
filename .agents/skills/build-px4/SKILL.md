---
name: build-px4
description: Build PX4 board firmware in the px4-dev Docker container, including git worktrees, and stage commit-labeled artifacts without flashing hardware.
---

# Build PX4 Firmware

Use this workflow for flashable board firmware, especially when a macOS host
lacks the NuttX toolchain. Use `px4io/px4-dev` container with the latest available
this container is a native cross-arch container.
It does not supply Gazebo or ROS 2. For Gazebo, use the repository's documented
host setup or a suitable simulator image instead.

## Inputs

- A board target, for example `px4_fmu-v6xrt_default` or
  `cubepilot_cubeorange_default`. Ask for the target if it is missing.
- Optional git refs or existing worktree paths. Without them, build the current
  working tree as-is. Do not silently substitute a clean checkout.

For a POSIX SITL target, use an appropriate SITL workflow instead; do not promise
a flashable `.px4` artifact.

## Prepare

1. Read the build and code-review guidance referenced by `AGENTS.md`.
2. Inspect `git status --short` and `git worktree list`. Validate the target
   against `boards/` and the repository's build configuration.
3. Confirm Docker is available and running. Inspect the image with
   `docker image inspect px4io/px4-dev:v1.17.0`; pull it if missing.
4. Prefer an existing worktree for a requested ref. Otherwise resolve the ref
   (fetch its remote if necessary) and create a new detached worktree under
   `.agents/worktrees/build-<short-sha>/` using `git worktree add --detach`.
   Reuse a path only after confirming its HEAD and working-tree state match the
   request. Never switch the user's active branch.
5. Check submodule status. Initialize required missing submodules, but do not
   overwrite locally modified submodules.
6. Inspect previous build artifacts if moving between macOS and Linux. Remove
   only identified generated host binaries with an incompatible format or a
   confirmed stale `build/<target>/` directory. Do not blindly clean source or
   user files.

## Build

Read `Tools/docker_run.sh` in the checkout being built. Use it if it handles
the git common-directory mount, a writable HOME for the non-root user, and
container git ownership. Otherwise run the equivalent container explicitly.
The checked-in wrapper may not yet provide these worktree requirements.

From the selected worktree root, after assigning `TARGET` to the validated board
target, the explicit form is:

```bash
SRC_DIR="$(pwd -P)"
CCACHE_DIR="${HOME}/.ccache"
GIT_COMMON_DIR="$(git rev-parse --path-format=absolute --git-common-dir)"
mkdir -p "$CCACHE_DIR"

docker run --rm \
  --workdir="$SRC_DIR" \
  --user="$(id -u):$(id -g)" \
  --env=HOME=/tmp \
  --env=CCACHE_DIR="$CCACHE_DIR" \
  --env=GIT_CONFIG_COUNT=1 \
  --env=GIT_CONFIG_KEY_0=safe.directory \
  --env=GIT_CONFIG_VALUE_0='*' \
  --volume="$SRC_DIR:$SRC_DIR:rw" \
  --volume="$GIT_COMMON_DIR:$GIT_COMMON_DIR:rw" \
  --volume="$CCACHE_DIR:$CCACHE_DIR:rw" \
  px4io/px4-dev:v1.17.0 \
  make "$TARGET"
```

Run this with Bash and stop if path resolution or preparation fails. Mounting
the worktree and git common directory at their original absolute paths preserves
git's worktree references. The ownership exception is scoped to this container;
do not change the user's global git configuration.

Preserve the build's exit status. If capturing output through `tee` or `tail`,
enable `set -o pipefail` in that shell. Report failures; do not treat an old
artifact as evidence of success.

## Stage and report

After a successful build, require `build/<target>/<target>.px4` to exist.
Copy it to `/tmp/px4-firmware/` with a distinct name such as
`<target>_<ref-label>_<short-sha>_<timestamp>.px4`. Do not overwrite an existing
artifact. Label dirty-tree builds explicitly: their source is not represented
by the commit alone.

Report the source ref, commit, dirty state, target, absolute artifact path, and
size. Staging is not flashing; never invoke an uploader without a separate user
request. Artifacts under `/tmp` are temporary and may disappear on reboot.
Remove only clean temporary worktrees created for this invocation, using
`git worktree remove`, or report retained worktrees if cleanup is unsafe.
