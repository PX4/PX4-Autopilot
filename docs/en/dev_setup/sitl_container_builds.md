# Building SITL Containers

The [prebuilt SITL images](../simulation/px4_sitl_prebuilt_packages.md) share the existing `.deb` build and publishing workflow.
`Tools/packaging/containers/docker-bake.hcl` defines the image graph, tags and cache settings; `.github/workflows/build_deb_package.yml` supplies the simulator, architecture and version.

## Source Layout

Files are grouped by responsibility:

```text
Tools/packaging/
  deb/
    gazebo/             Package lifecycle hooks and the installed Gazebo launcher
    sih/                Package lifecycle hooks
  containers/
    Dockerfile.*        SIH, Gazebo and ROS image definitions
    docker-bake.hcl     Connected image build and publishing targets
    prepare_context.sh Build-context staging
    *-entrypoint.sh     Container startup scripts
    px4-network.sh     Shared runtime host routing
    ros2-install-dependencies.sh
    test_sih_mission.py
Tools/ros2/
  prepare_workspace.py Checkout-matched ROS source workspace preparation
  ros2.repos           Shared immutable source pins for images and integration CI
```

The generated `docker-context/` remains a flat staging directory; it is not the source layout.
ROS workspace preparation is separate from packaging because it is also used for development and integration tests.

## Build And Publish Flow

1. The package jobs build SIH and Gazebo `.deb`s for Ubuntu Noble and Jammy.
2. Four native container jobs consume the Noble packages: SIH/Gazebo on amd64/arm64.
   Each job runs `Tools/packaging/containers/prepare_context.sh` to stage the package context and matching PX4 messages, then builds the runtime image and its ROS child together with Docker Bake.
3. On release tags or a manual run with deployment enabled, the jobs push architecture-specific images to Docker Hub and GHCR.
   The deploy jobs combine those images into versioned and `latest` multi-architecture tags using `docker buildx imagetools create`.

Pull requests build the same graph without publishing.
The ROS image consumes its parent directly through a Bake target context, so neither local builds nor pull requests need an unpublished parent tag in a registry.

## Building Locally

Run these commands from the repository root on Ubuntu 24.04 with [PX4 build dependencies](dev_env_linux_ubuntu.md) and Docker Buildx installed:

```sh
make px4_sitl_sih
(cd build/px4_sitl_sih && cpack -G DEB)
mkdir -p docker-context
cp build/px4_sitl_sih/*.deb docker-context/
bash Tools/packaging/containers/prepare_context.sh
SIMULATOR=sih docker buildx bake -f Tools/packaging/containers/docker-bake.hcl --load
```

On macOS or Windows, obtain Noble `.deb`s for the same PX4 revision and Docker architecture instead of building the packages natively, then start at the context-preparation step.
Do not combine packages from different revisions or architectures in one context.
For Gazebo, build `px4_sitl_default` and set `SIMULATOR=gazebo`.

`ARCH` defaults to the Docker builder's local architecture and `VERSION` defaults to `dev`.
For example, an ARM64 SIH build produces `px4io/px4-sitl-ros2:dev-arm64`.
Override `VERSION` and `ARCH` through environment variables when needed.
Replace `--load` with `--print` to inspect the resolved graph without building, or add `--set ros2.tags=px4-ros2:local` to choose a local ROS image tag.

## Cache Boundaries

Every successful `RUN` creates a cacheable layer.
A failed step is not committed as an image layer.
Changing an input invalidates its layer and subsequent layers, not earlier ones.

`Tools/ros2/ros2.repos` pins the ROS repositories to immutable commits for both images and integration tests.
The interface-library pin includes the manual WaitSet ownership fixes merged in [Auterion/px4-ros2-interface-lib#222](https://github.com/Auterion/px4-ros2-interface-lib/pull/222).
To test another interface-library commit in an image, pass `--set ros2.args.PX4_ROS2_REF=<40-hex-SHA>` to Bake.
`PX4_MSGS_REF` can similarly override the message-package metadata; the definitions always come from the PX4 checkout.
The base ROS/rosdep setup, DDS Agent, message-package metadata, interface-library sources, workspace dependency installation, compilation and tests have separate cache boundaries.
Message definitions are copied after dependency installation, so changing them does not repeat `rosdep install`.
Repository checkouts live in a separate source stage and are mounted read-only for dependency installation.
The development toolchain does not inherit those source layers; only the ROS workspace builder copies the checkouts.
Changing only the ROS entrypoint does not rebuild the workspace, and changing only the test command does not recompile it.

The Agent and ROS workspace build in independent Ubuntu 24.04/Jazzy stages, keyed to source pins and the supplied PX4 message definitions rather than the packaged firmware.
A new PX4 binary with unchanged messages reuses those compilation and test layers.
Changed messages invalidate workspace compilation and tests; cached artifacts never substitute messages from another checkout.
The final image still derives from the connected SIH/Gazebo parent and installs its Ubuntu/ROS dependencies using the same helper as the builder.
Only the source-built Agent/logger and complete ROS workspace are copied, preserving the source/build/install layout and absolute symlinks without copying the builder's operating system or package database.
The Agent installation is staged with CMake's `DESTDIR`, so copying it cannot overwrite Python packages installed separately in the destination image.
APT cache mounts are held only during installation, not during compilation or tests.
The runtime Dockerfiles extract the package's `Depends` field into a separate stage, so their dependency layers are keyed to that field rather than to every firmware binary change.
Package jobs disable Ubuntu's container cleanup hook so downloaded APT archives survive installation and can be cached between runs.

In Actions, `CACHE_GHA=true` enables one cache per simulator and architecture.
The ROS target exports `mode=max`, covering its entire graph including the runtime parent.
The compiler cache is persisted separately with `actions/cache` and `buildkit-cache-dance`, allowing unchanged compilation units to be reused when workspace inputs change.
Ccache checks compiler contents and source/header inputs; it does not substitute an old ROS workspace for changed PX4 messages.
The container jobs use eight native CPUs, matching the Dockerfile's `BUILD_JOBS` default.
Override that build argument with Bake's `--set ros2.args.BUILD_JOBS=4` when building on a smaller machine.
Local builds use the builder's regular cache by default; standard Bake `--set` options can select other cache backends.
Cached installs do not automatically refresh upstream packages: rebuild without cache when intentionally refreshing dependencies.

## Updating ROS Source Pins

Source pins are maintained manually through reviewed pull requests, not advanced automatically when images are built.
Update the full commit SHA in `Tools/ros2/ros2.repos` when adopting upstream changes.
The same manifest is consumed by both container builds and ROS integration CI; run both workflows before accepting an update.

The `px4_msgs` pin selects package metadata and build logic, not the message definitions used to build the workspace.
Definitions always come from the PX4 checkout, so ordinary message changes do not require a pin update.
Image and workflow SHA overrides are for testing candidates; adopting a candidate requires updating the shared manifest.

## Testing A PX4 Checkout With ROS

The standalone `ghcr.io/px4/px4-dev-ros2:main-jazzy` image includes Jazzy, the Agent, PX4's source-build dependencies and the dependencies required by the pinned ROS repositories.
It is also available on Docker Hub as `px4io/px4-dev-ros2:main-jazzy`, with matching tags in both registries.
It contains no repository checkouts, packaged PX4 firmware or compiled ROS workspace, and its entrypoint sources only the Jazzy underlay.
It is published independently of the SITL package workflow, so it does not wait for `.deb` builds.
Publishing follows the same release-tag and opt-in manual policy as `px4-dev`.

The `main-jazzy` tag names the manually published toolchain for PX4 `main` development with ROS Jazzy, not bundled firmware.
Pushing a `v*` Git tag publishes `<git-tag>-jazzy`, such as `v1.18.0-jazzy`, without updating `main-jazzy`.
Each publication also provides `sha-<full-PX4-commit>-jazzy`, identifying the repository revision used to build the toolchain.
Release-line toolchains can use tags such as `v1.18-jazzy` through the manual publisher's `tag` input when that release line is maintained.
The source revision and intended PX4 release line do not change which firmware a checkout builds.
ROS integration CI pins the image by digest so publishing a new moving tag cannot silently change the test environment.
When adopting a rebuilt toolchain, update the digest in `.github/workflows/ros_integration_tests.yml`.

The optional `ros2-dev` Bake target builds this image without `.deb` packages or a published SITL parent:

```sh
bash Tools/packaging/containers/prepare_context.sh
docker buildx bake -f Tools/packaging/containers/docker-bake.hcl ros2-dev \
  --set ros2-dev.tags=px4-dev-ros2:local --load
SOURCE=$(pwd -P)
GIT_COMMON_DIR=$(git rev-parse --path-format=absolute --git-common-dir)
docker run --rm -it \
  -v "$SOURCE:$SOURCE" -v "$GIT_COMMON_DIR:$GIT_COMMON_DIR" -w "$SOURCE" \
  -e GIT_CONFIG_COUNT=1 -e GIT_CONFIG_KEY_0=safe.directory \
  -e "GIT_CONFIG_VALUE_0=$SOURCE" px4-dev-ros2:local bash
```

The same absolute mounts support both ordinary checkouts and git worktrees.
The Git ownership exception is scoped to this container and checkout.
Inside the container, build the current checkout; the build initialises its required submodules:

```bash
PX4_DIR=$PWD
make px4_sitl_sih
source /opt/ros/jazzy/setup.bash
python3 Tools/ros2/prepare_workspace.py /opt/px4_ros2_ci
cd /opt/px4_ros2_ci
sudo apt-get update -qq
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
colcon test --packages-select px4_ros2_cpp --ctest-args -R unit_tests
colcon test-result --verbose
source install/setup.bash
cd "$PX4_DIR"
python3 test/ros_test_runner.py --config-file test/ros_tests/config-sih.json \
  --build-dir build/px4_sitl_sih --model quadx \
  --px4-ros2-interface-lib-build-dir /opt/px4_ros2_ci/build/px4_ros2_cpp
```

The workspace helper requires a new or empty directory, imports the standard `ros2.repos` manifest with vcstool, and replaces message/service definitions with those from the current PX4 checkout, including removal of deleted definitions.
Each run imports the pinned repositories directly with vcstool, without reusing an existing source or compiled workspace.
It preserves all library packages, examples and Python tests.
An optional second argument selects another interface-library commit using its full 40-character SHA.
The helper prints the resolved source manifest, repository commits and PX4 commit; it does not build the workspace.

ROS integration CI builds PX4 afresh for each pull request and only sources the Jazzy underlay before building this separate workspace.
Its manual `px4_ros2_ref` input selects a full interface-library commit SHA; leaving it empty uses `ros2.repos`.
CI prepares and builds the ROS workspace on every run using the current checkout's message/service definitions.
Only the compiler cache is restored, not an old source, build or install workspace.
Ccache validates compiler contents and compilation inputs, including changed library sources and generated messages.
`CACHE_GHA=true` uses an independent `ros2-dev-<architecture>` image cache, without replacing the published SITL image caches.
The `ros_dev_container.yml` workflow builds the standalone target on every `main` push and on pull requests that change its watched paths, without publishing.
It publishes to Docker Hub and GHCR only on `v*` tag pushes or manual runs with `deploy_to_registry=true`.
Manual runs use the `tag` input, which defaults to `main-jazzy`; the deployment toggle also applies when dispatching against a Git tag.
Architecture tags use `sha-<full-PX4-commit>-jazzy-<architecture>`; the final selected tag and `sha-<full-PX4-commit>-jazzy` indexes preserve both architectures and their SBOMs.
Use `--set ros2-dev.args.PX4_ROS2_REF=<40-hex-SHA>` to build the development image with another interface-library commit.

Both ROS development images and the CI toolchain reuse `Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools` and its Python requirements.
Already installed Ubuntu/Jazzy Python versions are constrained during installation instead of gratuitously upgrading the ROS stack.
The SIH image does not acquire Gazebo or the NuttX toolchain.
Installing the complete source-build toolset increases cold image-build time and size; existing native architecture and incremental cache boundaries remain in use.

## Container SBOMs

The publishing workflow enables BuildKit's standard SBOM attestations for both runtime and ROS images.
It pins BuildKit v0.33.0, whose 80 MiB attestation limit accommodates the Gazebo/ROS SPDX documents without reducing their package or file coverage.
Local attested builds using BuildKit v0.32 can fail at export because that version limits each attestation to 40 MiB.
These SPDX inventories describe discoverable packages in the image filesystem, including Ubuntu and ROS dependencies.
They complement, rather than replace, PX4's [source and firmware SBOM](../contribute/sbom.md).

Each architecture is published with its attestation.
The final `imagetools create` step preserves those attestations when assembling multi-architecture indexes.
Inspect a published image with:

```sh
docker buildx imagetools inspect px4io/px4-sitl-ros2:<tag> --format '{{json .SBOM}}'
```

Non-publishing pull-request builds do not publish an SBOM for inspection.
Local `--load` builds are intended for running examples; use a registry or an OCI export when retaining attestations.
