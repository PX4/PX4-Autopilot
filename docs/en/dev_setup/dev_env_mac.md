# macOS Development Environment

The following instructions set up a PX4 development environment on macOS.
This environment can be used to build PX4 for:

- Pixhawk and other NuttX-based hardware
- [Gazebo Simulation](../sim_gazebo_gz/index.md) (Gazebo Harmonic)

It works on both Intel and Apple Silicon Macs.

:::tip
This setup is supported by the PX4 dev team.
To build for [other targets](../dev_setup/dev_env.md#supported-targets) you will need to use a [different OS](../dev_setup/dev_env.md#supported-targets) or an [unsupported development environment](../advanced/community_supported_dev_env.md).
:::

## Development Environment Setup

### Prerequisites

1. **Install Xcode Command Line Tools** — provides `git`, `make`, and the Apple `clang` compiler:

   ```sh
   xcode-select --install
   ```

2. **Install Homebrew** by following the [installation instructions](https://brew.sh).

3. **Increase the open-file limit.** The PX4 build opens many files simultaneously and the macOS default limit (256) is too low — you may see `"LD: too many open files"` errors without this.

   Add the following line to your shell startup file so it applies to every new terminal session.
   macOS defaults to **zsh** since Catalina, so add it to `~/.zshrc` (use `~/.bashrc` if you use bash):

   ```sh
   echo "ulimit -S -n 2048" >> ~/.zshrc
   ```

   Then **open a new terminal** (or run `source ~/.zshrc`) for the change to take effect.

4. **Ensure Python 3 is available.** Some PX4 build scripts require `python3` and `pip3` to be in your `PATH`. The Xcode Command Line Tools include Python 3 by default.

   :::tip
   If you need to install or manage a different Python version, we recommend [pyenv](https://github.com/pyenv/pyenv), which lets you set global and per-directory Python versions.
   :::

### Install Development Tools

1. **Download PX4 Source Code:**

   ```sh
   git clone https://github.com/PX4/PX4-Autopilot.git
   cd PX4-Autopilot
   git submodule update --init --recursive --force
   ```

2. **Install development environment libraries** from the [macos.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/macos.sh) helper script:

   ```sh
   ./Tools/setup/macos.sh --sim-tools
   ```

   This installs:
   - **`px4-dev`** — ARM cross-compiler (`arm-gcc-bin@13`), `cmake`, `ninja`, `ccache`, and other build tools
   - **Python packages** from `requirements.txt`
   - **`px4-sim`** (via `--sim-tools`) — Gazebo Harmonic simulation (`gz-harmonic`) and related tools

   ::: info
   Omit `--sim-tools` if you only need to build for NuttX hardware and don't need simulation.

   Use `--reinstall` to force reinstallation of all Homebrew formulas (useful if something is broken).
   :::

### Gazebo Simulation

The `--sim-tools` flag installs the `px4-sim` Homebrew formula, which pulls in Gazebo Harmonic.

If you skipped `--sim-tools` during initial setup and want to add simulation later:

```sh
brew tap PX4/px4
brew install px4-sim
```

::: info
Gazebo requires **XQuartz** for display on macOS.
If you don't already have it installed:

```sh
brew install --cask xquartz
```

You may need to log out and back in after installing XQuartz.
:::

### Verify Installation

After installation, verify the key tools are available:

```sh
# NuttX cross-compiler (from arm-gcc-bin@13)
arm-none-eabi-gcc --version

# Build tools
cmake --version
ninja --version

# Gazebo (if --sim-tools was used)
gz sim --versions
```

Quick smoke test — build and run a simulation target:

```sh
make px4_sitl gz_x500
```

If everything is set up correctly, this will build PX4 SITL and launch a Gazebo simulation with the x500 quadcopter.

## Next Steps

Once you have finished setting up the command-line toolchain:

- Install [VSCode](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).
- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)

  ::: tip
  The _daily build_ includes development tools that are hidden in release builds.
  It may also provide access to new PX4 features that are not yet supported in release builds.
  :::

- Continue to the [build instructions](../dev_setup/building_px4.md).
