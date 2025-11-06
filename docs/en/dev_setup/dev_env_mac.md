# macOS Development Environment

The following instructions set up a PX4 development environment for macOS.
This environment can be used to build PX4 for:

- Pixhawk and other NuttX-based hardware
- [Gazebo Simulation](../sim_gazebo_gz/index.md) (Gazebo Harmonic)

::: tip
This setup is supported by the PX4 dev team.
To build for [other targets](../dev_setup/dev_env.md#supported-targets) you will need to use a [different OS](../dev_setup/dev_env.md#supported-targets) or an [unsupported development environment](../advanced/community_supported_dev_env.md).
:::

<!-- ## Video Guide

> **Note:** The video guide previously available here is now outdated. Please follow the updated instructions below for the latest setup process. If you are referencing the YouTube video, be aware that some steps may have changed. -->

## Development Environment Setup

### Prerequisites

1. Install Homebrew by following these [installation instructions](https://brew.sh).

2. Enable more files to be opened in the shell process by appending `echo ulimit -S -n 2048` to your `~/.bashrc` file (or `~/.zshrc` for zsh).
   To append the line using the terminal enter:

   ```sh
   echo ulimit -S -n 2048 >> ~/.bashrc
   ```

   ::: info
   If you don't do this, the build toolchain may report the error: `"LD: too many open files"`
   :::

3. Enforce Python 3 by appending the following lines to `~/.bashrc`(or `~/.zshrc` for zsh):

   Ensure Python 3 is used by default, as some PX4 scripts require the `python3` and `pip3` executables to be available in your system `PATH`.

   ::: tip
   If you need help installing an updated version of Python 3, we recommend the [pyenv project](https://github.com/pyenv/pyenv?tab=readme-ov-file), which gives you utmost flexibility by allowing you to set a global and local Python version at the per-directory level.
   :::

4. Point pip3 to macOS system python 3 pip:

   ```sh
   alias pip3=/usr/bin/pip3
   ```

### Install Development Tools

1. Install the PX4 development toolchain and simulation environment:

   ```sh
   brew tap PX4/px4
   brew install px4-dev px4-sim
   ```

2. Download PX4 Source Code

   ```sh
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   ```

4. Run the macOS setup script.

   Run the [`macos.sh` setup script](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/macos.sh) to automatically install all development dependencies (homebrew packages, python libraries, etc.):

   ```sh
   cd PX4-Autopilot
   ./Tools/setup/macos.sh
   ```


## Gazebo Simulation

Gazebo Harmonic simulation support is included with the `px4-sim` formula installed above.
The simulation environment should be ready to use after the installation completes.

## Next Steps

Once you have finished setting up the command-line toolchain:

- Install [VSCode](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).
- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)

  :::tip
  The _daily build_ includes development tools that are hidden in release builds.
  It may also provide access to new PX4 features that are not yet supported in release builds.
  :::

- Continue to the [build instructions](../dev_setup/building_px4.md).
