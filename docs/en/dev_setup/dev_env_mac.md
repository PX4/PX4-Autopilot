# macOS Development Environment

The following instructions set up a PX4 development environment for macOS.
This environment can be used to build PX4 for:

- Pixhawk and other NuttX-based hardware
- [Gazebo Simulation](../sim_gazebo_gz/index.md) (Gazebo Harmonic)

:::tip
This setup is supported by the PX4 dev team.
To build other targets you will need to use a [different OS](../dev_setup/dev_env.md#supported-targets) or an [unsupported development environment](../advanced/community_supported_dev_env.md).
:::

## Video Guide

<lite-youtube videoid="tMbMGiMs1cQ" title="Setting up your PX4 development environment on macOS"/>

## Development Environment Setup

### Prerequisites

1. Install Homebrew by following these [installation instructions](https://brew.sh).

2. Enable more open files by appending the following line to the `~/.bashrc` file (creating it if necessary):

   ```sh
   echo ulimit -S -n 2048 >> ~/.bashrc
   ```

   ::: info
   If you don't do this, the build toolchain may report the error: `"LD: too many open files"`
   :::

3. Enforce Python 3 by appending the following lines to `~/.bashrc`:

   ```sh
   # Point pip3 to macOS system python 3 pip
   alias pip3=/usr/bin/pip3
   ```

### Install Development Tools

1. Install the PX4 development toolchain and simulation environment:

   ```sh
   brew tap PX4/px4
   brew install px4-dev px4-sim
   ```

2. **Important:** Link the ARM cross-compiler manually (required step):

   ```sh
   brew link --overwrite --force arm-gcc-bin@13
   ```

   ::: warning
   Homebrew does not link versioned formulae by default, so this manual linking step is required for the ARM cross-compiler (v13) to function properly.
   If you have other versions of `arm-none-eabi-gcc` installed, this may override them.
   You can unlink it manually later with `brew unlink arm-gcc-bin@13` if needed.
   :::

3. Install the required Python packages:

   ```sh
   # Install required packages using pip3
   python3 -m pip install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema
   # If this fails with a permissions error, your Python install is in a system path - use this command instead:
   sudo -H python3 -m pip install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema
   ```

### Download PX4 Source Code

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
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
