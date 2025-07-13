# MacOS Development Environment

The following instructions set up a PX4 development environment for macOS.
This environment can be used to build PX4 for:

- Pixhawk and other NuttX-based hardware
- [Gazebo Simulation](.._/sim_gazebo_gz/index.md) (Gazebo Harmonic)

:::tip
This setup is supported by the PX4 dev team.
To build other targets you will need to use a [different OS](../dev_setup/dev_env.md#supported-targets) or an [unsupported development environment](../advanced/community_supported_dev_env.md).
:::

## Video Guide

<lite-youtube videoid="tMbMGiMs1cQ" title="Setting up your PX4 development environment on macOS"/>

## Base Setup

The "base" macOS setup installs the tools needed for building firmware, and includes the common tools that will be needed for installing/using the simulators.

### Environment Setup

First set up the environment

1. Enable more open files by appending the following line to the `~/.zshenv` file (creating it if necessary):

   ```sh
   echo ulimit -S -n 2048 >> ~/.zshenv
   ```

   ::: info
   If you don't do this, the build toolchain may report the error: `"LD: too many open files"`
   :::

1. Enforce Python 3 by appending the following lines to `~/.zshenv`

   ```sh
   # Point pip3 to MacOS system python 3 pip
   alias pip3=/usr/bin/pip3
   ```

### Common Tools

To setup the environment to be able to build for Pixhawk/NuttX hardware (and install the common tools for using simulators):

1. Install Homebrew by following these [installation instructions](https://brew.sh).

1. Install the required Python packages:

   ```sh
   # install required packages using pip3
   python3 -m pip install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema
   # if this fails with a permissions error, your Python install is in a system path - use this command instead:
   sudo -H python3 -m pip install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema
   ```

## Gazebo Simulation

To setup the environment for [Gazebo](../sim_gazebo_gz/index.md) simulation with Gazebo Harmonic:

1. Install Gazebo Harmonic and required dependencies:

   ```sh
   brew install gz-harmonic
   brew install qt@5 gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav
   ```

1. Configure Qt5 and library paths by adding the following to your `~/.zshrc` file:

   ```sh
   # Qt5 configuration
   export Qt5_DIR=/opt/homebrew/opt/qt@5/lib/cmake/Qt5
   export PATH="/opt/homebrew/opt/qt@5/bin:$PATH"
   export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:$CMAKE_PREFIX_PATH"

   # Library paths for GStreamer and other dependencies
   export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:$PKG_CONFIG_PATH"
   export LIBRARY_PATH="/opt/homebrew/lib:$LIBRARY_PATH"
   export DYLD_LIBRARY_PATH="/opt/homebrew/lib:$DYLD_LIBRARY_PATH"
   ```

1. Link Qt5 and reload your shell configuration:

   ```sh
   brew unlink qt@5 && brew link --force qt@5
   source ~/.zshrc
   ```

1. Verify Qt5 installation:

   ```sh
   ls -la /opt/homebrew/lib/cmake/Qt5Core/
   ```

1. Run the macOS setup script: `PX4-Autopilot/Tools/setup/macos.sh`
   The easiest way to do this is to clone the PX4 source, and then run the script from the directory, as shown:

   ```sh
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot/Tools/setup
   sh macos.sh
   ```

## Unsupported Simulation Options

### Gazebo Classic

[Gazebo Classic](../sim_gazebo_classic/index.md) is no longer actively supported for new installations. For simulation, we recommend using [Gazebo](../sim_gazebo_gz/index.md) (Gazebo Harmonic) instead.

If you need to use Gazebo Classic for legacy projects:

1. Run these commands in your shell to install the common tools:

   ```sh
   brew tap PX4/px4
   brew install px4-dev
   ```

1. Run the following commands in your shell:

   ```sh
   brew unlink tbb
   sed -i.bak '/disable! date:/s/^/  /; /disable! date:/s/./#/3' $(brew --prefix)/Library/Taps/homebrew/homebrew-core/Formula/tbb@2020.rb
   brew install tbb@2020
   brew link tbb@2020
   ```

   ::: info
   September 2021: The commands above are a workaround to this bug: [PX4-Autopilot#17644](https://github.com/PX4/PX4-Autopilot/issues/17644).
   They can be removed once it is fixed (along with this note).
   :::

1. To install SITL simulation with Gazebo Classic:

   ```sh
   brew install --cask temurin
   brew install --cask xquartz
   brew install px4-sim-gazebo
   ```

## Next Steps

Once you have finished setting up the command-line toolchain:

- Install [VSCode](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).
- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)

  :::tip
  The _daily build_ includes development tools that are hidden in release builds.
  It may also provide access to new PX4 features that are not yet supported in release builds.
  :::

- Continue to the [build instructions](../dev_setup/building_px4.md).
