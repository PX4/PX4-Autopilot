# macOS Development Environment

The following instructions set up a PX4 development environment on macOS.
This environment can be used to build PX4 for:

- Pixhawk and other NuttX-based hardware
- [Gazebo Simulation](../sim_gazebo_gz/index.md) (Gazebo Harmonic)

It works on both Intel and Apple Silicon Macs.
PX4 CI exercises this setup on Apple Silicon runners only; Intel is not covered by CI.

::: tip
This setup is supported by the PX4 dev team.
To build for [other targets](../dev_setup/dev_env.md#supported-targets) you will need to use a [different OS](../dev_setup/dev_env.md#supported-targets) or an [unsupported development environment](../advanced/community_supported_dev_env.md).
:::

## Development Environment Setup

### Prerequisites

1. **Install Xcode Command Line Tools**, which provide `git`, `make`, and the Apple `clang` compiler:

   ```sh
   xcode-select --install
   ```

2. **Install Homebrew** by following the [installation instructions](https://brew.sh).
   The setup script below also installs Homebrew if it is missing.

3. **Increase the open-file limit.** The PX4 build opens many files simultaneously and the macOS default limit (256) is too low. You may see `"LD: too many open files"` errors without this.

   Add the following line to your shell startup file so it applies to every new terminal session.
   macOS defaults to **zsh** since Catalina, so add it to `~/.zshrc` (use `~/.bashrc` if you use bash):

   ```sh
   echo "ulimit -S -n 2048" >> ~/.zshrc
   ```

   Then **open a new terminal** (or run `source ~/.zshrc`) for the change to take effect.

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

   ::: info
   The setup script creates a Python virtual environment at `.venv` in the repo root and installs all Python dependencies into it. This keeps PX4's Python requirements isolated from your system Python and avoids conflicts with Homebrew's externally-managed Python.

   Activate it before building:

   ```sh
   source .venv/bin/activate
   ```

   You'll need to re-run this command in each new terminal session. To activate it automatically when you `cd` into the repo, consider a tool like [direnv](https://direnv.net/) or add the activation to your `~/.zshrc`.
   :::

   The script installs the NuttX cross-compiler and build tools, the Python dependencies (into the `.venv` described above), and with `--sim-tools` the Gazebo simulation stack.
   It is the source of truth for what gets installed; read [macos.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/macos.sh) for the details.

   ::: info
   Omit `--sim-tools` if you only need to build for NuttX hardware and don't need simulation.
   All Gazebo dependencies are optional at build time, so `make px4_sitl` still works without them.

   Use `--reinstall` to force reinstallation of the Homebrew formulas (useful if something is broken).
   :::

   ::: info
   The script installs from third-party Homebrew taps and marks them as trusted (`brew trust`) on Homebrew 6.0 and later, which refuses to load formulae from untrusted taps.
   With `--sim-tools` it will prompt for your password, since the XQuartz installer and the JDK link into `/Library/Java/JavaVirtualMachines` need `sudo`.
   :::

### Gazebo Simulation

The `--sim-tools` flag installs Gazebo and the libraries PX4's simulation modules build against.

If you skipped `--sim-tools` during initial setup and want to add simulation later, re-run the setup script with the flag (it is safe to run repeatedly):

```sh
./Tools/setup/macos.sh --sim-tools
```

::: info
The script also installs **XQuartz**.
macOS may require you to log out and back in after XQuartz is first installed.
:::

### Verify Installation

After installation, verify the key tools are available:

```sh
# NuttX cross-compiler
arm-none-eabi-gcc --version

# Build tools
cmake --version
ninja --version

# Gazebo (if --sim-tools was used)
gz sim --versions
```

As a quick smoke test, build and run a simulation target:

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
