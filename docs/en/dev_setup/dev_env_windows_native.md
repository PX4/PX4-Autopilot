# Windows Native Development Environment (CMD/PowerShell)

<Badge type="tip" text="PX4 v1.18" /><Badge type="warning" text="Experimental" />

::: warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env.md).
It may or may not work with current versions of PX4.

The [Windows WSL2-Based Development Environment](../dev_setup/dev_env_windows_wsl.md) should be used by preference for most developers.
The native toolchain is intended for contributors working on Windows-specific platform code, and for users who need to run [SIH](../sim_sih/index.md) without a Linux VM.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

The following instructions explain how to set up a native (non-WSL) PX4 development environment on Windows 10 or 11, building directly from CMD or PowerShell.

This environment can be used to build PX4 for:

- [SIH SITL Simulation](../sim_sih/index.md) — the only simulator whose dynamics run **inside** `px4.exe`, so no external simulator process is needed on Windows.
- [Pixhawk and other NuttX-based hardware](../dev_setup/building_px4.md#nuttx-pixhawk-based-boards) (NuttX cross-toolchain installed separately).

::: warning
This development environment does not support Gazebo, Gazebo Classic, jMAVSim, or any other process that relies on Linux "out of the box".
You can however [run them separately from WSL](#running-non-sih-simulators-from-wsl-or-a-remote-linux-host) and connect to the Windows instance.
:::

## Overview

PX4 SITL is built natively on Windows using the same `make` command that is used on Linux and macOS: `make px4_sitl_default`.
The command builds the executable `build\px4_sitl_default\bin\px4.exe` (just like on the other platforms), which you launch directly from CMD or PowerShell.

Companion utilities (`px4-commander`, `px4-listener`, etc.) are created as additional `.exe` files alongside `px4.exe`.

### Choosing the C++ Toolchain

Two compilers are supported:

- **MSVC** (Microsoft Visual C++, from Visual Studio 2022 or the Build Tools) — the default and the recommended choice on Windows.
  MSVC is Microsoft's native Windows toolchain: it ships with the Windows SDK, produces binaries against the canonical Windows ABI, and is the toolchain the broader Windows C++ ecosystem (debugger, profilers, libraries) is built and tested against, so it gives the cleanest native Windows experience.
- **MinGW-w64** (the GCC port shipped with [MSYS2](https://www.msys2.org/)) — an alternative for users who prefer a GCC-style toolchain, want to reproduce a Linux MinGW cross-build, or want to keep the whole workflow inside plain _PowerShell_ (the MSVC path needs the _x64 Native Tools Command Prompt_).

Both are tested by the [`Windows SITL build` CI workflow](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/compile_windows.yml) on every PR, so a successful CI run confirms that both produce a working `px4.exe`.

The [setup script](#install-the-toolchain) installs MSVC by default; pass `-Toolchain MinGW` to install MinGW instead, or `-Toolchain Both` to install both.

::: info
On Windows, PX4 daemon listens on a local TCP socket (`127.0.0.1:14680 + instance_id`), instead of the Unix-domain socket used on Linux/macOS.
The behaviour is functionally equivalent — you can drive PX4 from the interactive `pxh>` prompt as on Linux, or invoke the per-command client wrappers (`px4-commander`, `px4-listener`, `px4-shutdown`, …) from any separate _PowerShell_ window.
:::

## Installation

### Preconditions

A first-time install takes roughly **30–60 minutes** end-to-end on a typical broadband connection, most of it unattended download time.
Before you start, you need:

- _Windows 10_ (21H2 or newer) or _Windows 11_, with the [winget](https://learn.microsoft.com/en-us/windows/package-manager/winget/) command available.
  This ships with the OS as part of _App Installer_ (if `winget` is not recognised in a fresh terminal, install [App Installer](https://apps.microsoft.com/store/detail/9NBLGGH4NNS1) from the Microsoft Store).
- About **10 GB** of free disk space for the toolchain (most of which is consumed by the Visual Studio 2022 Build Tools).
- Administrator rights on the machine — the Build Tools installer writes to `C:\Program Files`.
- The `git` tool.
  The instructions in [Get the PX4 Source Code](#get-the-px4-source-code) install _Git for Windows_ if needed.

The setup script lives inside the PX4 repository, so the source tree must be cloned **before** the toolchain script can be run.
Note that everything (other than `git`) is installed by the script: CMake, Ninja, Python, MSVC Build Tools, MSYS2, and so on.

### Choosing a Shell

This guide uses just two command-line shells — the table below shows which to use for each task.

| Task                                          | Shell                                                           |
| --------------------------------------------- | --------------------------------------------------------------- |
| Toolchain install (`Tools\setup\windows.ps1`) | _Windows PowerShell_ **as Administrator**                       |
| MinGW builds                                  | _Windows PowerShell_ (regular, no Administrator)                |
| MSVC builds                                   | _x64 Native Tools Command Prompt for VS 2022_ (a _CMD_ variant) |
| Running SIH, `.ps1` launchers, everyday `git` | _Windows PowerShell_ (regular, no Administrator)                |

::: info
You should not need (or use) _Git Bash_ or _MSYS2 MinGW64 Shell_ for this guide.

- _Git Bash_ works for `git` but adds Unix-shell quirks (forward-slash paths, different quoting, no `$env:` syntax) that these instructions are not written against — stick with _PowerShell_ to avoid surprises.
- The setup script generates wrapper `.cmd` shims for the MinGW toolchain so you can build from plain _PowerShell_ without ever opening the _MSYS2_ shell yourself.

:::

### Get the PX4 Source Code

To get the source code we clone the PX4-Autopilot repository:

1. Open a _PowerShell_ window (Press **Start**, type _PowerShell_, press **Enter**):
2. Install _Git for Windows_ (if it isn't already installed):

   ```sh
   winget install --id Git.Git -e --source winget
   ```

3. Close and reopen the _PowerShell_ window so the new `git` command is on `PATH`.
4. Clone the repository into a directory you have rights over (for example `C:\PX4` or any local folder you choose):

   ```sh
   cd C:\PX4\
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   ```

   ::: details A few things to keep in mind about _where_ you clone
   - **Keep the path short**: Windows still defaults to 260-char paths (MAX_PATH) and PX4 generates deep nested directories (especially under `external/Install/...`), so keep the source tree shallow.
   - **Prefer a path without spaces**: a few build tools have edge cases with quoted paths — `C:\Users\Some User\Documents\PX4-Autopilot` is risky, while `C:\PX4` or `D:\src\PX4-Autopilot` is safer.
   - **Avoid OneDrive-synchronized folders**: synced trees (the default `Documents` is often one) will fight the build for hundreds of thousands of intermediate object files.
     Check for the OneDrive cloud icon next to the folder in File Explorer, or run `Get-Item <path> | Select-Object Target` (a redirected folder has a `Target` set); if it is synced, clone into a local-only path instead.

   :::

   `cd` does not print the new path on success — run `pwd` if you want to confirm where you landed.
   PX4 build commands accept either backslash (`C:\PX4\...`) or forward-slash (`C:/PX4/...`) paths interchangeably.

::: info
PX4 ships a `.gitattributes` that pins line endings, so no `core.autocrlf` configuration is required.
If you have an existing global `core.autocrlf=true`, set it to `false` for this repository:

```sh
git -C PX4-Autopilot config core.autocrlf false
```

:::

### Install the Toolchain

The recommended way to install the toolchain is to use the `windows.ps1` setup script:

::: tip
Re-running the script is safe — it skips packages that are already installed.
:::

1. Open _Windows PowerShell_ **as an Administrator**
   (Press **Start**, type _PowerShell_, right-click _Windows PowerShell_, choose **Run as administrator**).

2. Navigate to your cloned `PX4-Autopilot` directory and run:

   ```sh
   cd PX4-Autopilot
   Set-ExecutionPolicy -Scope Process Bypass
   .\Tools\setup\windows.ps1
   ```

   ::: info
   `Set-ExecutionPolicy -Scope Process Bypass` is safe.
   `-Scope Process` confines the change to this PowerShell window and is forgotten when you close it, leaving the system-wide policy untouched.
   :::

3. Run the installation script to installed your desired toolchain.

   Visual Studio 2022 Build Tools:

   ```sh
   .\Tools\setup\windows.ps1
   ```

   MSYS2 only

   ```sh
   .\Tools\setup\windows.ps1 -Toolchain MinGW
   ```

   Both compilers

   ```sh
   .\Tools\setup\windows.ps1 -Toolchain Both
   ```

   ::: info
   The Visual Studio 2022 Build Tools installer downloads roughly 4 GB and runs **silently** for **10–15 minutes** — the script will look frozen during this stage but is working.
   Leave the window open until the script returns to the prompt; closing it mid-install can leave the Build Tools in a broken state that has to be repaired from _Add or remove programs_.
   :::

The script uses [winget](https://learn.microsoft.com/en-us/windows/package-manager/winget/) to install:

- _Python 3.11_, _CMake_, _Ninja_
- _GNU make_ (via [Chocolatey](https://chocolatey.org/) if Chocolatey is already installed, otherwise via [ezwinports](https://sourceforge.net/projects/ezwinports/))
- _Visual Studio 2022 Build Tools_ with the `Desktop development with C++` workload and the Windows 11 SDK
- The Python build-time packages (`jinja2`, `pyyaml`, `toml`, `numpy`, `packaging`, `jsonschema`, `future`, `empy`, `pyros-genmsg`, `kconfiglib`)

_Git for Windows_ is needed earlier to clone the repository and is left untouched if you already have it.

::: info
The script appends `C:\msys64\mingw64\bin` to your **user** `PATH` and also exposes it in the current PowerShell process, so the MinGW build works in the same shell that ran the setup script.
Any other shells you had open before running the script will not see the new `PATH` until they are closed and reopened.
:::

### Verify the Install

Open a **new** _PowerShell_ window (no need to elevate) so it picks up the `PATH` entries that the setup script registered, then check that the four commands the build relies on are all on `PATH`:

```sh
git --version
python --version
cmake --version
make --version
```

Each command should print a version number.
If any of them prints _"is not recognized as the name of a cmdlet…"_, see [Common Pitfalls](#common-pitfalls) below.

## Build PX4 SITL

Pick the subsection that matches the toolchain you installed (see [Choosing the C++ Toolchain](#choosing-the-c-toolchain) above for the rationale).

### MSVC Build

Open the _x64 Native Tools Command Prompt for VS 2022_ (see the [shell table](#choosing-a-shell) above) — the MSVC compiler is only on `PATH` inside a Visual Studio developer shell.
If you prefer to keep working in _PowerShell_, you can source the developer environment into an existing window with `vcvars64.bat` instead (see [`cl.exe` Not Found](#cl-exe-not-found-msvc) below).

1. Open the _x64 Native Tools Command Prompt for VS 2022_ (**Start** menu > _Visual Studio 2022_ > _Visual Studio Tools_ > VC > **x64 Native Tools Command Prompt for VS 2022**.)
2. Navigate to the repository root (`PX4-Autopilot`)
3. Set `PATH` to prepend Git Bash's POSIX utilities (`sed`, `awk`, `dirname`, `[`, `ps`) to the _x64 Native Tools Command Prompt_ `PATH`:

   ```sh
   set PATH=C:\Program Files\Git\usr\bin;C:\Program Files\Git\bin;%PATH%
   make px4_sitl_default
   ```

   ::: info
   The _x64 Native Tools Command Prompt_ provides the MSVC compiler but **not** the POSIX utilities installed with _Git for Windows_ that the PX4 Makefile depends on.
   This line adds them to the path.
   :::

A successful **first** build takes 5–15 minutes depending on the machine; CMake stays silent for the first couple of minutes while it configures the project and downloads upstream dependencies — this is normal, not a hang.
The build ends with a line of the form `[NNN/NNN] Linking CXX executable bin\px4.exe`.
Subsequent rebuilds in the same shell are incremental and finish in seconds.

::: info
The PX4 Makefile picks the Ninja generator when `ninja` is on `PATH` and lets CMake auto-detect `cl.exe` from the developer shell — no `-DCMAKE_TOOLCHAIN_FILE` flag is needed.
:::

### MinGW-w64 Build

Open a regular _PowerShell_ window (see the [shell table](#choosing-a-shell) above — _not_ the Visual Studio developer shell, and not _MSYS2 MinGW64 Shell_).
After the setup script finishes with `-Toolchain MinGW`, `C:\msys64\mingw64\bin` is on your user `PATH`, so plain _PowerShell_ can drive the MinGW toolchain through the wrapper `.cmd` shims the toolchain file generates:

```sh
cd PX4-Autopilot
$env:CMAKE_ARGS = "-DCMAKE_TOOLCHAIN_FILE=Toolchain-mingw-w64-x86_64"
make px4_sitl_default
```

If you ran the setup script in a different window than the one you build from, **close and reopen** the build shell so it picks up the updated user `PATH`.

The MinGW build statically links `libgcc`, `libstdc++`, and `winpthread`, so the resulting `px4.exe` has no MSYS2 runtime dependency.

A successful first build takes 5–15 minutes depending on the machine and ends with a line of the form:

```sh
[2/2] Linking CXX executable bin\px4.exe
```

The output is `build\px4_sitl_default\bin\px4.exe` plus the per-command client executables (`build\px4_sitl_default\bin\px4-commander.exe`, etc.).
Subsequent rebuilds in the same shell are incremental and finish in seconds.

::: info
You only need to run `Tools\setup\windows.ps1` and pick the developer shell **once** per machine.
For day-to-day work, just open the developer shell, `cd PX4-Autopilot`, and run `make px4_sitl_default` (or any other make target) — there is no need to re-elevate or re-run the setup script.
:::

## Run SIH on Native Windows

[SIH](../sim_sih/index.md) runs entirely inside `px4.exe` — its rigid-body dynamics are compiled into the autopilot binary, so no external simulator process is required on Windows.

The simplest way to launch a quadrotor instance is the same `make` target used on Linux/macOS, which sets `PX4_SIM_MODEL`/`PX4_SIMULATOR` and runs `px4.exe` for you:

```sh
make px4_sitl_default sihsim_quadx
```

Other airframes are exposed as sibling targets.
All SIH variants share the same Windows code path — physics run inside `px4.exe` and no Linux-only helper process is required — so anything that builds also launches natively.

For the full list of supported vehicles and the corresponding make targets / `PX4_SIM_MODEL` values, see the [SIH Simulation > Supported Vehicle Types](../sim_sih/index.md#supported-vehicle-types) table.

If you would rather invoke `px4.exe` directly (e.g. from a shortcut, a debugger, or a script that does not have `make` on `PATH`), set the environment variable yourself.
You can do this from either _PowerShell_ or _CMD_ — both are shown below; pick whichever shell you already have open.
The relative paths assume your current directory is the `PX4-Autopilot` repository root (the same directory you ran `make px4_sitl_default` from); adjust them if you launch from elsewhere.

From _PowerShell_:

```sh
cd PX4-Autopilot
$env:PX4_SIM_MODEL = "sihsim_quadx"
build\px4_sitl_default\bin\px4.exe build\px4_sitl_default\etc
```

The same command from _CMD_:

```sh
cd PX4-Autopilot
set PX4_SIM_MODEL=sihsim_quadx
build\px4_sitl_default\bin\px4.exe build\px4_sitl_default\etc
```

The path after `px4.exe` is the positional `<rootfs_directory>` argument — the directory where the startup files and mixers live (see `px4.exe -h` for the full usage).
`px4.exe` then brings up an interactive `pxh>` prompt in the same console.
Type a command and press Enter to dispatch it; type `exit`, `shutdown`, or press **CTRL+C** to stop PX4.
If the prompt is unresponsive (or the instance was started with `-d` and has no prompt), shut it down from a second _PowerShell_ window with `build\px4_sitl_default\bin\px4-shutdown.exe`, or as a last resort `Get-Process px4 | Stop-Process -Force` to kill every running `px4.exe`.

::: tip
Pass `-d` (daemon mode) before the rootfs path to suppress the `pxh>` prompt — useful when launching PX4 from a script or running multiple background instances.
:::

::: info
The Windows console does not provide tab completion, history navigation with arrow keys, or ANSI line editing — those features rely on Unix terminal APIs that are not available in the native Windows shell loop.
:::

A healthy first run prints the PX4 ASCII banner, then progresses through `INFO [px4] Startup script returned successfully`, opens the `pxh>` prompt, and starts emitting periodic `INFO [mavlink] partner IP: 127.0.0.1` lines once a ground station connects.
The repeating MAVLink _partner IP_ messages are the normal heartbeat exchange — they are not errors.

Logs land in `log\<YYYY-MM-DD>\<HH_MM_SS>.ulg` **relative to the working directory** of the `px4.exe` process — if you launched it from the repository root, look for the `log\` folder at the repository root; if you used the multi-instance helper below, each instance writes into its own `build\px4_sitl_default\instance_<N>\log\` tree.
To clean build artefacts, run `make clean` (per-board CMake clean) or `make distclean` (wipe the entire `build\` directory and reset submodules) from the repository root; deleting the `build\px4_sitl_default\` directory by hand has the same effect as the latter for a single board.
Per-instance log/parameter directories under `build\px4_sitl_default\instance_<N>\` are not touched by `make clean` — remove them manually when they grow stale.

To launch multiple instances (e.g. for multi-vehicle testing), use the bundled PowerShell helper, which mirrors `Tools/simulation/sitl_multiple_run.sh`.
Run it from the `PX4-Autopilot` repository root so the relative `.\Tools\simulation\` path resolves:

```sh
cd PX4-Autopilot
.\Tools\simulation\sitl_multiple_run.ps1 -SitlNum 3 -SimModel sihsim_quadx
```

::: info
Up to **3** concurrent instances are validated to run reliably on Windows today.
Higher counts (5+) typically see most instances exit silently shortly after `rcS` finishes.
This is being tracked in the multi-instance lifecycle work; until that lands, treat 3 as the supported envelope.
:::

Pass any of the SIH model names from the [vehicle table above](#run-sih-on-native-windows) (e.g. `sihsim_airplane`, `sihsim_standard_vtol`, `sihsim_hex`, `sihsim_rover_ackermann`) via `-SimModel`.
To stop every `px4.exe` started by the helper (e.g. when several backgrounded daemon instances are still running), invoke it with `-SitlNum 0`:

```sh
.\Tools\simulation\sitl_multiple_run.ps1 -SitlNum 0
```

### Tested Configurations

What has been validated to work on native Windows today:

- Single `px4.exe` instance at any `PX4_SIM_SPEED_FACTOR` (1x and accelerated).
- Mixed speed factors across instances (when running ≤3 of them).
- MAVLink heartbeat to QGroundControl on `127.0.0.1` (autoconnect).
- The interactive `pxh>` stdin prompt and `CTRL+C` / `shutdown` graceful exit.
- `px4-*` client wrappers (`px4-commander`, `px4-listener`, `px4-shutdown`, …) from a separate _PowerShell_ window.

Known limitations (being tracked, not yet fixed):

- 5+ concurrent instances via `sitl_multiple_run.ps1`; up to 3 is the supported envelope.

## Connecting QGroundControl

If you don't already have QGroundControl, install it first via the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md) page — the steps below assume it is already running on the same host.

When PX4 SITL runs natively on Windows, [QGroundControl](https://qgroundcontrol.com) on the same host autoconnects out of the box — no manual comm-link setup is required (unlike the [WSL2 workflow](../dev_setup/dev_env_windows_wsl.md#qgroundcontrol-on-windows), which has to bridge across the WSL virtual machine).

::: info
On the first launch of `px4.exe`, _Windows Defender Firewall_ raises a per-app prompt titled _Windows Security Alert_ with two checkboxes — _Private networks_ and _Public networks_.
Tick **Private networks** and click **Allow access** so MAVLink replies from QGC can reach PX4; if you dismiss the prompt, allow it after the fact under **Windows Security > Firewall & network protection > Allow an app through firewall**.
:::

## Running Non-SIH Simulators from WSL or a Remote Linux Host

Gazebo, Gazebo Classic, and jMAVSim depend on a Linux host (or, for jMAVSim, a JDK + `ant` install) for the simulator process itself, so the `make px4_sitl gz_x500` / `make px4_sitl jmavsim` / `make px4_sitl gazebo-classic_iris` convenience targets are not wired up natively on Windows.

You can still drive any of them from a natively-built `px4.exe`: start the simulator inside [WSL2](../dev_setup/dev_env_windows_wsl.md) (or on a remote Linux machine), then launch `px4.exe` on the Windows side with `PX4_SIM_HOSTNAME` (or `PX4_SIM_HOST_ADDR`) pointing at that host.
`simulator_mavlink` will connect to the external simulator over TCP/UDP just like it does on Linux — only the "spawn the simulator for you" wrappers are missing.

## ROS 2 Setup on Windows Native

This section is only relevant if you want to drive PX4 SITL from [ROS 2](../ros2/user_guide.md) on the same Windows host using the OSRF Windows binary release.
If you only need SIH plus QGroundControl, you can skip this section and the next.

::: info
ROS 2 on Windows is community-supported by OSRF — _not_ a first-class platform like Ubuntu Linux.
If you would rather run ROS 2 the way it is most commonly used in PX4 workflows, install it under [WSL2](../dev_setup/dev_env_windows_wsl.md) and run your ROS 2 nodes there while `px4.exe` runs natively on Windows; the [agent](#building-the-micro-xrce-dds-agent-optional-for-ros-2-dds-bridging) just needs to be reachable on UDP and works the same either way.
The instructions below cover the all-native path for users who want to keep everything on the Windows host.
:::

Starting with **ROS 2 Jazzy**, the OSRF Windows binary release is distributed as a [Pixi](https://pixi.sh/) / `conda-forge` bundle rather than the legacy Visual Studio 2019 + Chocolatey install used for Humble and Iron.
PX4 ships a helper that wraps the Pixi-based install end-to-end:

```powershell
cd PX4-Autopilot
.\Tools\setup\ros2_pixi_setup.ps1                  # default: Jazzy LTS
.\Tools\setup\ros2_pixi_setup.ps1 -Distro rolling  # ROS 2 Rolling
.\Tools\setup\ros2_pixi_setup.ps1 -Distro latest   # newest non-prerelease release
```

Pick **Jazzy** for a stable LTS (recommended), **Rolling** for the latest features (less stable), or **latest** as a shorthand for whichever release OSRF tagged most recently.

The script is idempotent and runs without Administrator privileges. It:

- resolves the matching ROS 2 binary release zip from the [`ros2/ros2` GitHub releases](https://github.com/ros2/ros2/releases), downloads it to `$env:TEMP` (cached; pass `-Force` to re-download), and extracts it to `C:\opt\ros\<distro>\` (override with `-RosRoot`),
- installs `pixi` to `%USERPROFILE%\.pixi\bin` (no admin required),
- runs `pixi install` against the extracted distro,
- runs OSRF's `preinstall_setup_windows.py` post-install step (mandatory — over a thousand release `.py` files have build-time hardcoded shebangs that need rewriting to point at the Pixi env's `python.exe`), and
- emits a per-distro activation wrapper at `$env:TEMP\activate_ros2_<distro>.ps1` plus a generic `$env:TEMP\activate_ros2.ps1` alias pointing at the most recently installed distro.

Dot-source the wrapper in any future _PowerShell_ session to put `ros2`, `colcon`, and `python` on `PATH`:

```powershell
. $env:TEMP\activate_ros2.ps1                # most recently installed distro
. $env:TEMP\activate_ros2_jazzy.ps1          # specific distro
. $env:TEMP\activate_ros2.ps1 -WithVcvars    # ros2 CLI + MSVC vcvars64 for `colcon build`
```

`-WithVcvars` is required when `colcon build` has to invoke `cl.exe` — `conda-forge` does not ship MSVC, so a `colcon build` without a sourced `vcvars64.bat` fails with `cl.exe is not recognized`.

::: warning
When launching multi-instance PX4 from a session that has activated ROS 2, pass `-w <work_dir>` to each `px4.exe` and pre-create an `etc` junction inside it; otherwise the instances collide on the shared rootfs.
:::

::: warning
Pass YAML payloads that contain whitespace (e.g. to `Start-Process -ArgumentList`) as a single quoted string rather than an inline `{...}` literal — PowerShell flattens the latter and the agent rejects the malformed message.
:::

Once ROS 2 is up, the [Micro XRCE-DDS Agent](#building-the-micro-xrce-dds-agent-optional-for-ros-2-dds-bridging) section below builds the bridge that connects PX4 SITL to your ROS 2 graph.
See the [uXRCE-DDS middleware page](../middleware/uxrce_dds.md) for the wider workflow, message conventions, and `px4_msgs` setup.

## Building the Micro-XRCE-DDS Agent (Optional — for ROS 2 / DDS bridging)

This section is only relevant if you want to bridge PX4 SITL to ROS 2 (or any other DDS subscriber) via the [uXRCE-DDS middleware](../middleware/uxrce_dds.md).
The agent is a separate process that PX4's `uxrce_dds_client` connects to over UDP; without it, ROS 2 nodes have no way to see PX4's uORB topics.
If you only need SIH plus QGroundControl, you can safely skip this section.

The upstream [eProsima Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) builds out of the box on Linux but hits two Windows-specific issues in its CMake superbuild that need a workaround.
The two-stage build below is a known-good recipe — it produces the same `MicroXRCEAgent.exe` binary you would get from the one-shot build on Linux.

### Prerequisites

- The same _Visual Studio 2022 Build Tools_ install you already use for `make px4_sitl_default` — no extra components required.
  The agent build is **MSVC-only**: upstream eProsima targets MSVC on Windows and the MinGW toolchain is not a supported configuration for the agent (PX4 SITL itself still builds fine with MinGW).
- A real `ninja-build` from `winget install Ninja-build.Ninja` (verified with 1.13.2 or newer).

  ::: warning
  Do **not** use the `ninja` package from `pip install ninja`.
  The pip-distributed Ninja triggers the recompaction issue described in [Known Limitations](#known-limitations) deterministically, even if the rest of the workaround is in place.

  If you may have previously `pip install`-ed Ninja, run `where.exe ninja` from the build shell first.
  Whichever directory comes first on `PATH` wins; if a Python `Scripts\ninja.exe` resolves before the winget install, either uninstall the pip version (`python -m pip uninstall ninja`) or move `C:\Program Files\Ninja\` ahead of the Python `Scripts\` directory in your user `PATH`.
  :::

- Git, CMake, and Python 3 — already on `PATH` from the main [Install the Toolchain](#install-the-toolchain) step above; nothing extra to install.

All `cmake` commands below must be run from a shell where the MSVC environment is loaded — either the _x64 Native Tools Command Prompt for VS 2022_ or a regular _PowerShell_ window into which you have sourced `vcvars64.bat` (see [`cl.exe` Not Found (MSVC)](#cl-exe-not-found-msvc) for the exact one-liner).

### Clone the Agent Repository

Pick a directory outside the `PX4-Autopilot` tree (the agent is independent of PX4 and lives on its own release cadence) — anywhere you have rights over will do.
The rest of this section uses `$agentRoot` as a placeholder; substitute with your chosen path (e.g. `C:\opt\Micro-XRCE-DDS-Agent`):

```powershell
$agentRoot = "C:\opt\Micro-XRCE-DDS-Agent"
git clone --recursive https://github.com/eProsima/Micro-XRCE-DDS-Agent.git $agentRoot
cd $agentRoot
```

### Stage 1 — Build the Dependencies

The agent's CMake project ships a _superbuild_ that, in one configure step, fetches and builds its four direct dependencies (`foonathan_memory`, `fastcdr`, `spdlog`, `fastdds`) and then the agent itself.
On Windows the dependency phase succeeds, but the subsequent inner reconfigure of the `uagent` ExternalProject reuses the outer build directory and fails to recompact `build.ninja` while the parent `ninja` still has it open.
The pragmatic workaround is to let the superbuild install the four dependencies and then build the agent in a separate directory in [Stage 2](#stage-2-build-the-agent-standalone).

From the agent root, in a shell with `vcvars64.bat` loaded:

```powershell
cmake -G Ninja -S . -B build `
    -DCMAKE_BUILD_TYPE=Release `
    -DCMAKE_INSTALL_PREFIX="$agentRoot\install" `
    -DUAGENT_P2P_PROFILE=OFF `
    -DUAGENT_BUILD_EXECUTABLE=ON
cmake --build build --target install
```

`-DUAGENT_P2P_PROFILE=OFF` is required: the peer-to-peer profile pulls in the `microxrcedds_client` ExternalProject, whose own ninja recompaction step fails on Windows for the same reason.
PX4's `uxrce_dds_client` does not use the P2P feature, so disabling it has no functional impact on the SITL ↔ ROS 2 use case.

::: info
This stage takes around **10 minutes** on a typical workstation — most of it is fastdds, which compiles a large amount of generated IDL code.
:::

The build will report success for the four dependencies and then fail when it reaches the `uagent` step with output similar to:

```sh
ninja: error: failed recompaction: Permission denied
```

**That failure is expected.**
Confirm that the four dependency installs landed under `$agentRoot\install\` (`install\include\fastcdr\`, `install\lib\foonathan_memory\`, etc.) before moving on.
If those directories are missing, the dependency phase did not actually succeed and re-running Stage 2 will not recover.

### Stage 2 — Build the Agent Standalone

With the dependencies installed under `$agentRoot\install`, configure a fresh build directory that points the agent's CMake at the now-prebuilt deps and skips the superbuild orchestration:

```powershell
cmake -G Ninja -S . -B build_uagent `
    -DUAGENT_SUPERBUILD=OFF `
    -DCMAKE_PREFIX_PATH="$agentRoot\install" `
    -DCMAKE_INSTALL_PREFIX="$agentRoot\install" `
    -DUAGENT_BUILD_EXECUTABLE=ON
cmake --build build_uagent --target install
```

`-DUAGENT_SUPERBUILD=OFF` flips the agent's top-level `CMakeLists.txt` from "fetch + build everything" mode to "I have all the deps already — just build me", which sidesteps the inner ExternalProject reconfigure that hit `build.ninja` in Stage 1.
`CMAKE_PREFIX_PATH` is how Stage 2 finds the `fastcdr`, `fastdds`, `foonathan_memory`, and `spdlog` packages that Stage 1 installed.

This stage takes under a minute on most machines.

### Verifying a Successful Build

When Stage 2 finishes, the install tree should contain:

- `install\bin\MicroXRCEAgent.exe` — the agent itself.
- `install\bin\fastcdr-2.2.dll` and `install\bin\fastdds-3.6.dll` (or whatever the current upstream version numbers are) staged alongside the executable.
  These are loaded at startup by `MicroXRCEAgent.exe` and the agent will fail to launch with a Windows error dialog if they are missing.

If the DLLs are not in `install\bin\`, copy them over from `install\lib\` — different upstream tags occasionally install them under `lib\` only.

### Running the Agent for SITL

Start the agent listening on UDP 8888 (the default that PX4's `uxrce_dds_client` connects to) from any _PowerShell_ window — `vcvars64` is **not** required for running, only for building:

```powershell
& "$agentRoot\install\bin\MicroXRCEAgent.exe" udp4 -p 8888 -v 6
```

`-v 6` selects the most verbose log level, which is useful while you are first verifying that PX4 connects; lower it (`-v 4` or omit the flag) once everything is working.

Then, in a separate _PowerShell_ window, launch PX4 SITL as usual (`make px4_sitl_default sihsim_quadx`).
Within a few seconds, the agent window should print `create_client | session_id: 0x...` and the matching ROS 2 / DDS topics will appear in your DDS subscriber.

For the wider ROS 2 / DDS workflow on top of this connection (workspace layout, `px4_msgs`, namespacing), see the [uXRCE-DDS bridge documentation](../middleware/uxrce_dds.md) — the agent built here is the same binary that those instructions assume, just compiled natively on Windows.

### Known Limitations

- **Two-stage build is mandatory on Windows.** The upstream superbuild reuses the outer build directory when reconfiguring the `uagent` ExternalProject, and the parent `ninja` process still holds `build.ninja` open at that point.
  Ninja then fails its recompaction step with `Permission denied` (Windows file-locking semantics — POSIX `unlink()`-while-open does not have an equivalent here).
  Track upstream progress at [eProsima/Micro-XRCE-DDS-Agent issues](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues); until a fix lands, the two-stage workaround above is the supported path.
- **Use the winget-installed Ninja.** The `ninja` package distributed via `pip` reproduces the recompaction failure deterministically even with the workaround applied.
  `winget install Ninja-build.Ninja` ships a build that handles the lock contention gracefully on the dependency phase, which is what makes the two-stage recipe viable in the first place.
- **`UAGENT_P2P_PROFILE=ON` is not supported on Windows today.** The peer-to-peer profile triggers a separate, earlier recompaction failure inside the `microxrcedds_client` ExternalProject.
  PX4's SITL bridge does not use P2P, so leaving it off is safe.

## Next Steps

Once you have finished setting up the command-line toolchain:

- Install [VS Code](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).
- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md).
- Continue to [Building PX4 Software](../dev_setup/building_px4.md).

## Troubleshooting

### Common Pitfalls

- **A freshly-installed command is _"not recognized"_ in the same shell that ran the setup script.**

  The setup script updates the user `PATH` for the current process, but Windows does not propagate registry `PATH` changes into shells that were already open.
  **Close every terminal window and open a new one** — including any IDE-integrated terminals — and the missing command will resolve.

- **`winget` itself prints _"is not recognized"_.**

  _App Installer_ is missing or out of date.
  Open the Microsoft Store, search for _App Installer_, install or update it, then reopen the terminal.

- **MSVC builds abort early with `'sed' is not recognized`**

  The _x64 Native Tools Command Prompt_ provides the MSVC compiler but **not** the POSIX utilities the PX4 Makefile depends on.
  You need to set the PATH before calling `make px4_sitl_default` (this is covered in the build instructions).

- **The build appears to hang for several minutes the first time.**

  CMake configures the project and downloads several upstream dependencies on the first run; that part is silent.
  Subsequent rebuilds are incremental and complete in seconds.

- **`pxh>` keeps emitting `INFO [mavlink] partner IP: 127.0.0.1`.**

  This is the normal MAVLink heartbeat exchange with QGroundControl (or any other connected GCS).
  It is not an error.

- **The first launch of `px4.exe` shows a _Windows Defender Firewall_ prompt.**

  See [Connecting QGroundControl](#connecting-qgroundcontrol) above.

- **The `pxh>` prompt won't respond.**

  From a different PowerShell window, kill all running PX4 instances with `Get-Process px4 -ErrorAction SilentlyContinue | Stop-Process -Force`.
  The same one-liner cleans up the background instances launched by `Tools\simulation\sitl_multiple_run.ps1`.
  Passing `-SitlNum 0` to the helper script also works: `.\Tools\simulation\sitl_multiple_run.ps1 -SitlNum 0`.

- **`PX4 server already running for instance 0` on a fresh launch.**

  If the previous `px4.exe` was killed forcibly (`Stop-Process -Force`, Task Manager, `CTRL+BREAK`) it may not have unlinked its lock file under `%TEMP%`, so the next launch sees a stale `px4_lock-<N>` and exits.
  Clear the leftovers and re-launch: `Remove-Item "$env:TEMP\px4_lock-*" -ErrorAction SilentlyContinue`.
  A graceful shutdown (`shutdown` from `pxh>`, `px4-shutdown.exe`, or `CTRL+C`) removes the lock automatically.

### `make` Not Found

PowerShell and CMD do not ship `make`, so the Makefile entry point fails until either `ezwinports.make` (winget) or `make` from Chocolatey is installed.
The setup script installs it for you; to install it manually, run:

```sh
winget install --id ezwinports.make -e
```

### `cl.exe` Not Found (MSVC)

CMake searches `PATH` for `cl.exe`; the MSVC compiler is only on `PATH` inside a Visual Studio developer shell.
Either launch the build from _x64 Native Tools Command Prompt for VS 2022_ (Start menu → _Visual Studio 2022_ → _Visual Studio Tools_ → _VC_), or source the developer environment into an existing _PowerShell_ window with `vcvars64.bat`.
Use the path that matches your install — the bundled setup script installs the _Build Tools_ edition:

```sh
# Build Tools (installed by Tools\setup\windows.ps1)
& "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"

# Visual Studio 2022 Community
& "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
```

The leading `&` is the PowerShell call operator; from CMD, drop it and run the path directly (e.g. `"C:\Program Files\...\vcvars64.bat"`).

### `mingw-w64` Not Found (MinGW)

The MinGW toolchain file (`platforms/posix/cmake/Toolchain-mingw-w64-x86_64.cmake`) searches `C:\msys64\mingw64\bin` and `$env:MINGW_PREFIX\bin` for the GCC binaries.
If you installed MSYS2 to a non-default location, set `MINGW_PREFIX` before running `make`:

```sh
$env:MINGW_PREFIX = "D:\msys64\mingw64"
```

### Antivirus Slowing Builds

Real-time scanning of every object file produced by the build can slow compilation by an order of magnitude.
Adding the PX4-Autopilot directory to _Windows Defender_'s exclusion list under **Windows Security > Virus & threat protection > Manage settings > Exclusions** typically restores normal build times.
