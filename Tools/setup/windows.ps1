<#
.SYNOPSIS
    Set up a native Windows development environment for PX4 SITL.

.DESCRIPTION
    Installs the toolchain required to build px4_sitl_default natively on
    Windows 10 or 11, using either MSVC (the CI-tested default) or
    MinGW-w64 via MSYS2. Mirrors what Tools/setup/ubuntu.sh does on Linux.

    The script is idempotent: re-running it skips packages that are already
    installed. Package installs are performed via winget; if winget is not
    available the script falls back to Chocolatey for the few packages that
    cannot be installed any other way (currently only GNU make).

    Run from an elevated PowerShell prompt:

        Set-ExecutionPolicy -Scope Process Bypass
        .\Tools\setup\windows.ps1

    To pick a build path:

        .\Tools\setup\windows.ps1 -Toolchain MSVC    # default, CI-tested
        .\Tools\setup\windows.ps1 -Toolchain MinGW   # MSYS2 mingw-w64
        .\Tools\setup\windows.ps1 -Toolchain Both

.PARAMETER Toolchain
    Which build path to install. One of: MSVC, MinGW, Both. Default: MSVC.

.PARAMETER NoBuildTools
    Skip the Visual Studio Build Tools install (assume the user already has
    Visual Studio 2022 with the "Desktop development with C++" workload).

.PARAMETER NoPip
    Skip the pip install step (assume Python deps are already installed).

.NOTES
    See docs/en/dev_setup/dev_env_windows_native.md for the full guide.
#>
[CmdletBinding()]
param(
    [ValidateSet('MSVC', 'MinGW', 'Both')]
    [string]$Toolchain = 'MSVC',
    [switch]$NoBuildTools,
    [switch]$NoPip
)

$ErrorActionPreference = 'Stop'
$ProgressPreference    = 'SilentlyContinue'

function Write-Section($Message) {
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Test-Command($Name) {
    # Restrict to real applications/scripts on PATH and ignore PowerShell
    # functions and aliases. Otherwise a same-named function defined in the
    # caller's session (e.g. a test stub) would silently satisfy this check.
    [bool](Get-Command -Name $Name -CommandType Application,ExternalScript -ErrorAction SilentlyContinue)
}

# Refresh $env:Path from the machine + user registry hives. winget installs
# update the registry PATH but do NOT propagate it to the current process,
# so any post-install command (`python -m pip ...`, `make ...`) would fail
# in the same shell unless we re-pull PATH ourselves.
function Update-PathFromRegistry {
    $machinePath = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    $userPath    = [Environment]::GetEnvironmentVariable('Path', 'User')
    $combined = @($machinePath, $userPath) | Where-Object { $_ } | ForEach-Object { $_.TrimEnd(';') }
    if ($combined.Count -gt 0) { $env:Path = ($combined -join ';') }
}

# Locate a tool that winget just installed. Refreshes PATH from the registry
# first, then falls back to a list of well-known install locations. Returns
# the full path to the executable, or $null if it cannot be found.
function Resolve-Tool($Name, [string[]]$ExtraSearchPaths = @()) {
    if (Test-Command $Name) { return (Get-Command $Name -CommandType Application,ExternalScript).Source }
    Update-PathFromRegistry
    if (Test-Command $Name) { return (Get-Command $Name -CommandType Application,ExternalScript).Source }
    foreach ($candidate in $ExtraSearchPaths) {
        if (Test-Path $candidate) {
            $dir = Split-Path -Parent $candidate
            $procPathEntries = $env:Path.Split(';', [StringSplitOptions]::RemoveEmptyEntries)
            if (-not ($procPathEntries | Where-Object { $_.TrimEnd('\') -ieq $dir.TrimEnd('\') })) {
                $env:Path = "$env:Path;$dir"
            }
            return $candidate
        }
    }
    return $null
}

function Install-Winget($Id, $Override = $null) {
    Write-Host "  winget install $Id"
    # NOTE: do NOT name this $args -- that's an automatic variable inside every
    # function, and `& winget @args` would splat the function's auto-args (here
    # empty) rather than the intended array.
    $wingetArgs = @('install', '--id', $Id, '-e',
                    '--source', 'winget',
                    '--accept-package-agreements',
                    '--accept-source-agreements',
                    '--disable-interactivity')
    if ($Override) { $wingetArgs += @('--override', $Override) }
    & winget @wingetArgs
    # winget exit codes that mean "nothing to do" -- treat as success:
    #   -1978335189 (0x8A15002B) APPINSTALLER_CLI_ERROR_NO_APPLICABLE_UPGRADE
    #   -1978335212 (0x8A150014) APPINSTALLER_CLI_ERROR_UPDATE_NOT_APPLICABLE
    #   -1978335215 (0x8A150011) APPINSTALLER_CLI_ERROR_PACKAGE_ALREADY_INSTALLED
    $okCodes = @(0, -1978335189, -1978335212, -1978335215)
    if ($okCodes -notcontains $LASTEXITCODE) {
        throw "winget install $Id failed with exit code $LASTEXITCODE"
    }
}

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------

# Visual Studio Build Tools writes to C:\Program Files and winget's
# machine-scope installs require elevation, so fail fast with an actionable
# error rather than partway through a 5+ GB install.
$principal = [Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw @"
This script must be run from an ELEVATED PowerShell prompt.

  1. Press Start, type 'PowerShell'
  2. Right-click 'Windows PowerShell' and choose 'Run as administrator'
  3. cd to the PX4-Autopilot directory and re-run:
       Set-ExecutionPolicy -Scope Process Bypass
       .\Tools\setup\windows.ps1
"@
}

if (-not (Test-Command 'winget')) {
    throw "winget is not on PATH. Install 'App Installer' from the Microsoft Store, then re-run this script."
}

# ---------------------------------------------------------------------------
# Common dependencies (both toolchains)
# ---------------------------------------------------------------------------

Write-Section "Installing common build dependencies"
Install-Winget 'Git.Git'
Install-Winget 'Python.Python.3.11'
Install-Winget 'Kitware.CMake'
Install-Winget 'Ninja-build.Ninja'

# Pull any PATH entries the four installs above just registered into the
# current process so the make / python / cmake checks below see them without
# requiring the user to open a new shell first.
Update-PathFromRegistry

# Git for Windows installs `git.exe` to `C:\Program Files\Git\cmd` (which is
# what its installer puts on the system PATH), but the PX4 top-level Makefile
# also calls into `sed`, `awk`, `dirname`, `[`, and `ps` -- none of which ship
# with Windows and all of which live in `<git>\usr\bin`. Without that directory
# on PATH, `make px4_sitl_default` aborts in the parameter / version pre-build
# steps even with a working MSVC compiler. Probe the registry for the actual
# Git install location (in case the user installed to a non-default path) and
# fall back to the default `C:\Program Files\Git`.
$gitRoot = $null
$gitUninstallKeys = @(
    'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
    'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*',
    'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*'
)
foreach ($key in $gitUninstallKeys) {
    $entry = Get-ItemProperty $key -ErrorAction SilentlyContinue |
             Where-Object { $_.DisplayName -like 'Git*' -and $_.InstallLocation -and (Test-Path (Join-Path $_.InstallLocation 'usr\bin')) } |
             Select-Object -First 1
    if ($entry) { $gitRoot = $entry.InstallLocation.TrimEnd('\'); break }
}
if (-not $gitRoot) { $gitRoot = "${env:ProgramFiles}\Git" }
$gitUsrBin = Join-Path $gitRoot 'usr\bin'
if (Test-Path $gitUsrBin) {
    # Mirror the dedup logic used for `mingw64\bin` further down: split the
    # user PATH on ';' and compare entries case-insensitively to avoid false
    # negatives from trailing slashes and to avoid appending duplicates on
    # re-run.
    $userPathGit = [Environment]::GetEnvironmentVariable('Path', 'User')
    $pathEntriesGit = @()
    if ($userPathGit) { $pathEntriesGit = $userPathGit.Split(';', [StringSplitOptions]::RemoveEmptyEntries) }
    $alreadyOnPathGit = $pathEntriesGit | Where-Object { $_.TrimEnd('\') -ieq $gitUsrBin.TrimEnd('\') }
    if (-not $alreadyOnPathGit) {
        Write-Host "  Adding $gitUsrBin to user PATH (provides sed/awk/dirname/ps for the PX4 Makefile)"
        $newPathGit = if ($userPathGit) { "$userPathGit;$gitUsrBin" } else { $gitUsrBin }
        [Environment]::SetEnvironmentVariable('Path', $newPathGit, 'User')
    }
    # Also expose `usr\bin` in the current process so a `make px4_sitl_default`
    # in this same shell finds the GNU tools without requiring a new shell.
    $procPathEntriesGit = $env:Path.Split(';', [StringSplitOptions]::RemoveEmptyEntries)
    if (-not ($procPathEntriesGit | Where-Object { $_.TrimEnd('\') -ieq $gitUsrBin.TrimEnd('\') })) {
        $env:Path = "$env:Path;$gitUsrBin"
    }
} else {
    Write-Host "  WARNING: Git Bash 'usr\bin' not found at $gitUsrBin."
    Write-Host "           PX4's Makefile needs sed/awk/dirname/ps from Git Bash."
    Write-Host "           If Git is installed elsewhere, add <git-root>\usr\bin to PATH manually."
}

# GNU make is not on the default winget source. The PX4 top-level Makefile
# is required by `make px4_sitl_default`, so install it from chocolatey if
# present; otherwise fall back to ezwinports' winget package which ships a
# native Win32 make.exe.
if (-not (Test-Command 'make')) {
    Write-Section "Installing GNU make"
    if (Test-Command 'choco') {
        & choco install -y make --no-progress
        # choco exit code 0 = installed, 1641/3010 = reboot required (benign).
        # Anything else is a real failure.
        $okChocoCodes = @(0, 1641, 3010)
        if ($okChocoCodes -notcontains $LASTEXITCODE) {
            throw "choco install make failed with exit code $LASTEXITCODE"
        }
    } else {
        Install-Winget 'ezwinports.make'
    }
    # Both choco's make and ezwinports.make register their bin directory on
    # the registry PATH but not on the running process's PATH; refresh so a
    # subsequent `make px4_sitl_default` in this session can find make.exe.
    $makePath = Resolve-Tool 'make' @(
        'C:\ProgramData\chocolatey\bin\make.exe',
        "${env:ProgramFiles}\ezwinports\make-*\bin\make.exe",
        "${env:LOCALAPPDATA}\Microsoft\WinGet\Packages\ezwinports.make_Microsoft.Winget.Source_*\bin\make.exe"
    )
    if (-not $makePath) {
        Write-Host "  WARNING: make was installed but is not on PATH for the current shell."
        Write-Host "           Open a NEW shell before running 'make px4_sitl_default'."
    }
}

# ---------------------------------------------------------------------------
# MSVC toolchain
# ---------------------------------------------------------------------------

if ($Toolchain -in @('MSVC', 'Both')) {
    Write-Section "Installing MSVC toolchain"
    if ($NoBuildTools) {
        Write-Host "  -NoBuildTools set; assuming Visual Studio 2022 is already installed."
    } else {
        # The VCTools workload ("C++ build tools") pulls in the MSVC core
        # IDE bits but lists the latest MSVC v143 toolset and the Windows
        # 11 SDK only as *Recommended* (not Required) components, so the
        # latest SDK is NOT installed unless we add it explicitly. Pin the
        # newest Windows 11 SDK (26100) and add the MSVC x64/x86 toolset
        # plus the C++ CMake tools so builds are reproducible.
        # Reference: https://learn.microsoft.com/en-us/visualstudio/install/workload-component-id-vs-build-tools?view=vs-2022
        $vsOverride = '--quiet --wait --norestart --nocache ' +
            '--add Microsoft.VisualStudio.Workload.VCTools ' +
            '--add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 ' +
            '--add Microsoft.VisualStudio.Component.VC.CMake.Project ' +
            '--add Microsoft.VisualStudio.Component.Windows11SDK.26100'
        Install-Winget 'Microsoft.VisualStudio.2022.BuildTools' $vsOverride
    }
}

# ---------------------------------------------------------------------------
# MinGW toolchain (MSYS2)
# ---------------------------------------------------------------------------

if ($Toolchain -in @('MinGW', 'Both')) {
    Write-Section "Installing MSYS2 (MinGW-w64 toolchain)"
    Install-Winget 'MSYS2.MSYS2'

    # MSYS2 installs to C:\msys64 by default but can be relocated via a custom
    # --location override on the installer. Probe the Uninstall registry key
    # for the actual InstallLocation before falling back to the default path.
    $msys2Root = $null
    $uninstallKeys = @(
        'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*',
        'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*'
    )
    foreach ($key in $uninstallKeys) {
        $entry = Get-ItemProperty $key -ErrorAction SilentlyContinue |
                 Where-Object { $_.DisplayName -like 'MSYS2*' -and $_.InstallLocation } |
                 Select-Object -First 1
        if ($entry) { $msys2Root = $entry.InstallLocation.TrimEnd('\'); break }
    }
    if (-not $msys2Root) { $msys2Root = 'C:\msys64' }
    $msys2Bash = Join-Path $msys2Root 'usr\bin\bash.exe'
    if (-not (Test-Path $msys2Bash)) {
        throw @"
MSYS2 was installed but bash.exe was not found at:
  $msys2Bash

To recover:
  1. Verify the install: winget list MSYS2.MSYS2
  2. If MSYS2 was installed to a non-default location, set the install root
     and re-run this script, OR install the MinGW toolchain manually from an
     MSYS2 shell:
       pacman -S --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-ccache
  3. If MSYS2 is missing entirely, reinstall:
       winget install MSYS2.MSYS2 -e --source winget
"@
    }
    $mingwBin = Join-Path $msys2Root 'mingw64\bin'

    # `pacman -Syu` can update the MSYS2 runtime itself, in which case it
    # forcibly closes its shell and requires a second invocation to finish.
    # Run it twice so a runtime upgrade on the first pass converges on the
    # second; the second pass is a near-instant no-op when nothing changed.
    Write-Host "  Updating MSYS2 package database (pass 1/2)"
    & $msys2Bash -lc 'pacman -Syu --noconfirm --noprogressbar'
    if ($LASTEXITCODE -ne 0) {
        throw "MSYS2 'pacman -Syu' failed with exit code $LASTEXITCODE"
    }
    Write-Host "  Updating MSYS2 package database (pass 2/2)"
    & $msys2Bash -lc 'pacman -Syu --noconfirm --noprogressbar'
    if ($LASTEXITCODE -ne 0) {
        throw "MSYS2 'pacman -Syu' (second pass) failed with exit code $LASTEXITCODE"
    }
    Write-Host "  Installing mingw-w64-x86_64 toolchain"
    & $msys2Bash -lc 'pacman -S --noconfirm --noprogressbar --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-ccache'
    if ($LASTEXITCODE -ne 0) {
        throw "MSYS2 'pacman -S mingw-w64-x86_64-toolchain' failed with exit code $LASTEXITCODE"
    }

    # Toolchain-mingw-w64-x86_64.cmake searches <msys2>/mingw64/bin first,
    # but the wrapper .cmd shims it generates need the directory on PATH at
    # build time too. ($mingwBin was set above from the MSYS2 install root.)
    $userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
    # Split on ';' and compare entries case-insensitively to avoid false
    # negatives from a substring search (e.g. trailing slashes) and to avoid
    # appending a duplicate on re-run.
    $pathEntries = @()
    if ($userPath) { $pathEntries = $userPath.Split(';', [StringSplitOptions]::RemoveEmptyEntries) }
    $alreadyOnPath = $pathEntries | Where-Object { $_.TrimEnd('\') -ieq $mingwBin.TrimEnd('\') }
    if (-not $alreadyOnPath) {
        Write-Host "  Adding $mingwBin to user PATH"
        $newPath = if ($userPath) { "$userPath;$mingwBin" } else { $mingwBin }
        [Environment]::SetEnvironmentVariable('Path', $newPath, 'User')
    }
    # Also expose mingw64\bin in the current process so any subsequent steps
    # (e.g. a follow-up `make` in the same shell) can find gcc/g++ without
    # the user having to spawn a new shell first.
    $procPathEntries = $env:Path.Split(';', [StringSplitOptions]::RemoveEmptyEntries)
    if (-not ($procPathEntries | Where-Object { $_.TrimEnd('\') -ieq $mingwBin.TrimEnd('\') })) {
        $env:Path = "$env:Path;$mingwBin"
    }
}

# ---------------------------------------------------------------------------
# Python build-time dependencies
# ---------------------------------------------------------------------------

if (-not $NoPip) {
    Write-Section "Installing Python build dependencies"
    # winget installs Python.Python.3.11 to the user's AppData and updates the
    # registry PATH, but the running PowerShell session inherited PATH at
    # startup and will NOT see the new entry. Refresh PATH from the registry
    # and, if python is still missing, fall back to the `py` launcher and to
    # the well-known Python 3.11 install locations. This is the difference
    # between a one-shot setup and a "open a new shell and re-run" loop.
    $pythonExe = Resolve-Tool 'python' @(
        "${env:LOCALAPPDATA}\Programs\Python\Python311\python.exe",
        "${env:ProgramFiles}\Python311\python.exe",
        "${env:ProgramFiles(x86)}\Python311\python.exe"
    )
    if (-not $pythonExe) {
        # Last-resort: the Python launcher (`py.exe`) ships in C:\Windows and
        # is on the per-machine PATH that survives a fresh PowerShell session.
        $pyLauncher = Resolve-Tool 'py' @('C:\Windows\py.exe')
        if ($pyLauncher) {
            Write-Host "  python.exe not on PATH for this session; using 'py -3.11' instead."
            $pythonExe = $pyLauncher
            $pythonArgs = @('-3.11')
        } else {
            throw @"
Python was installed but neither 'python' nor 'py' is on PATH for this shell.
This is usually a transient PATH-propagation issue after a winget install.

To recover, open a NEW PowerShell window and re-run with -NoBuildTools so the
script skips straight to the pip step:
  .\Tools\setup\windows.ps1 -NoBuildTools
"@
        }
    } else {
        $pythonArgs = @()
    }
    # Mirrors the package set installed by .github/workflows/compile_windows.yml.
    # kconfiglib is required by cmake/kconfig.cmake; on the Ubuntu MinGW CI
    # runner it is provided by the `python3-kconfiglib` apt package, on the
    # MSVC runner and here it must come from pip.
    & $pythonExe @pythonArgs -m pip install --upgrade pip
    if ($LASTEXITCODE -ne 0) { throw "pip self-upgrade failed with exit code $LASTEXITCODE" }
    # empy is pinned <4 to match Tools/setup/requirements.txt: PX4's msg /
    # uxrce_dds / flight_tasks / zenoh code generators use the empy 3.x
    # `em.Interpreter(..., options={em.RAW_OPT: True, em.BUFFERED_OPT: True})`
    # API which was removed in empy 4.x. The Ubuntu CI runners get the right
    # version via the `python3-empy` apt package (3.3.x); on Windows pip would
    # otherwise pull empy 4.x and break code generation.
    & $pythonExe @pythonArgs -m pip install jinja2 pyyaml toml numpy packaging jsonschema future "empy>=3.3,<4" pyros-genmsg kconfiglib
    if ($LASTEXITCODE -ne 0) { throw "pip install failed with exit code $LASTEXITCODE" }
}

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------

Write-Section "Setup complete"
Write-Host ""
Write-Host "Open a NEW shell so PATH changes take effect, then build PX4 SITL:"
Write-Host ""
if ($Toolchain -in @('MSVC', 'Both')) {
    Write-Host "  MSVC build (from 'x64 Native Tools Command Prompt for VS 2022'):"
    Write-Host "      cd PX4-Autopilot"
    Write-Host "      make px4_sitl_default"
    Write-Host ""
}
if ($Toolchain -in @('MinGW', 'Both')) {
    Write-Host "  MinGW build (from PowerShell):"
    Write-Host "      cd PX4-Autopilot"
    Write-Host "      `$env:CMAKE_ARGS = '-DCMAKE_TOOLCHAIN_FILE=Toolchain-mingw-w64-x86_64'"
    Write-Host "      make px4_sitl_default"
    Write-Host ""
}
Write-Host "Run a SIH simulation from the build output:"
Write-Host "      `$env:PX4_SIM_MODEL = 'sihsim_quadx'"
Write-Host "      build\px4_sitl_default\bin\px4.exe -d build\px4_sitl_default\etc"
Write-Host ""
Write-Host "See docs/en/dev_setup/dev_env_windows_native.md for the full guide."
