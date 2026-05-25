<#
.SYNOPSIS
    Run multiple instances of the 'px4' binary on Windows, without starting
    an external simulator. Mirrors Tools/simulation/sitl_multiple_run.sh -
    same args, same per-instance working directories, same logfile names.

.DESCRIPTION
    For developers on PowerShell or CMD (the .sh script also works under
    Git Bash on Windows). Assumes px4 is already built with the specified
    build target (e.g., px4_sitl_default).

    Pass 0 for SitlNum to just stop everything currently running:
        .\Tools\simulation\sitl_multiple_run.ps1 0

.PARAMETER SitlNum
    Number of instances to start. Defaults to 2.

.PARAMETER SimModel
    Value to export as PX4_SIM_MODEL. Defaults to 'gazebo-classic_iris' to
    match the .sh script; set to 'sihsim_quadx' for a Windows-friendly run
    that needs no external simulator.

.PARAMETER BuildTarget
    Name of the cmake build directory under build/. Defaults to
    'px4_sitl_default'.

.EXAMPLE
    .\Tools\simulation\sitl_multiple_run.ps1 3 sihsim_quadx px4_sitl_default

.EXAMPLE
    # From CMD:
    powershell -ExecutionPolicy Bypass -File Tools\simulation\sitl_multiple_run.ps1 2 sihsim_quadx
#>
param(
    [int]$SitlNum = 2,
    [string]$SimModel = 'gazebo-classic_iris',
    [string]$BuildTarget = 'px4_sitl_default'
)

$ErrorActionPreference = 'Stop'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$SrcPath = (Resolve-Path (Join-Path $ScriptDir '..\..')).Path
$BuildPath = Join-Path $SrcPath "build\$BuildTarget"

# Prefer px4.exe; fall back to px4 so this also runs against a WSL/MinGW
# layout that drops the suffix.
$Px4Bin = Join-Path $BuildPath 'bin\px4.exe'
if (-not (Test-Path $Px4Bin)) { $Px4Bin = Join-Path $BuildPath 'bin\px4' }
if ($SitlNum -gt 0 -and -not (Test-Path $Px4Bin)) {
    Write-Error "px4 binary not found in $BuildPath\bin"
}

Write-Host 'killing running instances'
Get-Process -Name 'px4' -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 1

# Clean up stale lock files left by force-killed instances. Each instance
# uses %TEMP%\px4_lock-<N>; if the previous owner was killed before it could
# unlink, the next start sees "already running" and aborts.
Get-ChildItem -Path "$env:TEMP" -Filter 'px4_lock-*' -ErrorAction SilentlyContinue | Remove-Item -Force -ErrorAction SilentlyContinue

# Clear stale per-instance log files from previous runs. These are not
# truncated by the new launch (Start-Process -RedirectStandardOutput appends),
# so leftover content from a different rcS / different model can mislead
# debugging by appearing alongside fresh output.
if ($SitlNum -gt 0) {
    for ($n = 0; $n -lt $SitlNum; $n++) {
        $stale = Join-Path $BuildPath "instance_$n"
        if (Test-Path $stale) {
            Remove-Item -Path (Join-Path $stale 'out.log') -Force -ErrorAction SilentlyContinue
            Remove-Item -Path (Join-Path $stale 'err.log') -Force -ErrorAction SilentlyContinue
        }
    }
}

$env:PX4_SIM_MODEL = $SimModel

for ($n = 0; $n -lt $SitlNum; $n++) {
    $WorkingDir = Join-Path $BuildPath "instance_$n"
    if (-not (Test-Path $WorkingDir)) { New-Item -ItemType Directory -Path $WorkingDir | Out-Null }
    Write-Host "starting instance $n in $WorkingDir"
    # Spawn px4.exe directly with redirected stdout/stderr.
    #
    # Earlier revisions wrapped each launch in `cmd.exe /c "px4.exe ... > out.log"`,
    # which had a fatal flaw at higher instance counts: every cmd.exe child
    # inherits the launcher's console, and when that console is torn down
    # (cmd.exe exiting after spawning px4, or the launcher PowerShell window
    # closing) Windows broadcasts CTRL_CLOSE_EVENT to every attached process.
    # PX4's SetConsoleCtrlHandler maps CTRL_CLOSE_EVENT to SIGINT and shuts
    # down cleanly, so multi-instance launches died silently a few seconds
    # after rcS finished -- no error, empty stderr, just gone.
    #
    # Start-Process with -RedirectStandard* writes both streams to per-instance
    # log files without involving cmd.exe, and -WindowStyle Hidden gives each
    # child its own (hidden) console so it survives the launcher exiting.
    $stdoutPath = Join-Path $WorkingDir 'out.log'
    $stderrPath = Join-Path $WorkingDir 'err.log'
    Start-Process -FilePath $Px4Bin `
        -ArgumentList @('-i', $n, '-d', "$BuildPath\etc") `
        -WorkingDirectory $WorkingDir `
        -RedirectStandardOutput $stdoutPath `
        -RedirectStandardError $stderrPath `
        -WindowStyle Hidden | Out-Null
}
