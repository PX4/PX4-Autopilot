# ros2_pixi_setup.ps1 — provision a Pixi-based ROS 2 environment on Windows
# native and produce a reusable activation wrapper.
#
# Why Pixi: starting with Jazzy, the OSRF Windows ROS 2 binary release ships
# with a `pixi.toml` and `preinstall_setup_windows.py`. The runtime expects a
# conda-forge-provisioned environment (created by `pixi install`), not the
# legacy VS2019 + chocolatey toolchain used for Humble/Iron.
#
# What this script does (idempotent, no admin required):
#   1. Resolves the ROS 2 binary release zip URL for the requested -Distro
#      (jazzy = current LTS default, rolling, or latest = newest non-prerelease)
#      via the GitHub Releases API, with a hardcoded fallback when rate-limited.
#   2. Downloads (and caches under $env:TEMP) the zip and extracts it to
#      C:\opt\ros\<Distro>\ if not already populated. Pass -Force to re-download.
#   3. Installs pixi to %USERPROFILE%\.pixi\bin if missing.
#   4. Runs `pixi install` against the extracted distro.
#   5. Runs preinstall_setup_windows.py to patch hardcoded shebangs in the
#      release .py files / colcon setup files to point at the pixi env's
#      python.exe.
#   6. Writes a per-distro activation wrapper at
#      $env:TEMP\activate_ros2_<distro>.ps1 (and, for the default distro,
#      a generic $env:TEMP\activate_ros2.ps1 alias) that future sessions
#      dot-source to get a working `ros2`, `colcon`, `python`, etc. on PATH.
#      Pass -WithVcvars when colcon/CMake builds need MSVC (vcvars64.bat).
#
# Usage:
#   .\Tools\setup\ros2_pixi_setup.ps1                  # default: Jazzy LTS
#   .\Tools\setup\ros2_pixi_setup.ps1 -Distro rolling  # Rolling
#   .\Tools\setup\ros2_pixi_setup.ps1 -Distro latest   # newest non-prerelease
#   .\Tools\setup\ros2_pixi_setup.ps1 -RosRoot D:\ros2_jazzy
#   .\Tools\setup\ros2_pixi_setup.ps1 -Force           # re-download release zip
#
# After setup, a typical e2e session looks like:
#   . $env:TEMP\activate_ros2.ps1                      # ros2 only (default distro)
#   . $env:TEMP\activate_ros2_rolling.ps1              # ros2 from Rolling
#   . $env:TEMP\activate_ros2.ps1 -WithVcvars          # +MSVC for colcon
#   ros2 topic list

[CmdletBinding()]
param(
    [ValidateSet('jazzy', 'rolling', 'latest')]
    [string]$Distro = 'jazzy',
    [string]$RosRoot,
    [string]$OutFile,
    [string]$VcvarsBat = 'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat',
    [switch]$Force
)

$ErrorActionPreference = 'Stop'

function L($msg) {
    Write-Host ("[ros2_pixi_setup] " + $msg)
}

# Hardcoded fallback release tags per distro, used when the GitHub Releases API
# is unreachable / rate-limited (unauthenticated = 60 req/hr). Bump as new
# releases land. Tag format: release-<distro>-YYYYMMDD.
$FallbackTags = @{
    'jazzy'   = 'release-jazzy-20260128'
    'rolling' = 'release-rolling-20260128'
}

function Resolve-Ros2ReleaseUrl {
    param([string]$Distro)

    $api = 'https://api.github.com/repos/ros2/ros2/releases?per_page=30'
    $tag = $null
    try {
        $headers = @{ 'User-Agent' = 'px4-ros2-pixi-setup' }
        $releases = Invoke-RestMethod -Uri $api -Headers $headers -TimeoutSec 15
        if ($Distro -eq 'latest') {
            # First non-prerelease, regardless of distro family.
            $rel = $releases | Where-Object { -not $_.prerelease } | Select-Object -First 1
        } else {
            $rel = $releases | Where-Object { -not $_.prerelease -and $_.tag_name -match $Distro } | Select-Object -First 1
        }
        if ($null -ne $rel) { $tag = $rel.tag_name }
    } catch {
        L "GitHub Releases API lookup failed ($($_.Exception.Message)); falling back to hardcoded tag."
    }

    if ([string]::IsNullOrWhiteSpace($tag)) {
        if ($Distro -eq 'latest') {
            $tag = $FallbackTags['jazzy']  # safest default if we can't query
            L "Falling back to hardcoded tag '$tag' for -Distro latest (Jazzy)."
        } elseif ($FallbackTags.ContainsKey($Distro)) {
            $tag = $FallbackTags[$Distro]
            L "Falling back to hardcoded tag '$tag' for -Distro $Distro."
        } else {
            throw "Unable to resolve a release tag for distro '$Distro'."
        }
    }

    # Tag form: release-<distro>-YYYYMMDD -> zip name ros2-<distro>-YYYYMMDD-windows-release-amd64.zip
    if ($tag -notmatch '^release-(?<d>[a-z]+)-(?<date>\d{8})$') {
        throw "Unexpected ros2 release tag format: '$tag'"
    }
    $resolvedDistro = $matches['d']
    $date = $matches['date']
    $zipName = "ros2-$resolvedDistro-$date-windows-release-amd64.zip"
    $url = "https://github.com/ros2/ros2/releases/download/$tag/$zipName"
    return [pscustomobject]@{
        Distro = $resolvedDistro
        Tag    = $tag
        Date   = $date
        ZipName = $zipName
        Url    = $url
    }
}

# 0. Resolve distro / release / install path
$release = Resolve-Ros2ReleaseUrl -Distro $Distro
$effectiveDistro = $release.Distro
L ("Resolved: distro=" + $effectiveDistro + " tag=" + $release.Tag + " url=" + $release.Url)

if ([string]::IsNullOrWhiteSpace($RosRoot)) {
    $RosRoot = "C:\opt\ros\$effectiveDistro"
}
if ([string]::IsNullOrWhiteSpace($OutFile)) {
    $OutFile = "$env:TEMP\activate_ros2_${effectiveDistro}.ps1"
}
L "Install root: $RosRoot"
L "Activation wrapper: $OutFile"

# 1. Download + extract release zip if needed
$cachedZip = Join-Path $env:TEMP $release.ZipName
if (-not (Test-Path "$RosRoot\pixi.toml")) {
    if ($Force -and (Test-Path $cachedZip)) {
        L "Removing cached zip due to -Force: $cachedZip"
        Remove-Item $cachedZip -Force
    }
    if (-not (Test-Path $cachedZip)) {
        L "Downloading $($release.Url) -> $cachedZip ..."
        Invoke-WebRequest -Uri $release.Url -OutFile $cachedZip -UseBasicParsing
    } else {
        L "Reusing cached release zip: $cachedZip"
    }
    if (-not (Test-Path $RosRoot)) {
        New-Item -ItemType Directory -Path $RosRoot -Force | Out-Null
    }
    L "Extracting to $RosRoot ..."
    Expand-Archive -Path $cachedZip -DestinationPath $RosRoot -Force
}

# 2. Pixi
$pixiBin = "$env:USERPROFILE\.pixi\bin"
if (-not (Get-Command pixi -ErrorAction SilentlyContinue) -and -not (Test-Path "$pixiBin\pixi.exe")) {
    L "Installing pixi..."
    try { Invoke-Expression (Invoke-WebRequest -UseBasicParsing https://pixi.sh/install.ps1).Content } catch { L "pixi install warning: $_" }
}
$env:Path = "$pixiBin;$env:Path"
if (-not (Get-Command pixi -ErrorAction SilentlyContinue)) {
    throw "pixi not found on PATH after install attempt (looked in $pixiBin)"
}
L ("pixi version: " + ((& pixi --version) -join ' '))

# 3. Validate distro and run pixi install
if (-not (Test-Path "$RosRoot\pixi.toml")) {
    throw "pixi.toml not found at $RosRoot. Extract the ROS 2 binary release zip there first."
}
L "Running 'pixi install' in $RosRoot ..."
Push-Location $RosRoot
try {
    & pixi install
    if ($LASTEXITCODE -ne 0) { throw "pixi install failed (rc=$LASTEXITCODE)" }
} finally { Pop-Location }

# 4. Patch shebangs (idempotent)
$preinstall = Join-Path $RosRoot 'preinstall_setup_windows.py'
if (Test-Path $preinstall) {
    L "Running preinstall_setup_windows.py to patch shebangs..."
    Push-Location $RosRoot
    try {
        & pixi run --manifest-path "$RosRoot\pixi.toml" python preinstall_setup_windows.py | Select-Object -Last 5
    } finally { Pop-Location }
} else {
    L "preinstall_setup_windows.py not found in $RosRoot, skipping shebang patch."
}

# 5. Write reusable activation wrapper
L "Writing activation wrapper to $OutFile ..."
$wrapper = @"
# Auto-generated by Tools/setup/ros2_pixi_setup.ps1
# Distro: $effectiveDistro (release tag: $($release.Tag))
# Dot-source to activate ROS 2 (Pixi/conda-forge) in the current PowerShell session.
# Usage:
#   . `$env:TEMP\activate_ros2_${effectiveDistro}.ps1                # ros2 / colcon / python
#   . `$env:TEMP\activate_ros2_${effectiveDistro}.ps1 -WithVcvars    # also load MSVC vcvars64
param([switch]`$WithVcvars)

`$env:Path = "`$env:USERPROFILE\.pixi\bin;`$env:Path"
`$hook = & pixi shell-hook --manifest-path '$RosRoot\pixi.toml' --shell powershell 2>`$null
if (`$LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace(`$hook)) { throw 'pixi shell-hook failed' }
Invoke-Expression (`$hook -join "`n")
. '$RosRoot\local_setup.ps1'

if (`$WithVcvars) {
    `$vc = '$VcvarsBat'
    if (-not (Test-Path `$vc)) { throw "vcvars64.bat not found at `$vc" }
    `$tmp = [System.IO.Path]::GetTempFileName()
    `$batLine = '"' + `$vc + '" && set'
    & cmd.exe /c `$batLine > `$tmp 2>`$null
    Get-Content `$tmp | ForEach-Object {
        if (`$_ -match '^([^=]+)=(.*)`$') {
            Set-Item -Path "Env:`$(`$matches[1])" -Value `$matches[2]
        }
    }
    Remove-Item `$tmp -Force
}
"@

$wrapper | Out-File -FilePath $OutFile -Encoding utf8 -Force

# Also publish a generic 'activate_ros2.ps1' alias pointing at the same content,
# so the legacy command (used in docs / muscle memory) keeps working for the
# most recently installed distro.
$genericOut = "$env:TEMP\activate_ros2.ps1"
if ($OutFile -ne $genericOut) {
    $wrapper | Out-File -FilePath $genericOut -Encoding utf8 -Force
    L "Also wrote alias wrapper to $genericOut (points at $effectiveDistro)."
}

L "Done. Sanity check:"
& powershell -NoProfile -ExecutionPolicy Bypass -Command ". '$OutFile'; ros2 --help 2>&1 | Select-Object -First 3"
L "OK. Source '$OutFile' in future sessions."
