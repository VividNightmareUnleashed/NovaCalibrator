# Nova Calibrator installer (script-based, replaces the NSIS installer).
# Run elevated. Installs the overlay app + OpenVR driver and registers the
# app manifest with SteamVR.
[CmdletBinding()]
param(
    # Captured before elevation: an elevated shell can run under a different
    # profile, and openvrpaths.vrpath lives in the *user's* LocalAppData.
    [string]$UserLocalAppData = $env:LOCALAPPDATA,
    # For scripted installs and the install test: never wait for a key press,
    # never elevate (run it elevated), and decline the OpenVR-SpaceCalibrator
    # removal prompt rather than answer it for the user. The exit code is the
    # result.
    [switch]$Unattended,
    # Optional modules. An unattended install gets only the ones named here;
    # an interactive one asks, offering what the previous install had.
    [switch]$Lighthouse
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'FilesystemPolicy.ps1')

function Pause-ForUser {
    if (-not $Unattended) { Read-Host "Press Enter to close this window" | Out-Null }
}

# Ensure the window stays open on any failure so the user can read the error.
trap {
    Write-Host ""
    Write-Host "UNEXPECTED ERROR: $_" -ForegroundColor Red
    Write-Host ""
    Pause-ForUser
    exit 1
}

function Fail([string]$msg) {
    Write-Host ""
    Write-Host "ERROR: $msg" -ForegroundColor Red
    Write-Host ""
    Pause-ForUser
    exit 1
}

# --- Admin check ------------------------------------------------------------
$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    if ($Unattended) { Fail "An unattended installation needs administrator permission. Start it from an elevated shell." }
    Write-Host "The installer needs administrator permission because it writes to the SteamVR folder." -ForegroundColor Yellow
    Write-Host "The installer starts again with administrator permission."
    # Use -NoExit so the elevated window stays open to show output/errors
    $relaunchArgs = @(
        '-NoExit', '-NoProfile', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"",
        '-UserLocalAppData', "`"$env:LOCALAPPDATA`""
    )
    if ($Lighthouse) { $relaunchArgs += '-Lighthouse' }
    Start-Process -FilePath "powershell.exe" -Verb RunAs -ArgumentList $relaunchArgs
    exit 0
}

# --- SteamVR must be closed --------------------------------------------------
# steam.exe itself also locks driver DLLs, not just the VR processes
$steamProcesses = Get-Process -Name 'steam','vrserver','vrmonitor','vrcompositor' -ErrorAction SilentlyContinue
if ($steamProcesses) {
    $names = ($steamProcesses | Select-Object -ExpandProperty Name) -join ', '
    Fail "Steam or SteamVR is open ($names). Close Steam completely. Make sure that the Steam icon is not in the system tray. Then start the installer again."
}

# --- Optional modules ---------------------------------------------------------
# Asked before anything changes, and read before the upgrade path's uninstall
# clears the registry, so an upgrade can offer the previous choice.
$modulesKey = 'HKLM:\Software\NovaCalibrator\Modules'
$installLighthouse = [bool]$Lighthouse
if (-not $Unattended -and -not $Lighthouse) {
    # A first install over QuestCalibrator offers what that install had.
    $previousModules = Get-ItemProperty $modulesKey -ErrorAction SilentlyContinue
    if (-not $previousModules) { $previousModules = Get-ItemProperty 'HKLM:\Software\QuestCalibrator\Modules' -ErrorAction SilentlyContinue }
    $hadLighthouse = $previousModules.Lighthouse -eq 1
    Write-Host ""
    Write-Host "Optional module: Lighthouse" -ForegroundColor Cyan
    Write-Host "  The Lighthouse module adds a page that shows each base station and the"
    Write-Host "  Lighthouse devices that can see it. Later versions add more base station"
    Write-Host "  tools. Calibration does not need this module."
    $default = if ($hadLighthouse) { 'Y' } else { 'N' }
    $answer = Read-Host "Install the Lighthouse module? [Y/N] (default $default)"
    if (-not $answer) { $answer = $default }
    $installLighthouse = $answer -match '^[Yy]'
}

$packageDir = $PSScriptRoot
$appSrc     = Join-Path $packageDir 'app'
$driverSrc  = Join-Path $packageDir 'driver\01novacalibrator'

foreach ($p in @($appSrc, $driverSrc)) {
    if (-not (Test-Path $p)) { Fail "The package is not complete. The installer cannot find '$p'. Extract the zip file again. Then start the installer again." }
}

# --- Locate the OpenVR/SteamVR runtime ---------------------------------------
# Steam records its install root in HKLM, and libraryfolders.vdf lists every
# additional library - including custom paths that guessing drive layouts misses.
function Get-SteamLibraryPaths {
    $roots = @()
    foreach ($key in @('HKLM:\SOFTWARE\WOW6432Node\Valve\Steam', 'HKLM:\SOFTWARE\Valve\Steam')) {
        $p = (Get-ItemProperty $key -ErrorAction SilentlyContinue).InstallPath
        if ($p) { $roots += $p }
    }
    $p = (Get-ItemProperty 'HKCU:\SOFTWARE\Valve\Steam' -ErrorAction SilentlyContinue).SteamPath
    if ($p) { $roots += ($p -replace '/', '\') }
    $roots += Join-Path ${env:ProgramFiles(x86)} 'Steam'

    $libraries = @()
    foreach ($root in ($roots | Where-Object { $_ } | Select-Object -Unique)) {
        if (-not (Test-Path $root)) { continue }
        $libraries += $root
        $vdf = Join-Path $root 'steamapps\libraryfolders.vdf'
        if (-not (Test-Path $vdf)) { continue }
        foreach ($line in (Get-Content $vdf -ErrorAction SilentlyContinue)) {
            if ($line -match '"path"\s+"(.+?)"') {
                # paths are backslash-escaped inside the vdf
                $libraries += ($Matches[1] -replace '\\\\', '\')
            }
        }
    }
    $libraries | Where-Object { $_ } | Select-Object -Unique
}

function Find-VrPathReg {
    foreach ($lib in (Get-SteamLibraryPaths)) {
        $c = Join-Path $lib 'steamapps\common\SteamVR\bin\win64\vrpathreg.exe'
        if (Test-Path $c) { return $c }
    }
    # Last resort: sweep fixed drives for the common layouts. Fixed drives only,
    # so a disconnected network drive can't stall the installer.
    $drives = Get-CimInstance Win32_LogicalDisk -Filter 'DriveType=3' -ErrorAction SilentlyContinue |
        Select-Object -ExpandProperty DeviceID
    foreach ($drive in $drives) {
        foreach ($sub in @('SteamLibrary', 'Steam', 'Games\SteamLibrary', 'Games\Steam',
                           'Program Files (x86)\Steam', 'Program Files\Steam')) {
            $c = Join-Path "$drive\" "$sub\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
            if (Test-Path $c) { return $c }
        }
    }
    return $null
}

function Get-VrRuntimePath {
    $vrpathreg = Find-VrPathReg
    if ($vrpathreg) {
        Write-Host "SteamVR path tool: $vrpathreg"
        $out = & $vrpathreg show 2>&1 | Out-String
        if ($out -match 'Runtime path\s*=\s*(.+)') {
            $candidate = $Matches[1].Trim()
            if (Test-Path $candidate) { return $candidate }
        }
    }
    # openvrpaths.vrpath is the file vrpathreg itself reads. Going straight to it
    # covers non-Steam runtimes and layouts the search above didn't guess.
    foreach ($base in @($UserLocalAppData, $env:LOCALAPPDATA)) {
        if (-not $base) { continue }
        $vrpath = Join-Path $base 'openvr\openvrpaths.vrpath'
        if (-not (Test-Path $vrpath)) { continue }
        try {
            $json = Get-Content $vrpath -Raw | ConvertFrom-Json
            foreach ($rt in @($json.runtime)) {
                if ($rt -and (Test-Path $rt)) { return $rt }
            }
        } catch {
            Write-Host "  The installer could not read $vrpath." -ForegroundColor Yellow
        }
    }
    return $null
}

$vrRuntimePath = Get-VrRuntimePath
if (-not $vrRuntimePath) {
    Fail "The installer could not find SteamVR. Make sure that SteamVR is installed. If SteamVR has not started on this PC before, start SteamVR one time. Close Steam. Then start the installer again."
}
Write-Host "SteamVR folder: $vrRuntimePath"

# --- Conflicting software: OpenVR-SpaceCalibrator ----------------------------
# Its driver hooks the same vrserver internals; both installed at once
# double-apply offsets, so it has to go before we install.
$spaceCalUninstall = Get-ItemProperty 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator' -ErrorAction SilentlyContinue
if ($spaceCalUninstall) {
    Write-Host ""
    Write-Host "OpenVR-SpaceCalibrator is installed. If two calibrators operate together, they apply the calibration two times." -ForegroundColor Yellow
    $answer = if ($Unattended) { 'N' } else { Read-Host "Uninstall it and continue? [Y/N]" }
    if ($answer -notmatch '^[Yy]') { Fail "The installer stopped and made no changes to Nova Calibrator. Remove OpenVR-SpaceCalibrator. Then start the installer again." }

    $spaceCalDir = (Get-ItemProperty 'HKLM:\Software\OpenVR-SpaceCalibrator\Main' -ErrorAction SilentlyContinue).'(default)'
    if (-not $spaceCalDir) { $spaceCalDir = Join-Path ${env:ProgramFiles} 'OpenVR-SpaceCalibrator' }
    Assert-QuestcalTree $spaceCalDir 'OpenVR-SpaceCalibrator'

    $spaceCalUninstaller = Join-Path $spaceCalDir 'Uninstall.exe'
    if (Test-Path $spaceCalUninstaller) {
        Write-Host "The installer removes OpenVR-SpaceCalibrator."
        $removedConflict = Start-Process -FilePath $spaceCalUninstaller -ArgumentList '/S', "_?=$spaceCalDir" -Wait -PassThru
        if ($removedConflict.ExitCode -ne 0) { Fail "The installer could not remove OpenVR-SpaceCalibrator (exit code $($removedConflict.ExitCode)). The installer made no changes to Nova Calibrator." }
        Remove-Item $spaceCalUninstaller -Force -ErrorAction SilentlyContinue
        Remove-QuestcalTree $spaceCalDir 'OpenVR-SpaceCalibrator'
    } else {
        # Half-removed install: uninstaller is gone, clean up its traces directly
        Remove-Item 'HKLM:\Software\OpenVR-SpaceCalibrator' -Recurse -Force -ErrorAction SilentlyContinue
        Remove-Item 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator' -Recurse -Force -ErrorAction SilentlyContinue
        Remove-Item (Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs\OpenVR-SpaceCalibrator.lnk') -Force -ErrorAction SilentlyContinue
        Remove-Item (Join-Path $env:ProgramData 'Microsoft\Windows\Start Menu\Programs\OpenVR-SpaceCalibrator.lnk') -Force -ErrorAction SilentlyContinue
    }
}

# --- The same app under its old name: QuestCalibrator -------------------------
# Nova Calibrator was called QuestCalibrator. Its own uninstaller removes the
# old app, driver, SteamVR registration and shortcut, and leaves the player's
# calibration and settings, which Nova Calibrator brings over when it first
# starts. Its driver folder is swept with the conflicts below: both drivers
# loaded at once would apply the calibration twice.
$oldInstallDir = (Get-ItemProperty 'HKLM:\Software\QuestCalibrator\Main' -ErrorAction SilentlyContinue).'(default)'
if (-not $oldInstallDir) { $oldInstallDir = Join-Path ${env:ProgramFiles} 'QuestCalibrator' }
Assert-QuestcalTree $oldInstallDir 'QuestCalibrator'
if (Test-Path (Join-Path $oldInstallDir 'Uninstall.ps1')) {
    Write-Host "QuestCalibrator is installed. The installer replaces it with Nova Calibrator. Nova Calibrator keeps your calibration and settings."
    $oldRemoval = Start-Process -FilePath 'powershell.exe' -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', ('"' + (Join-Path $oldInstallDir 'Uninstall.ps1') + '"'), '-Silent', '-UserLocalAppData', ('"' + $UserLocalAppData + '"')) -Wait -PassThru
    if ($oldRemoval.ExitCode -ne 0) { Fail "The installer could not remove QuestCalibrator (exit code $($oldRemoval.ExitCode)). The installer made no changes to Nova Calibrator." }
} elseif (Test-Path (Join-Path $oldInstallDir 'Uninstall.exe')) {
    Write-Host "QuestCalibrator (NSIS) is installed. The installer removes it first."
    $oldRemoval = Start-Process -FilePath (Join-Path $oldInstallDir 'Uninstall.exe') -ArgumentList '/S', "_?=$oldInstallDir" -Wait -PassThru
    if ($oldRemoval.ExitCode -ne 0) { Fail "The installer could not remove QuestCalibrator (exit code $($oldRemoval.ExitCode)). The installer made no changes to Nova Calibrator." }
    Remove-Item (Join-Path $oldInstallDir 'Uninstall.exe') -Force -ErrorAction SilentlyContinue
}
# Traces an uninstaller left, or a hand install never had one to remove.
foreach ($path in @('HKLM:\Software\QuestCalibrator',
    'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\QuestCalibrator',
    (Join-Path $env:ProgramData 'Microsoft\Windows\Start Menu\Programs\QuestCalibrator.lnk'))) {
    Remove-Item -LiteralPath $path -Recurse -Force -ErrorAction SilentlyContinue
}

# The driver folder is the actual conflict; sweep it even when the
# uninstaller ran (or was never registered), covering forks and manual installs.
foreach ($conflict in @('01spacecalibrator', '000spacecalibrator', '01questcalibrator')) {
    $driversRoot = [IO.Path]::GetFullPath((Join-Path $vrRuntimePath 'drivers'))
    $p = [IO.Path]::GetFullPath((Join-Path $driversRoot $conflict))
    if (-not $p.StartsWith($driversRoot.TrimEnd('\') + '\', [StringComparison]::OrdinalIgnoreCase)) {
        Fail 'The path of the other calibrator driver is not in the SteamVR "drivers" folder.'
    }
    Assert-QuestcalTree $p $conflict
    if (Test-Path -LiteralPath $p) {
        Write-Host "The installer removes this other calibrator driver: $p"
        for ($i = 1; $i -le 3; $i++) {
            try {
                Remove-QuestcalTree $p $conflict
                break
            } catch {
                if ($i -lt 3) {
                    Write-Host "  The driver files are locked (attempt $i of 3). The installer tries again in 2 seconds."
                    Start-Sleep -Seconds 2
                } else {
                    Fail "The installer could not remove this other calibrator driver: $p. The installer made no changes to Nova Calibrator. Start Windows again. Delete that driver folder. Then start the installer again."
                }
            }
        }
        if (Test-Path -LiteralPath $p) {
            Fail "This other calibrator driver is still there: $p. The installer made no changes to Nova Calibrator. Delete that driver folder. Then start the installer again."
        }
    }
}

# --- Install location ---------------------------------------------------------
$installDir = (Get-ItemProperty 'HKLM:\Software\NovaCalibrator\Main' -ErrorAction SilentlyContinue).'(default)'
if (-not $installDir) { $installDir = Join-Path ${env:ProgramFiles} 'NovaCalibrator' }
Assert-QuestcalTree $appSrc
Assert-QuestcalTree $driverSrc
Assert-QuestcalTree $installDir 'NovaCalibrator'

# Upgrade path: clear out whichever installer put files here last. Early builds
# shipped an NSIS installer, so Uninstall.exe has to be handled too - otherwise
# it is orphaned in Program Files with its registry entry overwritten by ours.
if (Test-Path (Join-Path $installDir 'Uninstall.ps1')) {
    Write-Host "Nova Calibrator is already installed. The installer replaces it."
    $oldRemoval = Start-Process -FilePath 'powershell.exe' -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', ('"' + (Join-Path $installDir 'Uninstall.ps1') + '"'), '-Silent', '-UserLocalAppData', ('"' + $UserLocalAppData + '"')) -Wait -PassThru
    if ($oldRemoval.ExitCode -ne 0) { Fail "The installer could not remove the earlier installation (exit code $($oldRemoval.ExitCode))." }
} elseif (Test-Path (Join-Path $installDir 'Uninstall.exe')) {
    Write-Host "Nova Calibrator (NSIS) is already installed. The installer removes it first."
    $oldRemoval = Start-Process -FilePath (Join-Path $installDir 'Uninstall.exe') -ArgumentList '/S', "_?=$installDir" -Wait -PassThru
    if ($oldRemoval.ExitCode -ne 0) { Fail "The installer could not remove the earlier installation (exit code $($oldRemoval.ExitCode))." }
    Remove-Item (Join-Path $installDir 'Uninstall.exe') -Force -ErrorAction SilentlyContinue
}

New-Item -ItemType Directory -Force -Path $installDir | Out-Null
Copy-Item (Join-Path $appSrc '*') -Destination $installDir -Recurse -Force
Copy-Item (Join-Path $packageDir 'Uninstall.ps1')      -Destination $installDir -Force
Copy-Item (Join-Path $packageDir 'FilesystemPolicy.ps1') -Destination $installDir -Force
Copy-Item (Join-Path $packageDir 'README-INSTALL.txt') -Destination $installDir -Force -ErrorAction SilentlyContinue
Copy-Item (Join-Path $packageDir 'LICENSE')            -Destination $installDir -Force -ErrorAction SilentlyContinue
Copy-Item (Join-Path $packageDir 'THIRD-PARTY-NOTICES.txt') -Destination $installDir -Force -ErrorAction SilentlyContinue
Copy-Item (Join-Path $packageDir 'THIRD-PARTY-NOTICES')     -Destination $installDir -Recurse -Force -ErrorAction SilentlyContinue

$appExe = Join-Path $installDir 'NovaCalibrator.exe'
if (-not (Test-Path $appExe)) { Fail "The installer could not copy the app to $installDir." }

$version = (Get-Item $appExe).VersionInfo.ProductVersion
if (-not $version) { $version = 'unknown' }

# --- Driver files -------------------------------------------------------------
$driverDest = Join-Path $vrRuntimePath 'drivers\01novacalibrator'
Assert-QuestcalTree $driverDest '01novacalibrator'
if (Test-Path $driverDest) {
    # The driver DLL may be locked by a recently-closed SteamVR or AV scan.
    # Retry a few times with a delay before giving up.
    $removed = $false
    for ($i = 1; $i -le 3; $i++) {
        try {
            Remove-QuestcalTree $driverDest '01novacalibrator'
            $removed = $true
            break
        } catch {
            if ($i -lt 3) {
                Write-Host "The driver files are locked (attempt $i of 3). The installer tries again in 2 seconds."
                Start-Sleep -Seconds 2
            }
        }
    }
    if (-not $removed) {
        Write-Host ""
        Write-Host "The installer could not remove the earlier driver. Its files are still locked." -ForegroundColor Yellow
        Write-Host "Usually, SteamVR stopped a short time ago, or an antivirus program examines the files." -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Do these steps:" -ForegroundColor Cyan
        Write-Host "  1. Start Windows again. This releases all file locks." -ForegroundColor Cyan
        Write-Host "  2. Immediately after Windows starts, start the installer again." -ForegroundColor Cyan
        Write-Host ""
        Pause-ForUser
        exit 1
    }
}
Copy-Item $driverSrc -Destination $driverDest -Recurse -Force
if (-not (Test-Path (Join-Path $driverDest 'bin\win64\driver_01novacalibrator.dll'))) {
    Fail "The installer could not copy the driver to $driverDest."
}
Write-Host "The installer copied the driver to $driverDest."

# --- Register with SteamVR ----------------------------------------------------
# NovaCalibrator.exe is a GUI binary: PowerShell's call operator does not wait
# for one and $LASTEXITCODE would be meaningless, so use Start-Process -Wait.
# -WorkingDirectory matters because the app resolves manifest.vrmanifest next to
# itself; -noui suppresses the result dialog so a scripted install never blocks.
function Invoke-App([string]$appArgument) {
    $proc = Start-Process -FilePath $appExe -ArgumentList @($appArgument, '-noui') `
        -WorkingDirectory $installDir -Wait -PassThru
    return $proc.ExitCode
}

$code = Invoke-App '-installmanifest'
if ($code -ne 0) {
    Fail "The installer could not register Nova Calibrator with SteamVR (exit code $code). Thus, Nova Calibrator does not start with SteamVR and does not show in the SteamVR dashboard."
}
Write-Host "The installer registered Nova Calibrator with SteamVR."

# Not fatal: the app also enables this itself every time it starts, so a failure
# here (usually SteamVR having never been run) heals on first launch.
$code = Invoke-App '-activatemultipledrivers'
if ($code -ne 0) {
    Write-Host "NOTE: The installer could not set the SteamVR option for multiple drivers to on (exit code $code)." -ForegroundColor Yellow
    Write-Host "      Nova Calibrator sets this option to on when it starts." -ForegroundColor Yellow
}

# --- Registry + shortcut ------------------------------------------------------
New-Item -Path 'HKLM:\Software\NovaCalibrator' -Force | Out-Null
Set-ItemProperty -Path 'HKLM:\Software\NovaCalibrator' -Name '(default)' -Value '' -ErrorAction SilentlyContinue
New-Item -Path 'HKLM:\Software\NovaCalibrator\Main' -Force | Out-Null
Set-ItemProperty -Path 'HKLM:\Software\NovaCalibrator\Main' -Name '(default)' -Value $installDir
New-Item -Path 'HKLM:\Software\NovaCalibrator\Driver' -Force | Out-Null
Set-ItemProperty -Path 'HKLM:\Software\NovaCalibrator\Driver' -Name '(default)' -Value $vrRuntimePath
# The overlay reads this at startup (UserInterface.cpp) to enable each module.
New-Item -Path $modulesKey -Force | Out-Null
Set-ItemProperty -Path $modulesKey -Name 'Lighthouse' -Value ([int]$installLighthouse) -Type DWord

$installedKb  = [int](((Get-ChildItem $installDir -Recurse -File | Measure-Object -Property Length -Sum).Sum) / 1KB)
$uninstallCmd = "powershell.exe -NoProfile -ExecutionPolicy Bypass -File `"$installDir\Uninstall.ps1`""

$uninstallReg = 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\NovaCalibrator'
New-Item -Path $uninstallReg -Force | Out-Null
Set-ItemProperty -Path $uninstallReg -Name 'DisplayName'          -Value 'Nova Calibrator'
Set-ItemProperty -Path $uninstallReg -Name 'DisplayVersion'       -Value $version
Set-ItemProperty -Path $uninstallReg -Name 'Publisher'            -Value 'VividNightmare'
Set-ItemProperty -Path $uninstallReg -Name 'DisplayIcon'          -Value $appExe
Set-ItemProperty -Path $uninstallReg -Name 'InstallLocation'      -Value $installDir
Set-ItemProperty -Path $uninstallReg -Name 'UninstallString'      -Value $uninstallCmd
Set-ItemProperty -Path $uninstallReg -Name 'QuietUninstallString' -Value "$uninstallCmd -Silent"
Set-ItemProperty -Path $uninstallReg -Name 'EstimatedSize'        -Value $installedKb -Type DWord
Set-ItemProperty -Path $uninstallReg -Name 'NoModify'             -Value 1 -Type DWord
Set-ItemProperty -Path $uninstallReg -Name 'NoRepair'             -Value 1 -Type DWord

$shortcutPath = Join-Path $env:ProgramData 'Microsoft\Windows\Start Menu\Programs\Nova Calibrator.lnk'
$wsh = New-Object -ComObject WScript.Shell
$shortcut = $wsh.CreateShortcut($shortcutPath)
$shortcut.TargetPath = $appExe
$shortcut.WorkingDirectory = $installDir
$shortcut.IconLocation = "$appExe,0"
$shortcut.Save()

Write-Host ""
Write-Host "Nova Calibrator $version is installed." -ForegroundColor Green
Write-Host ("Lighthouse module: " + $(if ($installLighthouse) { 'installed.' } else { 'not installed. To add it, start the installer again.' }))
Write-Host "The installer added a Nova Calibrator shortcut to the Start menu."
Write-Host "To use Nova Calibrator, start SteamVR."
Write-Host ""
Pause-ForUser
exit 0
