<#
.SYNOPSIS
    Checks the two readers of a pose recording that are not the overlay's own
    use of them: the C++ reader compiled by itself, and tools\read-recording.py.

.DESCRIPTION
    docs\recording-format.md is the contract a reader is written from, and
    Tests\recordings holds reference recordings with the listing a correct
    reader prints for each. The harness already checks that the generator
    still makes those bytes and that the C++ listing is the stored one. This
    script adds what the harness cannot:

      1. tools\novapose-listing.cpp compiles with only a copy of
         Overlay\PoseRecordingFormat.h and Overlay\PoseRecordingListing.h on
         its include path: the proof that the reader needs nothing of Nova
         Calibrator, SteamVR or a UI library.
      2. That program's listing of every reference recording is the stored one.
      3. tools\read-recording.py, a second reader written from the document
         and sharing no code with the first, prints the same (when Python 3
         is installed).
      4. tools\novapose-convert.cpp compiles the same way, and writes each
         reference recording again as a compact one (format version 2) and
         as a raw one. Both readers print one listing for each result, so
         they agree on every bit of every sample a block gives; and the
         compact copy of a whole recording holds the same records and
         samples as the recording it was made from.

    Run it after a change to the format, the listing or either reader.

.PARAMETER WriteReference
    Write Tests\recordings\*.novapose and *.txt from the generator
    (Tests\ReferenceRecordings.h) first, with the harness that is already
    built. Only for a change to the format that is made on purpose, together
    with docs\recording-format.md; build again afterwards, because the harness
    has the files built in.

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File tools\check-recording-readers.ps1
#>
[CmdletBinding()]
param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',
    [switch]$WriteReference
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$reference = Join-Path $root 'Tests\recordings'
$out = Join-Path $root 'x64\recording-readers'
$alone = Join-Path $out 'include'
New-Item -ItemType Directory -Force $alone | Out-Null

# A reader's listing as it printed it: lines that end in a line feed.
function Get-Listing([string]$program, [string[]]$arguments) {
    $ErrorActionPreference = 'Continue'
    $lines = @(& $program @arguments 2>$null | ForEach-Object { [string]$_ })
    return ($lines -join "`n") + "`n"
}

if ($WriteReference) {
    $harness = Join-Path $root "x64\$Configuration\SolverTests.exe"
    if (-not (Test-Path -LiteralPath $harness -PathType Leaf)) { throw "Build the harness first: $harness is not there." }
    $env:NOVA_RECORDER_WRITE_REFERENCE = $reference
    try { & $harness '--test-case=pose recording' | Out-Null }
    finally { $env:NOVA_RECORDER_WRITE_REFERENCE = $null }
    Write-Output 'Wrote Tests\recordings from the generator. Build again: the harness has the files built in.'
}

# 1. The reader alone: its two headers, copied to a folder that holds nothing else.
foreach ($header in 'PoseRecordingFormat.h', 'PoseRecordingListing.h') {
    Copy-Item -LiteralPath (Join-Path $root "Overlay\$header") -Destination $alone -Force
}
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw 'The Visual Studio 2022 C++ build tools were not found.' }
$vcvars = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
foreach ($line in (& cmd.exe /c "`"$vcvars`" >nul 2>nul && set")) {
    if ($line -match '^([^=]+)=(.*)$') { [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process') }
}
$listingExe = Join-Path $out 'novapose-listing.exe'
$convertExe = Join-Path $out 'novapose-convert.exe'
foreach ($tool in @(@{ Source = 'tools\novapose-listing.cpp'; Exe = $listingExe },
        @{ Source = 'tools\novapose-convert.cpp'; Exe = $convertExe })) {
    $ErrorActionPreference = 'Continue'
    # Objects land in the working directory.
    Push-Location $out
    $compile = & cl.exe /nologo /std:c++17 /EHsc /W4 /WX /O2 /I $alone `
        (Join-Path $root $tool.Source) "/Fe$($tool.Exe)" 2>&1
    $compileExit = $LASTEXITCODE
    Pop-Location
    $ErrorActionPreference = 'Stop'
    if ($compileExit -ne 0) {
        $compile | Write-Output
        throw "$($tool.Source) did not compile by itself: it needs something outside the reader's two headers and the standard library."
    }
}

# 2 and 3. Both readers against the stored listings.
$python = Get-Command python -ErrorAction SilentlyContinue
$pythonRuns = $false
if ($python) {
    # Windows answers "python" with a Store stub where none is installed.
    $ErrorActionPreference = 'Continue'
    try { $null = & $python.Source --version 2>&1; $pythonRuns = $LASTEXITCODE -eq 0 } catch { $pythonRuns = $false }
    $ErrorActionPreference = 'Stop'
}
$recordings = @(Get-ChildItem -LiteralPath $reference -Filter '*.novapose')
if ($recordings.Count -eq 0) { throw 'Tests\recordings holds no recordings.' }
$readerScript = Join-Path $root 'tools\read-recording.py'
foreach ($recording in $recordings) {
    $listing = [IO.File]::ReadAllText([IO.Path]::ChangeExtension($recording.FullName, '.txt'))
    if ((Get-Listing $listingExe @($recording.FullName)) -cne $listing) {
        throw "The C++ reader's listing of $($recording.Name) is not Tests\recordings\$($recording.BaseName).txt."
    }
    if ($pythonRuns -and (Get-Listing $python.Source @('-B', $readerScript, '--listing', $recording.FullName)) -cne $listing) {
        throw "tools\read-recording.py's listing of $($recording.Name) is not Tests\recordings\$($recording.BaseName).txt."
    }
}

# 4. Each recording again in both modes: one listing from both readers for
#    each copy, and a compact copy that holds what its recording holds.
$copies = Join-Path $out 'copies'
New-Item -ItemType Directory -Force $copies | Out-Null
$converted = 0
foreach ($recording in $recordings) {
    $whole = [IO.File]::ReadAllText([IO.Path]::ChangeExtension($recording.FullName, '.txt')).EndsWith("file: complete`n")
    foreach ($mode in 'compact', 'raw') {
        $copy = Join-Path $copies "$($recording.BaseName)-$mode.novapose"
        $arguments = @($recording.FullName, $copy) + $(if ($mode -eq 'raw') { @('--raw') } else { @() })
        $ErrorActionPreference = 'Continue'
        $null = & $convertExe @arguments 2>&1
        $convertExit = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
        if ($convertExit -ne 0) { throw "novapose-convert could not write $($recording.Name) as a $mode recording." }
        $fromCpp = Get-Listing $listingExe @($copy)
        if (-not $fromCpp.Contains("`nfile: complete`n")) {
            throw "The $mode copy of $($recording.Name) is not a whole recording to the C++ reader."
        }
        if ($pythonRuns -and (Get-Listing $python.Source @('-B', $readerScript, '--listing', $copy)) -cne $fromCpp) {
            throw "The two readers print different listings for the $mode copy of $($recording.Name)."
        }
        # A recording that was cut short has no whole copy to compare with.
        if ($whole) {
            $ErrorActionPreference = 'Continue'
            $null = & $convertExe '--compare' $recording.FullName $copy 2>&1
            $compareExit = $LASTEXITCODE
            $ErrorActionPreference = 'Stop'
            if ($compareExit -ne 0) {
                throw "The $mode copy of $($recording.Name) does not hold the records and samples of the recording."
            }
        }
        $converted++
    }
}
Write-Output ("The C++ reader and the converter compile by themselves, and the reader's listings of the $($recordings.Count) reference recordings are the stored ones" +
    $(if ($pythonRuns) { '; tools\read-recording.py prints the same. ' } else { '. Python 3 is not installed: tools\read-recording.py was not checked. ' }) +
    "Each recording was written again as a compact and as a raw one ($converted copies)" +
    $(if ($pythonRuns) { ', and both readers print one listing for each copy.' } else { '.' }))
