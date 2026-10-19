# Filesystem preconditions for owned install/removal trees. A concurrent process
# must not replace checked paths during the operation; Win32 path races are
# outside this sequential policy. Reparse points are refused, including parents.
function Assert-QuestcalTree([string]$Path, [string]$ExpectedLeaf = '') {
    if (-not $Path -or -not [IO.Path]::IsPathRooted($Path)) { throw 'An installation path must be absolute.' }
    $full = [IO.Path]::GetFullPath($Path).TrimEnd([IO.Path]::DirectorySeparatorChar, [IO.Path]::AltDirectorySeparatorChar)
    if (-not $full -or $full -eq [IO.Path]::GetPathRoot($Path).TrimEnd([IO.Path]::DirectorySeparatorChar)) { throw 'A drive root cannot be an installation folder.' }
    if ($ExpectedLeaf -and [IO.Path]::GetFileName($full) -ine $ExpectedLeaf) { throw "The folder name is not '$ExpectedLeaf', so the folder stays unchanged: $Path" }
    $ancestor = $full
    while ($ancestor) {
        if (Test-Path -LiteralPath $ancestor) {
            $item = Get-Item -LiteralPath $ancestor -Force -ErrorAction Stop
            if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "This path is a link (reparse point), so the folder stays unchanged: $ancestor" }
        }
        $ancestor = [IO.Path]::GetDirectoryName($ancestor)
    }
    if (Test-Path -LiteralPath $full) {
        foreach ($item in Get-ChildItem -LiteralPath $full -Recurse -Force -ErrorAction Stop) {
            if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "This item is a link (reparse point), so the folder stays unchanged: $($item.FullName)" }
        }
    }
}

function Remove-QuestcalTree([string]$Path, [string]$ExpectedLeaf) {
    Assert-QuestcalTree $Path $ExpectedLeaf
    if (Test-Path -LiteralPath $Path) { Remove-Item -LiteralPath $Path -Recurse -Force -ErrorAction Stop }
    if (Test-Path -LiteralPath $Path) { throw "The removal of this folder did not complete: $Path" }
}
