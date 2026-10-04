#requires -Version 7
# Signs a stable release package with the release key, for
# install\package-release.ps1: writes <package>.minisig with minisign, its
# trusted comment the package name, which the updater requires
# (Overlay/UpdateSignature.h), so one release's signature cannot pass for
# another's package. Then checks the signature with the public key the build
# trusts (Overlay/UpdateSigningKey.h), so a release whose own updater could not
# verify the next one, or signed with another key, is refused. Prereleases are
# not signed: the updater only ever offers stable releases.
#
# The secret key comes from -SecretKeyFile, or else from $env:MINISIGN_SECRET_KEY
# (the key file's text, as the release workflow passes its secret), and its
# password from $env:MINISIGN_PASSWORD; without it minisign asks. Returns the
# signature path and the public key it was checked with.
[CmdletBinding()]
param(
    [Parameter(Mandatory)] [string]$Package,
    [string]$SecretKeyFile = '',
    # minisign.exe; by default the one on PATH.
    [string]$Minisign = '',
    # The public key line the build trusts; by default the one in
    # Overlay/UpdateSigningKey.h.
    [string]$PublicKey
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not $PSBoundParameters.ContainsKey('PublicKey')) {
    $header = Get-Content -LiteralPath (Join-Path $repoRoot 'Overlay\UpdateSigningKey.h') -Raw
    $PublicKey = [regex]::Match($header, 'ReleaseSigningPublicKey\[\]\s*=\s*"([^"]*)"').Groups[1].Value
}
$name = Split-Path -Leaf $Package
$signature = "$Package.minisig"
Remove-Item -LiteralPath $signature -ErrorAction SilentlyContinue
if (-not $PublicKey) {
    throw 'Overlay/UpdateSigningKey.h has no public key, so this build could never verify an update. Add the release key (docs/releasing.md, Signing) and tag again.'
}
if (-not $Minisign) { $Minisign = (Get-Command minisign -ErrorAction SilentlyContinue).Source }
if (-not $Minisign) { throw 'minisign was not found: install it (winget install jedisct1.minisign) or pass -Minisign.' }

$temporaryKey = $null
if (-not $SecretKeyFile -and $env:MINISIGN_SECRET_KEY) {
    $temporaryKey = Join-Path ([IO.Path]::GetTempPath()) ('questcal-' + [Guid]::NewGuid().ToString('N') + '.key')
    [IO.File]::WriteAllText($temporaryKey, $env:MINISIGN_SECRET_KEY.Trim() + "`n")
    $SecretKeyFile = $temporaryKey
}
if (-not $SecretKeyFile) { throw 'A stable release must be signed: set MINISIGN_SECRET_KEY or pass -SecretKeyFile.' }
try {
    $version = [IO.Path]::GetFileNameWithoutExtension($name) -replace '^QuestCalibrator-', ''
    $arguments = @('-S', '-s', $SecretKeyFile, '-m', $Package, '-x', $signature, '-t', $name,
        '-c', "QuestCalibrator $version, signed with the release key")
    # minisign reads the password from standard input when that is not a
    # terminal, and asks for it when it is.
    if ($env:MINISIGN_PASSWORD) { $env:MINISIGN_PASSWORD | & $Minisign @arguments | Out-Host }
    else { & $Minisign @arguments | Out-Host }
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $signature)) { throw "Signing $name failed." }
} finally {
    if ($temporaryKey) { Remove-Item -LiteralPath $temporaryKey -Force -ErrorAction SilentlyContinue }
}

# -Q prints the trusted comment alone.
$comment = & $Minisign -V -Q -P $PublicKey -m $Package -x $signature
if ($LASTEXITCODE -ne 0 -or "$comment".Trim() -ne $name) {
    Remove-Item -LiteralPath $signature -ErrorAction SilentlyContinue
    throw "The key this build trusts ($PublicKey) does not verify the signature, so it was made with another key. Sign with the release key, or fix Overlay/UpdateSigningKey.h and tag again."
}
Write-Host "Signed $name; the key this build trusts verifies it."
[pscustomobject]@{ Signature = $signature; PublicKey = $PublicKey }
