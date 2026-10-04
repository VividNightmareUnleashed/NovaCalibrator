# Fork release and package provenance

The inherited `v*` tags describe OpenVR-SpaceCalibrator releases. QuestCalibrator
tags must therefore use the unambiguous form
`questcalibrator-vMAJOR.MINOR.PATCH` (for example,
`questcalibrator-v1.0.1`). Do not retag or reuse an inherited version tag.

Releases are published in this repository,
[VividNightmareUnleashed/QuestCalibrator](https://github.com/VividNightmareUnleashed/QuestCalibrator).
Installed copies look for updates there, so it must keep that exact name and stay
public.

## Branches

Work lands on `alpha`, and prereleases are tagged there. `stable` is the default
branch and holds the last stable release, so visitors and anything that reads the
default branch get released code. It moves only when a stable release is
published, by a fast-forward to that release's tag, and nothing is committed to it
directly:

```powershell
git push origin "questcalibrator-vMAJOR.MINOR.PATCH^{commit}:refs/heads/stable"
```

Scheduled workflows start from the default branch; the weekly fuzz run checks out
`alpha` so it searches the code still changing.

Pushing a `questcalibrator-v*` tag runs `.github/workflows/release.yml`. It stops at
once in any other repository (a fork, or this one under another name). Otherwise, on
a clean Windows runner, it checks out the tag and the VirtualQuest commit it pins,
builds with the full solver suite (including the VirtualQuest scenarios and the
formal-model links), and in parallel jobs replays the pose hub traces through their
TLA+ model, runs the duplicate scan as an advisory gate, and runs Clang-Tidy over every
translation unit (`clang-tidy.yml`, where any first-party finding stops the release).
It checks the version against the tag and packages, signs, scans and writes the notes
with `install\package-release.ps1` (which runs `install\build-package.ps1`,
`install\sign-package.ps1` and `install\virustotal-scan.ps1`), creates a **draft**
release with the zip, its `.sha256`, a stable release's `.minisig` and notes carrying the hash and
the VirusTotal table, and then runs the install test on that draft. It refuses to run
if the tag already has a release. The release is written with the workflow's own
token; its secrets are `VIRTUALQUEST_DEPLOY_KEY` (a read-only deploy key on
VirtualQuest; required, so a release is never tested on less than a local build),
`VT_API_KEY` (without it the scan is skipped and the notes say so), and
`MINISIGN_SECRET_KEY` and `MINISIGN_PASSWORD`, the release signing key (Signing,
below; a stable tag fails without them). A run that failed before it created the draft
can be repeated for its tag from the Actions tab (`workflow_dispatch`); once the tag
has a draft the check refuses, and the way on is a new version.

`install\release.ps1` is the local fallback when Actions is unavailable, run from the
tagged commit with PowerShell 7 and your own `gh` login. It refuses unless VirtualQuest
is checked out, unchanged, at the commit the tag pins, builds with the full solver
suite, runs Clang-Tidy over every translation unit (skipped by `-DryRun`), and creates
the draft. Both paths package, scan and write the notes with
`install\package-release.ps1`, so the package, its `BUILD-INFO.txt` and the notes
cannot drift apart. It does not run the formal suite, replay the hub traces, run the
advisory duplicate scan or run the install test. To keep the tag
push from starting the workflow, put `[skip release]` in the message of the commit
the tag points to (the version bump). It checks that the working tree is clean and
that the tag is at HEAD and pushed; `-DryRun` stops before creating the release. Its
VirusTotal key comes from `$env:VT_API_KEY` or the git-ignored `.env` at the
repository root, and a stable release needs `-SigningKey <path to the .key file>`. Package output in `install/out/` and `install/test-out/` is never
committed.

The hosted release also proves the tag against VirtualQuest's formal suite on
Linux (`formal-assurance.yml`), all at once: the private core (TLC, Lean and
GenMC) on four runners and the complete numeric suite on four, and the draft waits
for them. This repository is public, so those jobs keep the private checks'
output off the log; a failure names the checks that failed, and is investigated
locally. The local release path does not run them, so let validation pass on the
release commit first. It runs the same checks, each only for a change that can
affect it (`tools/ci/formal-scope.py`): a check it skips has the same inputs as
when it last passed.

## Source preflight

- Confirm the release is authorized by the QuestCalibrator copyright holder.
- Start from a clean `alpha` checkout and fetch both `origin` and `upstream`.
- Review `git status`, `git log upstream/master..HEAD`, and
  `git rev-list --left-right --count upstream/master...HEAD`. Record the upstream
  base commit in the release notes.
- Set `common/Version.h` to the intended release version and confirm the overlay and
  driver version resources use it. A final release clears
  `QUESTCAL_VERSION_PRERELEASE_LABEL` to `""` and
  `QUESTCAL_VERSION_PRERELEASE_ORDINAL` to `0`, and drops the suffix from
  `QUESTCAL_VERSION_STRING`, in the same commit. The solver harness asserts that the
  string and the numbers agree, so a half-finished edit fails validation rather than
  shipping.
- Review `docs/vendored-dependencies.md`; resolve missing or changed notice material
  before producing a distributable binary.
- Run the Release build and solver harness locally, which catches a broken version
  edit and gives the scenario count for the tag message:

  ```powershell
  tools\validate-cpp.ps1 -Mode Build
  ```

- Push VirtualQuest's commits, then `alpha`, and wait for its validation workflow
  (build, harness, hub trace replay, duplicate scan, input-validation proofs,
  Clang-Tidy and the private core) on the exact release commit. Review every
  advisory clone it reports.
- A private check that fails in a hosted run is reproduced locally; its output
  never reaches the log:

  ```powershell
  pwsh -NoProfile -File VirtualQuest/formal/validate-local.ps1 -Setup
  ```

  Its `.local-validation/run-*/result.json` records the VirtualQuest commit, tool
  versions and every shard's status. See `VirtualQuest/formal/README.md` for tool
  setup and resources.

## Remote validation setup

The local private-suite entry point also supports a Linux runner with PowerShell 7.
Keep `VirtualQuest/formal/validate-local.ps1`, `check.ps1`, all model and proof
sources and both formal Dockerfiles in Git. `formal-assurance.yml` here documents
the hosted tool setup.

Provision Java 11 or newer, PowerShell 7, Git, elan and a working Linux Docker
daemon. Give the runner read access to the private VirtualQuest repository through
its secret store, then initialize the exact submodule commit. Run the same command
shown above with `-Setup`; it verifies TLC's pinned hash, selects the pinned Lean
toolchain and builds GenMC from the committed Dockerfile. Choose `-Shards` and
`-HeapGiB` to fit the runner's memory. A host that cannot run Docker needs a
container-capable executor for GenMC.

`.local-validation/` is disposable: tool downloads can be cached, while each run's
result JSON and shard logs should be collected as job artifacts. Credentials,
Docker Desktop settings, local socket repairs and machine-specific paths are not
inputs to this workflow and must not be committed.

## Commit, push, and tag

- Commit the version and release-note changes, then push `alpha`. Verify the exact
  release commit is visible on `origin`; do not tag an unpushed-only commit.
- Create an annotated `questcalibrator-vMAJOR.MINOR.PATCH` tag on that reviewed
  commit and include the version, validation result, and upstream base in its
  message.
- Push the tag explicitly and verify that the remote tag resolves to the recorded
  commit.
  Never use a force-updated release tag; issue a new patch version instead.

## Package and provenance record

- The release workflow runs on the pushed tag. The package's `BUILD-INFO.txt`
  records the tag, commit, the VirtualQuest commit, the scenario count, UTC build
  time, runner image, Visual Studio version, the packaging script's SHA-256 and the
  workflow run. The run keeps the zip, its `.sha256`, any `.minisig` and the VirusTotal table as an
  artifact for 90 days; copy them into the private release record. The default package name is
  `QuestCalibrator-MAJOR.MINOR.PATCH.zip`, from the executable's version resource.
- `build-package.ps1` writes the SHA-256 of every packaged file to `SHA256SUMS.txt`
  inside the package, and the zip's own hash to `<zip>.sha256` beside it.
- `virustotal-scan.ps1` hashes the executables, DLLs and scripts inside the zip, plus
  the zip itself, uploads any VirusTotal hasn't seen, and writes
  `<zip-name>.virustotal.md`, which the release notes carry. Below the table it
  names every detection (file, engine, label) and says what a model or heuristic
  label means and that Microsoft's engine is Defender's. Investigate any detection
  before publishing, and add what you found (and whether you reported it to the
  vendor) under that list.
- Test install, SteamVR startup and handshake, calibration, upgrade, and uninstall
  on a clean supported Windows environment, using the package from the draft.
  Confirm the original Space Calibrator driver is disabled so transforms are not
  applied twice.
- The zip's `.minisig` is the release key's signature (Signing, below). Packaging
  checks it against the key built into the release; check it once more by hand with
  the command the notes give before publishing.

## Publish the GitHub Release

- Open the draft the release workflow created. Replace the **Changes** section
  with the user-visible changes; it starts from the tag message. Keep the SHA-256
  and VirusTotal sections as generated.
- The draft carries only the package ZIP, its `.sha256` and, if stable, its `.minisig`. Never
  add, replace or delete assets by hand; if the package must change, tag a new
  version.
- Publish only when the install test passed. The release workflow runs it on the
  draft; run it again by hand to test against another earlier release, or after a
  local `release.ps1`:

  ```powershell
  gh workflow run install-test.yml --ref alpha -f tag=questcalibrator-vMAJOR.MINOR.PATCH
  ```

  It installs the attached ZIP on a clean Windows runner against the fake OpenVR
  runtime in `tools/fake-openvr-runtime`, then covers reinstall, removal of leftover
  Space Calibrator drivers, an upgrade from the newest published release (or the
  `previous` input), and uninstall. It checks files, the driver, registry, shortcut,
  and what the overlay registered with the runtime. Every push to `alpha` and
  `stable` already runs the same test on its own build's package; this run tests
  the package users download. It does not replace the SteamVR, handshake, and
  calibration checks above. Record the run URL, then publish the draft and record
  the final GitHub Release URL.
- Download the ZIP from the published release and verify its SHA-256 against the
  release notes once more.
- On `alpha`, point the README's VirusTotal badge at the new zip's report (its row
  in the release's VirusTotal table) and put the new version in its label.
- For a stable release, fast-forward `stable` to the release tag (Branches, above).

### Automatic-update contract

The overlay's opt-in updater reads the GitHub Releases API of the
`VividNightmareUnleashed/QuestCalibrator` repository without a token.
Keep these names exact or the release deliberately fails closed:

- Stable tag: `questcalibrator-vMAJOR.MINOR.PATCH` with no suffix.
- Package asset: `QuestCalibrator-MAJOR.MINOR.PATCH.zip`.
- Exactly one asset with that name, containing the normal single-folder package with
  `Install.ps1`, `Uninstall.ps1`, `app/QuestCalibrator.exe`, and the driver DLL.
- The published asset must expose GitHub's `sha256:` digest. Verify that digest against
  the provenance record before publishing.
- Exactly one signature asset, `QuestCalibrator-MAJOR.MINOR.PATCH.zip.minisig`, made
  with the release key, its trusted comment the package name.

Drafts, prereleases, inherited `v*` tags, packages without a SHA-256 digest or a
signature, duplicate canonical assets or signatures, and download URLs outside this
repository are not eligible. A newest release that is not eligible is refused, never
passed over for an older one. The updater checks the signature before it offers the
package and again before the installer starts. The
updater checks and downloads only after the user opts in; applying the package remains
an explicit action because Steam must be fully closed and Windows must approve the
elevated installer.

### Signing

Installed copies install only a package the release key signed. The updater reads
`<package>.minisig` with the release, checks it against the public key built into
the app (`Overlay/UpdateSigningKey.h`), and requires its signed trusted comment to be
the package name, so a release cannot carry an older package's genuine signature
under a newer name. The verifier is `Overlay/UpdateSignature.cpp`, on Monocypher.
`install\sign-package.ps1` signs with [minisign](https://jedisct1.github.io/minisign/),
for both release paths, and refuses a stable release that is unsigned or that the key
built into it does not verify. Prereleases are not signed, and their runs never see
the key: the updater never offers a prerelease.

The key lives in two places: offline with you, and in this repository's secrets for
the release workflow. Anyone who can change a workflow here can use it, so the
signature guards against a package replaced on GitHub or altered on the way, not
against a compromised account. Setting it up, once:

1. Install minisign (`winget install jedisct1.minisign`) and make the key pair:
   `minisign -G -p questcalibrator.pub -s questcalibrator.key`. Choose a password.
2. Keep `questcalibrator.key` and its password backed up offline. Never commit them.
3. Add the secrets: `gh secret set MINISIGN_SECRET_KEY < questcalibrator.key` and
   `gh secret set MINISIGN_PASSWORD` (it asks for the value).
4. Put the second line of `questcalibrator.pub` in `ReleaseSigningPublicKey` in
   `Overlay/UpdateSigningKey.h`, and commit it. Until then a build takes no update
   and a stable tag fails at packaging.

Versions before the first signed release ignore the `.minisig` and update to it as
before; from then on each version checks the next. A build trusts exactly the key it
was built with, so the key is for good: a release signed with another key reaches no
installed copy, and everyone installs it by hand. If the key is lost or leaks, make a
new pair, put its public key in the header, release with it, and say in the notes
that this version must be installed by hand.

Turn on release immutability (Settings, General, Releases, Enable release
immutability). Once a release is published, its tag and assets can then no longer be
changed or deleted, so a published package cannot be swapped for another with a
stolen token. GitHub applies it to every release published after it is turned on,
prereleases included; it has no per-release choice. Drafts are still edited freely,
and the rules above never change a published release's assets anyway.

### Prerelease builds

A prerelease sets both prerelease macros in `common/Version.h` (`"alpha"` and `3` give
`1.2.0-alpha.3`) so the build reports what it actually is. Before this the alphas
declared the bare `1.2.0`, the number of the release they precede, and every alpha
build reported a version it was not.

Prereleases are their own delivery lane, not a step on the stable one. They are
installed by hand and left by hand:

- A build with a prerelease label never asks the stable feed for anything. The updater
  skips the check and reports which prerelease this is, and `SelectReleaseCandidate`
  refuses a prerelease caller outright so no later path can hand a tester a stable
  package by accident.
- Testers move to a stable release by installing it by hand. Say so in the prerelease
  notes: an alpha will not update itself when the release it precedes is published.
- Prerelease tags stay out of the stable feed as before: tag them
  `questcalibrator-vMAJOR.MINOR.PATCH-LABEL.N` and mark the GitHub Release as a
  prerelease.

Two consequences of the suffix reaching the version resources:

- The numeric `FILEVERSION` has only four integers and cannot express a prerelease, so
  `1.2.0-alpha.3` and `1.2.0` both carry `1,2,0,0`. Never distinguish a prerelease from
  its release by the numeric file version; read the `FileVersion` string, which does
  carry the suffix.
- `install\build-package.ps1` names the ZIP from that string, so a prerelease package
  is named for the prerelease. Stable releases are unaffected, and the exact
  `QuestCalibrator-MAJOR.MINOR.PATCH.zip` asset name above still applies to them.
