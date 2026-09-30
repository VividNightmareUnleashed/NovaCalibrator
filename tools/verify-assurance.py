#!/usr/bin/env python3
"""Validate complete, exact-pair formal evidence before a release.

Evidence is a local assurance record, not an authenticated third-party attestation.
The release workflow still separately gates the Windows build and integration tests.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


REQUIRED_SUITES = {"private-core", "input-validation", "contracts", "hub-traces"}


def git(repo, *arguments):
    return subprocess.check_output(["git", "-C", str(repo), *arguments], text=True).strip()


def source_identity(quest, virtual):
    identities = {}
    for name, root in (("QuestCalibrator", quest), ("VirtualQuest", virtual)):
        paths = subprocess.check_output(
            ["git", "-C", str(root), "ls-files", "-z", "--cached", "--others", "--exclude-standard"]
        ).decode().split("\0")
        hashes = {}
        for path in sorted(set(paths)):
            file = root / path
            if not file.is_file():
                continue
            if file.suffix.lower() in {".h", ".cpp", ".c", ".lean", ".tla", ".cfg", ".ps1", ".py", ".yml", ".json", ".inc", ".g", ".vcxproj", ".targets"} or file.name == "Dockerfile":
                hashes[path] = hashlib.sha256(file.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
        identities[name] = {"commit": git(root, "rev-parse", "HEAD"), "files": hashes}
    return identities


def validate(record, identity, expected, required=REQUIRED_SUITES, release=True):
    if record.get("schemaVersion") != 2:
        raise ValueError("Unsupported assurance record")
    if record.get("sources") != identity or not record.get("sourcesUnchanged"):
        raise ValueError("Assurance sources differ from this exact source pair")
    if not record.get("success"):
        raise ValueError("Assurance run did not succeed")
    if release and (not record.get("fullSuite") or record.get("dirty")):
        raise ValueError("A partial or uncommitted-source run cannot satisfy a release")
    suites = record.get("suites", {})
    for suite in required:
        entry = suites.get(suite, {})
        checks = entry.get("checks", [])
        names = [check.get("name") for check in checks]
        if not entry.get("complete") or not entry.get("tools"):
            raise ValueError(f"{suite}: missing completion or tool versions")
        if len(names) != len(set(names)) or set(names) != set(expected[suite]):
            raise ValueError(f"{suite}: missing, duplicate or unexpected checks")
        if any(check.get("status") != "passed" for check in checks):
            raise ValueError(f"{suite}: failed or skipped checks")
    return True


def expected_checks(virtual, source, pwsh="pwsh"):
    expected = {"contracts": ["A01", "A02", "A03", "P01", "P02", "P03", "P04", "P05", "S04", "S07", "N05", "N06"]}
    for suite, selection in (("private-core", ["-Core"]), ("input-validation", ["-Only", "input-validation"])):
        result = subprocess.check_output([pwsh, "-NoProfile", "-File", str(virtual / "formal/check.ps1"), "-ListChecks", "-SourceRoot", str(source), *selection], text=True)
        checks = json.loads(result)
        if isinstance(checks, dict):
            checks = [checks]
        expected[suite] = [check["Name"] for check in checks]
    # These names come from the committed implementation trace generator.
    # A trace record stores its exact emitted names; enforce positives plus all
    # ten registered negative traces, rather than accepting an empty artifact.
    expected["hub-traces"] = trace_names(virtual)
    return expected


def trace_names(virtual):
    import re
    text = (virtual / "Tests/FormalConformanceTests.cpp").read_text(encoding="utf-8-sig")
    match = re.search(r"const int perConsumer = (\d+);", text)
    if not match or '"hub-%s-%02d.tla"' not in text:
        raise ValueError("Trace generator changed; update its correspondence contract")
    positives = [f"Trace-{consumer}-{index:02d}" for consumer in ("collector", "monitor") for index in range(int(match[1]))]
    mutants = ["Trace-" + name for name in ("HoleNotClosed", "OverflowUncounted", "BoundaryNotInHole", "GapAfterPrefix", "BacklogKeptAtBoundary", "MonitorBeforeFix", "BoundaryReadBeforeDrain", "PerDrainLeash", "CollectorPerPass", "NoRunBoundaryCheck")]
    return positives + mutants


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quest", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--virtual", type=Path)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--pwsh", default="pwsh")
    args = parser.parse_args()
    virtual = args.virtual or args.quest / "VirtualQuest"
    expected = expected_checks(virtual, args.quest, args.pwsh)
    record = json.loads(args.evidence.read_text(encoding="utf-8-sig"))
    validate(record, source_identity(args.quest, virtual), expected)
    if git(args.quest, "rev-parse", "HEAD:VirtualQuest") != git(virtual, "rev-parse", "HEAD"):
        raise ValueError("QuestCalibrator submodule pin does not match the verified VirtualQuest commit")
    print("Complete formal evidence matches both exact release commits.")


if __name__ == "__main__":
    main()
