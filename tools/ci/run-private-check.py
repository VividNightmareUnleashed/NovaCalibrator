#!/usr/bin/env python3
"""Run one private VirtualQuest proof command with its output kept off the log.

This repository is public, so anyone can read its Actions logs and artifacts,
while VirtualQuest is private. The command's output goes to a file in the
runner's temp folder that is never printed or uploaded; investigate a failure
locally. With --raw, the check.ps1 record the command writes is read for the
log to show how many checks passed and the names of any that did not; the
record itself is neither printed nor kept.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def summarize(label, checks):
    counts = {}
    for check in checks:
        counts[check.get("status")] = counts.get(check.get("status"), 0) + 1
    print(f"{label}: {counts.get('passed', 0)} passed, {counts.get('failed', 0)} failed, "
          f"{counts.get('skipped', 0)} skipped")
    for check in checks:
        if check.get("status") != "passed":
            print(f"  {check.get('status')}: {check.get('name')}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True)
    parser.add_argument("--raw", type=Path, help="the record the command writes")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("no command to run")
    temp = Path(os.environ.get("RUNNER_TEMP") or tempfile.gettempdir())
    log = temp / ("private-" + "".join(c if c.isalnum() else "-" for c in args.label) + ".log")
    with log.open("wb") as output:
        code = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT).returncode
    if args.raw is None:
        print(f"{args.label}: " + ("passed" if code == 0 else f"failed (exit {code})"))
        return code

    try:
        raw = json.loads(args.raw.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        print(f"{args.label}: wrote no record (exit {code})")
        return code or 1
    checks = [{key: check[key] for key in ("name", "status") if key in check}
              for check in raw.get("checks") or []]
    summarize(args.label, checks)
    names = {check.get("name") for check in checks}
    succeeded = (code == 0 and bool(checks) and raw.get("complete") is True and raw.get("success") is True
                 and raw.get("sourcesUnchanged") is True and set(raw.get("expected") or []) <= names
                 and all(check.get("status") == "passed" for check in checks))
    return 0 if succeeded else (code or 1)


if __name__ == "__main__":
    sys.exit(main())
