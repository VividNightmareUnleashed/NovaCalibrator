#!/usr/bin/env python3
"""Assemble complete named proof suites; fail closed on stale or partial input."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path


def load(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quest", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--virtual", type=Path)
    parser.add_argument("--contracts", type=Path, required=True)
    parser.add_argument("--core", type=Path, nargs="+", required=True)
    parser.add_argument("--numeric", type=Path, nargs="+", required=True)
    parser.add_argument("--traces", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--linux-only", action="store_true")
    args = parser.parse_args()
    virtual = args.virtual or args.quest / "VirtualQuest"
    spec = importlib.util.spec_from_file_location("assurance", args.quest / "tools/verify-assurance.py")
    assurance = importlib.util.module_from_spec(spec);spec.loader.exec_module(assurance)
    identity = assurance.source_identity(args.quest, virtual)
    record = load(args.contracts)
    if record.get("sources") != identity or not record.get("success") or not record.get("sourcesUnchanged"):
        raise ValueError("Contracts do not describe this immutable source pair")
    expected = assurance.expected_checks(virtual, args.quest)
    for suite, paths in (("private-core", args.core), ("input-validation", args.numeric), ("hub-traces", [args.traces] if args.traces else [])):
        if not paths and not args.linux_only:
            raise ValueError("Hub trace evidence is required")
        if not paths:
            continue
        checks = []
        tools = {}
        for path in paths:
            result = load(path)
            if result.get("schemaVersion") != 2 or not result.get("sourcesUnchanged") or not result.get("complete") or not result.get("success"):
                raise ValueError(f"Incomplete or failed proof result: {path}")
            if not result.get("sourceHashes") or not result.get("tools"):
                raise ValueError(f"Missing tool/source provenance: {path}")
            for source, digest in result["sourceHashes"].items():
                repository, relative = source.split(":", 1)
                root = args.quest if repository == "QuestCalibrator" else virtual
                file = root / relative
                if not file.is_file() or hashlib.sha256(file.read_bytes().replace(b"\r\n", b"\n")).hexdigest() != digest:
                    raise ValueError(f"Stale checker input: {source}")
            for name, version in result["tools"].items():
                if name in tools and tools[name] != version:
                    raise ValueError(f"Inconsistent tool identity: {name}")
                tools[name] = version
            checks.extend(result["checks"])
        record["suites"][suite] = {"complete": True, "tools": tools, "checks": checks}
    record["fullSuite"] = not args.linux_only
    record["dirty"] = any(assurance.git(root,"status","--porcelain","--untracked-files=no") for root in (args.quest,virtual))
    required = assurance.REQUIRED_SUITES - ({"hub-traces"} if args.linux_only else set())
    assurance.validate(record, identity, expected, required=required, release=not args.linux_only)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record,indent=2)+"\n")
    print("Complete selected assurance suites match the source pair.")


if __name__ == "__main__":
    main()
