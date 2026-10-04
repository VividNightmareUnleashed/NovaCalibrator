"""Which of the validation workflow's formal jobs a change needs to run.

Each formal job is a deterministic check of particular files. Run again on
the same files, it gives the same answer, so a push that touches none of
them cannot fail it. This works out each job's inputs from what the job
itself reads, rather than from a list kept by hand:

- the files its scripts name in string literals (sources they compile,
  mutation targets), and every file those scripts name in turn;
- every C or C++ source of its harnesses, and all they #include, transitively.

A job then runs when the change touches one of its public inputs. Any job runs
when the VirtualQuest pin moves (its harnesses and models changed), or when
the workflows or the tools that run the checks change. A base
that cannot be compared (a new branch, a force push, a manual run) runs all of
them. An input this misses is still checked by the release workflow, which
runs the whole suite on the tag.

  python tools/ci/formal-scope.py --base <commit>       # what the change needs
  python tools/ci/formal-scope.py --list core           # a job's public inputs

With GITHUB_OUTPUT set it also writes <job>=true|false lines there. Needs
VirtualQuest checked out at the pinned commit.
"""
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VIRTUAL = ROOT / 'VirtualQuest'
FORMAL = VIRTUAL / 'formal'

# Each job: the harness folders whose C/C++ sources it compiles, and the
# scripts that drive it. The inputs are derived from these.
JOBS = {
    # formal/check.ps1 -Only input-validation: ESBMC and Gappa.
    'input_validation': {'folders': [FORMAL / 'input-validation'], 'exclude': [],
                         'scripts': [FORMAL / 'check.ps1']},
    # formal-assurance.yml on a push: check.ps1 -Core, TLC, Lean and
    # GenMC over VirtualQuest's own models.
    'core': {'folders': [FORMAL], 'exclude': [FORMAL / 'input-validation'],
             'scripts': [FORMAL / 'check.ps1', ROOT / 'tools' / 'ci' / 'run-private-check.py']},
}

# A change to any of these reruns every job: they run or judge the checks.
SHARED = [re.compile(p) for p in (
    r'^VirtualQuest$',
    r'^\.github/workflows/(validation|formal-assurance)\.yml$',
    r'^tools/ci/',
)]

CODE = {'.c', '.cc', '.cpp', '.h', '.hpp'}
SCRIPTS = {'.py', '.ps1', '.psm1'}
LITERAL = re.compile(r'''(?:"([^"\n]{3,200})"|'([^'\n]{3,200})')''')
INCLUDE = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.M)
FIRST_PARTY = ['Driver', 'Overlay', 'common', 'Tests', 'install', 'tools']


def inside(path, folder):
    try:
        path.relative_to(folder)
        return True
    except ValueError:
        return False


def by_name():
    """First-party files by bare name, for literals that name only the file."""
    names = {}
    for folder in FIRST_PARTY:
        for path in (ROOT / folder).rglob('*'):
            if path.is_file():
                names.setdefault(path.name, []).append(path.resolve())
    return names


NAMES = by_name()
# Build output and other untracked files are never part of a change.
TRACKED = set(subprocess.run(['git', 'ls-files'], cwd=ROOT, capture_output=True,
                             text=True).stdout.splitlines())


def referenced(path):
    """Files a script names in its string literals."""
    try:
        text = path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return []
    found = []
    for match in LITERAL.finditer(text):
        literal = (match.group(1) or match.group(2)).replace('\\', '/').strip()
        if any(c in literal for c in '*?<>|$\n') or literal.endswith('/'):
            continue
        if literal.startswith('./'):
            literal = literal[2:]
        candidates = [base / literal for base in (path.parent, ROOT, VIRTUAL, FORMAL)]
        hit = next((c.resolve() for c in candidates if c.is_file()), None)
        if hit is None and '/' not in literal and len(NAMES.get(literal, [])) == 1:
            hit = NAMES[literal][0]
        if hit is not None and inside(hit, ROOT):
            found.append(hit)
    return found


def included(path):
    try:
        text = path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return []
    found = []
    for name in INCLUDE.findall(text):
        for base in (path.parent, ROOT, ROOT / 'lib', ROOT / 'lib' / 'openvr'):
            if (base / name).is_file():
                found.append((base / name).resolve())
                break
    return found


def inputs(job):
    """The public files (outside VirtualQuest) a job reads."""
    spec = JOBS[job]
    # (path, how many scripts away from the job's own). Includes are compile
    # inputs and are always followed. Names in a script are followed one
    # script deep: the job's scripts run the scripts they name (an installer
    # test), but what those name may only be looked at (a file whose syntax
    # a test parses), and following it further reaches files no check reads.
    queue = [(s.resolve(), 0) for s in spec['scripts'] if s.is_file()]
    for folder in spec['folders']:
        for path in folder.rglob('*'):
            if path.is_file() and path.suffix in CODE and not any(inside(path, e) for e in spec['exclude']):
                queue.append((path.resolve(), 0))
    seen = set()
    while queue:
        path, depth = queue.pop()
        if path in seen:
            continue
        seen.add(path)
        if path.suffix in CODE:
            queue.extend((p, depth) for p in included(path))
        elif path.suffix in SCRIPTS and depth <= 1:
            queue.extend((p, depth + 1) for p in referenced(path))
    return sorted(p.relative_to(ROOT).as_posix() for p in seen
                  if not inside(p, VIRTUAL) and p.relative_to(ROOT).as_posix() in TRACKED)


def changed(base, head='HEAD'):
    """Files the change touches, or None when the base cannot be compared."""
    if not base or set(base) == {'0'}:
        return None
    if subprocess.run(['git', 'cat-file', '-e', base + '^{commit}'], cwd=ROOT,
                      capture_output=True).returncode != 0:
        return None
    diff = subprocess.run(['git', 'diff', '--name-only', base, head], cwd=ROOT,
                          capture_output=True, text=True)
    return diff.stdout.split() if diff.returncode == 0 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--base', default='')
    # Another commit than HEAD, to see what a past change would have run.
    parser.add_argument('--head', default='HEAD')
    parser.add_argument('--list', choices=sorted(JOBS))
    args = parser.parse_args()
    if not (FORMAL / 'check.ps1').is_file():
        sys.exit('VirtualQuest is not checked out.')
    if args.list:
        print('\n'.join(inputs(args.list)))
        return

    files = changed(args.base, args.head)
    verdict = {}
    if files is None:
        verdict = {job: 'the base cannot be compared, so every job runs' for job in JOBS}
    else:
        shared = next((f for f in files if any(p.search(f) for p in SHARED)), None)
        for job in JOBS:
            if shared:
                verdict[job] = f'{shared} changed'
            else:
                read = set(inputs(job))
                hit = next((f for f in files if f in read), None)
                verdict[job] = f'{hit} changed' if hit else ''
    lines = []
    for job, reason in verdict.items():
        print(f'{job}: ' + (f'runs ({reason})' if reason else 'skipped, none of its inputs changed'))
        lines.append(f'{job}={"true" if reason else "false"}')
    if os.environ.get('GITHUB_OUTPUT'):
        with open(os.environ['GITHUB_OUTPUT'], 'a', encoding='utf-8') as out:
            out.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
