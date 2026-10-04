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
them.

In CI each job is compared with the last commit where it ran and passed
(--last-green), found in this repository's validation and release runs, not
with the previous push: a run cancelled by a newer push never proved its
change, so the push after it must not count it as proved. A release skips a
job the same way. With no such run, or no way to ask, the job runs.

  python tools/ci/formal-scope.py --base <commit>       # what the change needs
  python tools/ci/formal-scope.py --last-green          # as CI decides (GITHUB_TOKEN)
  python tools/ci/formal-scope.py --list core           # a job's public inputs

With GITHUB_OUTPUT set it also writes <job>=true|false lines there. Needs
VirtualQuest checked out at the pinned commit.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request
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
    # GenMC over VirtualQuest's own models. It compiles no public source, and
    # check.ps1 only hashes the headers the numeric harnesses name, so it has
    # no public input: it runs when VirtualQuest or the tools below change.
    'core': {'folders': [], 'exclude': [], 'scripts': []},
}

# The workflow jobs that prove each formal job on a commit, by name prefix:
# every one of them must have run and passed. validation.yml's and
# release.yml's (through formal-assurance.yml) both count.
PROVING = {
    'input_validation': ('input-validation (', 'formal-assurance / numeric ('),
    'core': ('private-core / private-core (', 'formal-assurance / private-core ('),
}
PROVING_WORKFLOWS = ('validation.yml', 'release.yml')

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


def github(path, attempts=3):
    """One GitHub API read for this repository, or None when it fails. A
    failed read is retried: one that fails for good makes the search look
    further back, which runs more checks than needed rather than fewer."""
    token = os.environ.get('GITHUB_TOKEN') or os.environ.get('GH_TOKEN')
    repo = os.environ.get('GITHUB_REPOSITORY', 'VividNightmareUnleashed/QuestCalibrator')
    if not token:
        return None
    request = urllib.request.Request(f'https://api.github.com/repos/{repo}/{path}', headers={
        'Authorization': f'Bearer {token}', 'Accept': 'application/vnd.github+json'})
    for attempt in range(1, attempts + 1):
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                return json.load(response)
        except urllib.error.HTTPError as error:
            failure = f'HTTP {error.code}'
            if error.code == 404:
                break
        except (OSError, ValueError) as error:
            failure = str(error) or type(error).__name__
        print(f'Reading {path} from GitHub failed ({failure}), attempt {attempt} of {attempts}.')
        if attempt < attempts:
            time.sleep(2 * attempt)
    return None


def last_green():
    """For each job, the newest commit where every workflow job proving it ran
    and passed, from the latest completed validation and release runs."""
    runs = []
    for workflow in PROVING_WORKFLOWS:
        page = github(f'actions/workflows/{workflow}/runs?status=completed&per_page=30')
        runs += (page or {}).get('workflow_runs', [])
    runs.sort(key=lambda run: run['created_at'], reverse=True)
    found = {}
    for run in runs:
        if len(found) == len(JOBS):
            break
        listing = github(f'actions/runs/{run["id"]}/jobs?per_page=100')
        if listing is None:
            print(f'The jobs of {run["name"]} run {run["id"]} could not be read; looking further back.')
            continue
        jobs = listing.get('jobs', [])
        for job, prefixes in PROVING.items():
            proving = [j for j in jobs if j['name'].startswith(prefixes)]
            if job not in found and proving and all(j['conclusion'] == 'success' for j in proving):
                found[job] = run['head_sha']
                print(f'{job}: last passed in {run["name"]} run {run["id"]}, at {run["head_sha"][:8]}')
    return found


def fetch(commit):
    """Make a commit's tree available in a shallow checkout; False if it can't be."""
    if subprocess.run(['git', 'cat-file', '-e', commit + '^{commit}'], cwd=ROOT,
                      capture_output=True).returncode == 0:
        return True
    return subprocess.run(['git', 'fetch', '--no-tags', '--depth=1', 'origin', commit], cwd=ROOT,
                          capture_output=True).returncode == 0


def reason_to_run(job, files):
    """Why the job must run given the files changed since its base, or ''."""
    shared = next((f for f in files if any(p.search(f) for p in SHARED)), None)
    if shared:
        return f'{shared} changed'
    read = set(inputs(job))
    hit = next((f for f in files if f in read), None)
    return f'{hit} changed' if hit else ''


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--base', default='')
    # Each job against the last commit where it passed, as CI decides.
    parser.add_argument('--last-green', action='store_true')
    # Another commit than HEAD, to see what a past change would have run.
    parser.add_argument('--head', default='HEAD')
    parser.add_argument('--list', choices=sorted(JOBS))
    args = parser.parse_args()
    if not (FORMAL / 'check.ps1').is_file():
        sys.exit('VirtualQuest is not checked out.')
    if args.list:
        print('\n'.join(inputs(args.list)))
        return

    verdict = {}
    if args.last_green:
        bases = last_green()
        for job in JOBS:
            base = bases.get(job)
            files = changed(base, args.head) if base and fetch(base) else None
            if files is None:
                verdict[job] = (f'its last green commit {base[:8]} cannot be compared' if base
                                else 'no run where it passed was found'), ''
            else:
                verdict[job] = reason_to_run(job, files), base
    else:
        files = changed(args.base, args.head)
        for job in JOBS:
            verdict[job] = ('the base cannot be compared, so every job runs' if files is None
                            else reason_to_run(job, files)), args.base
    lines = []
    for job, (reason, base) in verdict.items():
        since = f' since {base[:8]}, where it passed' if args.last_green and base else ''
        print(f'{job}: ' + (f'runs ({reason}{since})' if reason else f'skipped, none of its inputs changed{since}'))
        lines.append(f'{job}={"true" if reason else "false"}')
    if os.environ.get('GITHUB_OUTPUT'):
        with open(os.environ['GITHUB_OUTPUT'], 'a', encoding='utf-8') as out:
            out.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
