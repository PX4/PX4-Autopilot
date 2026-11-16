#!/usr/bin/env python3
"""Build a group of PX4 targets for the build_all_targets workflow.

Usage: build_all_runner.py [--slots N] TARGET [TARGET ...]
       build_all_runner.py --print-logs

With --slots 1 the targets build one after another in the checkout, exactly
like running `make TARGET` for each. With --slots N the targets are split
across N build slots that run concurrently: slot 0 is the checkout itself,
slots 1..N-1 are git worktrees of the same commit under .build_slots/.
NuttX compiles inside platforms/nuttx/NuttX/{nuttx,apps} rather than in
build/<target>/, so two builds can only run at once in separate checkouts.
Worktrees build the committed HEAD, not uncommitted changes in the checkout.

All submodules are fetched into the checkout before any build starts, and
each slot clones them from there (hardlinked, no network). The builds run
with GIT_SUBMODULES_ARE_EVIL=1 so CMake does not fetch or sync submodules
itself, which concurrent builds would do at the same time on the shared
.git/config.

Build output streams live with every line prefixed by its slot, target and
seconds since that target started (the mavsdk_tests runner convention), so
interleaved lines from concurrent builds stay attributable; compiler errors
and warnings are colored. Each target's unprefixed output is also kept in
.build_slots/logs/, and --print-logs prints those afterwards as one
collapsed group per target titled with its result. Every target is built even if an earlier
one fails, and the script exits non-zero if any target failed. Finally, the
slot build directories are moved into build/ so package_build_artifacts.sh
finds every target in one place.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

SLOTS_DIR = '.build_slots'
LOGS_DIR = f'{SLOTS_DIR}/logs'
RESULTS_FILE = f'{LOGS_DIR}/results.json'
FAILURE_EXCERPT_LINES = 60
FAILURE_TAIL_LINES = 5

ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
# ninja progress lines ("[12/990] Building CXX object ..."), noise in a failure excerpt
NINJA_PROGRESS = re.compile(r'^\[\d+/\d+\] ')
# linker memory report rows, e.g. "   FLASH_AXIM:     1405156 B      1952 KB     70.30%"
MEMORY_REGION = re.compile(r'^\s*(\S+):\s+\d+\s*[KMG]?B\s+\d+\s*[KMG]?B\s+([\d.]+)%')

# Makefile targets that build inside another target's build directory
# (Makefile metadata rules all run `make px4_sitl_default ...`).
METADATA_TARGETS = {'airframe_metadata', 'parameters_metadata',
                    'module_documentation', 'extract_events'}

print_lock = threading.Lock()

# Submodules are fetched before building; stop CMake's per-configure
# check_submodules.sh from syncing and updating them concurrently.
BUILD_ENV = {**os.environ, 'GIT_SUBMODULES_ARE_EVIL': '1'}

# ANSI colors render in the Actions log viewer; NO_COLOR turns them off.
USE_COLOR = 'NO_COLOR' not in os.environ
GRAY, RED, YELLOW, GREEN, RESET = '\033[90m', '\033[91m', '\033[93m', '\033[92m', '\033[0m'


def colorize(text, color):
    return f'{color}{text}{RESET}' if USE_COLOR else text


def highlight(line):
    """Color compiler and build-system errors red and warnings yellow."""
    lower = line.lower()
    if 'error:' in lower or 'failed:' in lower or ('make' in lower and 'error ' in lower):
        return colorize(line, RED)
    if 'warning:' in lower:
        return colorize(line, YELLOW)
    return line


def build_dir_name(target):
    """Directory under build/ that `make target` writes to."""
    if target in METADATA_TARGETS:
        return 'px4_sitl_default'
    if target.endswith('_deb'):
        # Makefile deb rule builds <board>_deb into build/<board>_default
        return target[:-len('_deb')] + '_default'
    return target


def assign_slots(targets, slots):
    """Split targets into at most `slots` contiguous, count-balanced lists.

    Targets that write to the same build directory always share a slot,
    otherwise two slots would produce the same build/ directory and one
    would overwrite the other. Contiguous splitting keeps variants of one
    board (alphabetically adjacent) together, and the assignment is
    deterministic, so a target lands in the same slot path on every run.
    """
    by_dir = {}
    for target in targets:
        by_dir.setdefault(build_dir_name(target), []).append(target)
    units = list(by_dir.values())

    slots = max(1, min(slots, len(units)))
    assigned = [[] for _ in range(slots)]
    position = 0
    for unit in units:
        assigned[position * slots // len(targets)].extend(unit)
        position += len(unit)
    return [a for a in assigned if a]


def check_plan(plan):
    """Refuse a plan where two slots would write the same build/ directory.

    collect_build_dirs() would otherwise replace one slot's output with the
    other's without any error.
    """
    owner = {}
    for slot, targets in enumerate(plan):
        for name in {build_dir_name(t) for t in targets}:
            if owner.setdefault(name, slot) != slot:
                sys.exit(f'refusing to build: build/{name} would be written by '
                         f'slot {owner[name]} and slot {slot}')


def git(*args, cwd):
    return subprocess.run(['git', *args], cwd=cwd, check=True,
                          capture_output=True, text=True).stdout.strip()


def fetch_submodules(root):
    """Fetch every submodule, recursively, into the checkout."""
    start = time.monotonic()
    git('submodule', 'update', '--init', '--recursive', '--jobs', '8', cwd=root)
    say(f'fetched all submodules in {time.monotonic() - start:.0f}s')


def prepare_slot(root, index):
    """Return the source directory for slot `index`, creating it if needed.

    A slot is a worktree of the checkout's commit whose submodules are
    cloned from the checkout's own (url.<path>.insteadOf), so no network is
    used. An existing slot (from an earlier local run) is moved to the
    checkout's current commit instead of being recreated.
    """
    if index == 0:
        return root
    path = root / SLOTS_DIR / f's{index}'
    head = git('rev-parse', 'HEAD', cwd=root)
    if path.exists():
        git('checkout', '--quiet', '--detach', head, cwd=path)
    else:
        git('worktree', 'add', '--quiet', '--detach', str(path), head, cwd=root)
    local = []
    for line in git('submodule', 'status', '--recursive', cwd=root).splitlines():
        sub = line.split()[1]
        url = git('remote', 'get-url', 'origin', cwd=root / sub)
        local += ['-c', f'url.{root / sub}.insteadOf={url}']
    # recorded submodule commits are often detached, not on a branch
    git('-c', 'protocol.file.allow=always', '-c', 'uploadpack.allowAnySHA1InWant=true', *local,
        'submodule', 'update', '--init', '--recursive', '--jobs', '8', cwd=path)
    return path


def say(*lines):
    with print_lock:
        for line in lines:
            print(line)
        sys.stdout.flush()


def memory_report(lines):
    """The linker's 'Memory region' table from a build log, or [] if absent."""
    for i, line in enumerate(lines):
        if line.lstrip().startswith('Memory region'):
            rows = [line]
            for row in lines[i + 1:]:
                if not MEMORY_REGION.match(row):
                    break
                rows.append(row)
            return rows
    return []


def flash_usage(report):
    """Highest '%age Used' among the FLASH regions of a memory report."""
    used = [float(m.group(2)) for m in map(MEMORY_REGION.match, report)
            if m and 'FLASH' in m.group(1).upper()]
    return max(used) if used else None


def is_error_line(line):
    plain = ANSI_ESCAPE.sub('', line)
    return 'error:' in plain.lower() or plain.startswith('FAILED:')


def failure_excerpt(lines):
    """The part of a failed build log that explains the failure.

    Starts at the first compiler/linker error or ninja FAILED line (ninja
    keeps printing other jobs after it, so a plain tail can miss it), skips
    the progress lines of those other jobs, and always ends with the last
    lines of the log, where make reports the exit.
    """
    start = next((i for i, line in enumerate(lines) if is_error_line(line)), None)
    if start is None:
        return lines[-FAILURE_EXCERPT_LINES:]
    diagnostics = [i for i in range(start, len(lines))
                   if not NINJA_PROGRESS.match(ANSI_ESCAPE.sub('', lines[i]))]
    shown = diagnostics[:FAILURE_EXCERPT_LINES]
    tail = [i for i in range(len(lines) - FAILURE_TAIL_LINES, len(lines)) if i > shown[-1]]
    return [lines[i] for i in shown] + (['...'] if tail else []) + [lines[i] for i in tail]


def annotation(text):
    """Escape text for a workflow command message."""
    return text.replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')


def result_line(result):
    status = '✅' if result['returncode'] == 0 else f'❌ exit {result["returncode"]}'
    flash = f', flash {result["flash"]:.1f}%' if result.get('flash') is not None else ''
    return f'{status}  {result["target"]}  (slot {result["slot"]}, {result["seconds"]:.0f}s{flash})'


def build(source, target, slot, logs, width):
    """Build one target, streaming its output with a slot/target/time prefix."""
    start = time.monotonic()

    def prefix():
        tag = f'[{time.monotonic() - start:7.1f}|s{slot} {target.ljust(width)}]'
        return colorize(tag, GRAY)

    say(f'{prefix()} ▶ make {target}')
    log = logs / f'{target}.log'
    with open(log, 'w') as out:
        process = subprocess.Popen(['make', target], cwd=source, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, errors='replace',
                                   env=BUILD_ENV)
        assert process.stdout is not None
        for line in process.stdout:
            out.write(line)
            say(f'{prefix()} {highlight(line.rstrip())}')
        returncode = process.wait()

    lines = log.read_text(errors='replace').splitlines()
    report = memory_report(lines)
    result = {'target': target, 'slot': slot, 'returncode': returncode,
              'seconds': time.monotonic() - start, 'flash': flash_usage(report)}
    # each block is printed under one lock so the other slot cannot interleave
    if returncode == 0:
        say(f'{prefix()} {colorize(result_line(result), GREEN)}',
            *(f'    {row}' for row in report))
    else:
        excerpt = failure_excerpt(lines)
        plain = [ANSI_ESCAPE.sub('', l).strip() for l in excerpt]
        # prefer the compiler/linker message over ninja's FAILED: <object> line
        first_error = next((l for l in plain if 'error:' in l.lower()),
                           next((l for l in plain if l.startswith('FAILED:')),
                                f'make exited with {returncode}'))
        say(f'{prefix()} {colorize(result_line(result), RED)}',
            f'--- {target}: failure ---', *(highlight(l) for l in excerpt),
            f'--- end of {target} failure (full log in the Build Logs step) ---',
            f'::error title={target} failed::{annotation(first_error)}')
    return result


def build_slot(source, targets, slot, logs, width):
    return [build(source, target, slot, logs, width) for target in targets]


def collect_build_dirs(root, source, targets):
    """Move a slot's build directories into the checkout's build/."""
    for name in {build_dir_name(t) for t in targets}:
        src = source / 'build' / name
        if not src.exists():
            continue
        dst = root / 'build' / name
        if dst.exists():
            shutil.rmtree(dst)
        dst.parent.mkdir(exist_ok=True)
        shutil.move(src, dst)


def write_summary(results):
    """Results table for the terminal and, in Actions, the job summary page."""
    def flash(r):
        return f'{r["flash"]:.1f}%' if r.get('flash') is not None else '-'

    say('', f'{"target":<50} {"slot":>4} {"time":>8} {"flash":>7}  result')
    for r in results:
        say(f'{r["target"]:<50} {r["slot"]:>4} {r["seconds"]:7.0f}s {flash(r):>7}  '
            f'{"ok" if r["returncode"] == 0 else "FAILED"}')
    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        with open(summary, 'a') as f:
            f.write('| target | slot | time | flash | result |\n|---|---|---|---|---|\n')
            for r in results:
                f.write(f'| `{r["target"]}` | {r["slot"]} | {r["seconds"]:.0f}s | {flash(r)} | '
                        f'{"✅" if r["returncode"] == 0 else "❌"} |\n')


def print_logs(root):
    """Print each target's full build log as one collapsed group."""
    results_file = root / RESULTS_FILE
    if not results_file.exists():
        # build step died before recording results: print whatever logs exist
        for log in sorted((root / LOGS_DIR).glob('*.log')):
            print(f'::group::{log.stem} (no result recorded)')
            print(log.read_text(errors='replace'), end='')
            print('::endgroup::')
        return 0
    for result in json.loads(results_file.read_text()):
        log = root / LOGS_DIR / f'{result["target"]}.log'
        print(f'::group::{result_line(result)}')
        print(log.read_text(errors='replace') if log.exists() else '(no log)', end='')
        print('::endgroup::')
    return 0


def main():
    parser = argparse.ArgumentParser(
        description='Build a group of PX4 targets, optionally in concurrent slots.')
    parser.add_argument('--slots', type=int, default=1,
                        help='number of concurrent builds (default: 1)')
    parser.add_argument('--print-logs', action='store_true',
                        help='print the build logs of the last run and exit')
    parser.add_argument('targets', nargs='*', help='make targets to build')
    args = parser.parse_args()

    root = Path(git('rev-parse', '--show-toplevel', cwd=None))
    if args.print_logs:
        return print_logs(root)
    if not args.targets:
        parser.error('no targets given')

    plan = assign_slots(args.targets, args.slots)
    check_plan(plan)
    for slot, targets in enumerate(plan):
        say(f'slot {slot}: {" ".join(targets)}')
    say('')

    logs = root / LOGS_DIR
    logs.mkdir(parents=True, exist_ok=True)
    fetch_submodules(root)
    sources = [prepare_slot(root, slot) for slot in range(len(plan))]

    width = max(len(t) for t in args.targets)
    with ThreadPoolExecutor(max_workers=len(plan)) as pool:
        futures = [pool.submit(build_slot, sources[slot], targets, slot, logs, width)
                   for slot, targets in enumerate(plan)]
        results = [r for f in futures for r in f.result()]
    (root / RESULTS_FILE).write_text(json.dumps(results))

    for slot, targets in enumerate(plan[1:], start=1):
        collect_build_dirs(root, sources[slot], targets)

    write_summary(results)
    failed = [r['target'] for r in results if r['returncode'] != 0]
    if failed:
        say('', f'{len(failed)} of {len(results)} targets failed: {" ".join(failed)}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
