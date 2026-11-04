"""Run model-written Python in a locked-down container.

The reviewer may write code to check its own math (re-deriving a mixer
matrix, sweeping a filter's response). That code is shaped by text anyone
can put in a PR, so it runs in the px4-dev image with:

- no network (--network none) and no host environment variables;
- the PR checkout mounted read-only, a small tmpfs for scratch;
- a non-root user, all capabilities dropped, no privilege escalation;
- CPU, memory, process and wall-clock limits;
- in CI, the gVisor runtime (runsc), so an escape has to break gVisor's
  user-space kernel as well as the host kernel.

The agent can only reach this through the `px4-sandbox-python` command,
which accepts nothing but the code to run. Everything else comes from
environment variables the pipeline sets.
"""

import json
import os
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from typing import List, Mapping, Optional, Sequence

MAX_CODE_BYTES = 64_000
MAX_OUTPUT_CHARS = 20_000
TIMEOUT_S = 90

ENV_IMAGE = 'AI_REVIEW_SANDBOX_IMAGE'
ENV_CHECKOUT = 'AI_REVIEW_SANDBOX_CHECKOUT'
ENV_RUNTIME = 'AI_REVIEW_SANDBOX_RUNTIME'
# optional: where to append one JSON line per call (set by the pipeline)
ENV_LOG = 'AI_REVIEW_SANDBOX_LOG'

USAGE = ('usage: px4-sandbox-python -c CODE\n'
         'Runs Python 3 (numpy, sympy, pyulog) with no network, the PR '
         'checkout read-only at /work, and a 90 s limit.')


class SandboxError(RuntimeError):
    pass


@dataclass(frozen=True)
class SandboxConfig:
    image: str
    checkout: str
    runtime: Optional[str]

    @staticmethod
    def from_env(env: Mapping[str, str]) -> 'SandboxConfig':
        image = env.get(ENV_IMAGE, '')
        checkout = env.get(ENV_CHECKOUT, '')
        if not image or not checkout:
            raise SandboxError('sandbox is not configured')
        return SandboxConfig(image=image, checkout=checkout,
                             runtime=env.get(ENV_RUNTIME) or None)


def parse_args(argv: Sequence[str]) -> str:
    """Accept exactly `-c CODE`; anything else is refused."""
    if len(argv) != 2 or argv[0] != '-c':
        raise SandboxError(USAGE)
    code = argv[1]
    if len(code.encode('utf-8')) > MAX_CODE_BYTES:
        raise SandboxError(f'code longer than {MAX_CODE_BYTES} bytes')
    return code


def docker_command(cfg: SandboxConfig, code: str, name: str) -> List[str]:
    cmd = ['docker', 'run', '--rm', '--name', name,
           '--network', 'none',
           '--read-only',
           '--tmpfs', '/tmp:rw,nosuid,nodev,size=64m,mode=1777',
           '--user', '65534:65534',
           '--cap-drop', 'ALL',
           '--security-opt', 'no-new-privileges',
           '--pids-limit', '128',
           '--memory', '1g', '--memory-swap', '1g',
           '--cpus', '1',
           '--env', 'HOME=/tmp', '--env', 'MPLCONFIGDIR=/tmp',
           '--volume', f'{cfg.checkout}:/work:ro',
           '--workdir', '/work',
           '--entrypoint', 'python3']
    if cfg.runtime:
        cmd += ['--runtime', cfg.runtime]
    # -I: isolated mode, ignore PYTHON* variables and the user site dir
    return cmd + [cfg.image, '-I', '-c', code]


def run(cfg: SandboxConfig, code: str) -> 'subprocess.CompletedProcess[str]':
    name = f'ai-review-sandbox-{uuid.uuid4().hex[:12]}'
    cmd = docker_command(cfg, code, name)
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=TIMEOUT_S, check=False,
                              env={'PATH': os.environ.get('PATH', '')})
    except subprocess.TimeoutExpired as e:
        subprocess.run(['docker', 'kill', name], capture_output=True,
                       check=False)
        raise SandboxError(f'timed out after {TIMEOUT_S} s') from e


def _truncate(text: str) -> str:
    if len(text) <= MAX_OUTPUT_CHARS:
        return text
    return text[:MAX_OUTPUT_CHARS] + '\n[output truncated]\n'


def log_call(path: str, code: str, returncode: int, seconds: float) -> None:
    """Append one JSON line per call, so the pilot can see how often and
    how the reviewer uses code execution. Never fails the call."""
    try:
        with open(path, 'a', encoding='utf-8') as fh:
            fh.write(json.dumps({'returncode': returncode,
                                 'seconds': round(seconds, 2),
                                 'code_bytes': len(code.encode('utf-8')),
                                 'code': code[:4000]}) + '\n')
    except OSError:
        pass


def main(argv: Optional[Sequence[str]] = None,
         env: Optional[Mapping[str, str]] = None) -> int:
    environ = os.environ if env is None else env
    started = time.monotonic()
    code = ''
    try:
        cfg = SandboxConfig.from_env(environ)
        code = parse_args(sys.argv[1:] if argv is None else argv)
        proc = run(cfg, code)
    except SandboxError as e:
        print(f'px4-sandbox-python: {e}', file=sys.stderr)
        if environ.get(ENV_LOG):
            log_call(environ[ENV_LOG], code, 2, time.monotonic() - started)
        return 2
    sys.stdout.write(_truncate(proc.stdout))
    sys.stderr.write(_truncate(proc.stderr))
    if environ.get(ENV_LOG):
        log_call(environ[ENV_LOG], code, proc.returncode,
                 time.monotonic() - started)
    return proc.returncode
