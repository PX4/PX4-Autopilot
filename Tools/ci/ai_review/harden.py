"""Prepare a RunsOn runner before the review agent starts.

The runner's EC2 instance role can write the CI cache bucket, which a
container escape from the sandbox could abuse. Two layers keep that out of
reach for the rest of the job:

1. gVisor (runsc) becomes the sandbox's container runtime, pinned and
   checksum-verified, so an escape has to break gVisor as well as the host
   kernel.
2. The EC2 metadata service is blocked for the host and for containers,
   so the instance role's credentials cannot be fetched. The job's Bedrock
   access comes from GitHub OIDC credentials already in the environment,
   which do not need the metadata service.

Finally a self-test proves the sandbox runs under runsc with no network
and no metadata service. Any failure fails the job: the review does not
run without its sandbox.
"""

import hashlib
import subprocess
import tarfile
import tempfile
import urllib.request
from pathlib import Path
from typing import Callable, List, Sequence

from .sandbox import SandboxConfig, docker_command

GVISOR_VERSION = '20260928'
GVISOR_URL = ('https://storage.googleapis.com/gvisor/releases/release/'
              f'{GVISOR_VERSION}/x86_64/gvisor.tar.bz2')
GVISOR_SHA512 = (
    'c8d3a9fd4d4c4f5b8ff213caa4517356be128d18659ec4cde37828fe797f61a9'
    '725a602a846c81a8ed19c057a996515d31c081eba343ed4613a89951ba32ed59')
GVISOR_BINARIES = ('runsc', 'containerd-shim-runsc-v1')
INSTALL_DIR = '/usr/local/bin'

IMDS_V4 = '169.254.169.254'
IMDS_V6 = 'fd00:ec2::254'

Runner = Callable[[Sequence[str]], 'subprocess.CompletedProcess[str]']


class HardenError(RuntimeError):
    pass


def _run(cmd: Sequence[str]) -> 'subprocess.CompletedProcess[str]':
    return subprocess.run(list(cmd), capture_output=True, text=True,
                          check=False, timeout=600)


def _must(runner: Runner, cmd: Sequence[str]) -> str:
    proc = runner(cmd)
    if proc.returncode != 0:
        raise HardenError(f'{" ".join(cmd)} failed: '
                          f'{(proc.stderr or proc.stdout).strip()[-800:]}')
    return proc.stdout


def verify_sha512(path: Path, expected: str) -> None:
    digest = hashlib.sha512(path.read_bytes()).hexdigest()
    if digest != expected:
        raise HardenError(f'gVisor checksum mismatch: {digest}')


def install_gvisor(runner: Runner = _run) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / 'gvisor.tar.bz2'
        urllib.request.urlretrieve(GVISOR_URL, archive)
        verify_sha512(archive, GVISOR_SHA512)
        with tarfile.open(archive, 'r:bz2') as tar:
            for name in GVISOR_BINARIES:
                member = tar.getmember(name)
                data = tar.extractfile(member) if member.isfile() else None
                if data is None:
                    raise HardenError(f'{name} is not a file in the archive')
                # write by our own name; never trust archive paths
                (Path(tmp) / name).write_bytes(data.read())
        for name in GVISOR_BINARIES:
            _must(runner, ['sudo', 'install', '-m', '0755',
                           str(Path(tmp) / name), f'{INSTALL_DIR}/{name}'])
    _must(runner, ['sudo', f'{INSTALL_DIR}/runsc', 'install'])
    _must(runner, ['sudo', 'systemctl', 'restart', 'docker'])


def imds_block_commands() -> List[List[str]]:
    """iptables rules that drop metadata-service traffic.

    OUTPUT covers processes on the host; DOCKER-USER covers containers on
    Docker networks (the sandbox itself uses --network none).
    """
    cmds = []
    for chain in ('OUTPUT', 'DOCKER-USER'):
        cmds.append(['sudo', 'iptables', '-I', chain, '1', '-d', IMDS_V4,
                     '-j', 'REJECT'])
        cmds.append(['sudo', 'ip6tables', '-I', chain, '1', '-d', IMDS_V6,
                     '-j', 'REJECT'])
    return cmds


def block_imds(runner: Runner = _run) -> None:
    for cmd in imds_block_commands():
        proc = runner(cmd)
        # ip6tables or the DOCKER-USER chain may be absent; IPv4 OUTPUT,
        # the path the instance role is normally fetched over, must work
        if proc.returncode != 0 and cmd[2:4] == ['-I', 'OUTPUT'] \
                and cmd[1] == 'iptables':
            raise HardenError(f'blocking the metadata service failed: '
                              f'{proc.stderr.strip()}')
    probe = runner(['curl', '-s', '-m', '3', '-o', '/dev/null', '-w',
                    '%{http_code}', '-X', 'PUT',
                    f'http://{IMDS_V4}/latest/api/token',
                    '-H', 'X-aws-ec2-metadata-token-ttl-seconds: 60'])
    if probe.stdout.strip() not in ('', '000'):
        raise HardenError(f'metadata service still reachable '
                          f'(HTTP {probe.stdout.strip()})')


SELF_TEST = (
    'import socket, urllib.request\n'
    'try:\n'
    '    urllib.request.urlopen("http://169.254.169.254/", timeout=3)\n'
    '    print("FAIL: metadata reachable")\n'
    'except OSError:\n'
    '    print("ok: no network")\n')


def self_test(cfg: SandboxConfig, runner: Runner = _run) -> None:
    if cfg.runtime:
        info = _must(runner, ['docker', 'info', '--format',
                              '{{json .Runtimes}}'])
        if cfg.runtime not in info:
            raise HardenError(f'docker runtime {cfg.runtime} not registered')
    out = _must(runner, docker_command(cfg, SELF_TEST, 'ai-review-selftest'))
    if 'ok: no network' not in out:
        raise HardenError(f'sandbox self-test failed: {out.strip()}')


def harden(cfg: SandboxConfig, runner: Runner = _run) -> None:
    if cfg.runtime == 'runsc':
        install_gvisor(runner)
    _must(runner, ['docker', 'pull', '--quiet', cfg.image])
    block_imds(runner)
    self_test(cfg, runner)
