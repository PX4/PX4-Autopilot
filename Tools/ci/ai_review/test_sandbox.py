"""The sandbox is the only way model-written code runs. These tests pin
its isolation flags, that the agent can pass nothing but code, and that
the CI hardening fails closed."""

import subprocess
import tempfile
import unittest
from pathlib import Path
from typing import List, Sequence

from ai_review import agent, harden, sandbox

CFG = sandbox.SandboxConfig(image='ghcr.io/px4/px4-dev:tag',
                            checkout='/w/pr', runtime='runsc')


class TestDockerCommand(unittest.TestCase):

    def setUp(self) -> None:
        self.cmd = sandbox.docker_command(CFG, 'print(1)', 'n')

    def flag(self, name: str) -> str:
        return self.cmd[self.cmd.index(name) + 1]

    def test_isolation_flags(self) -> None:
        self.assertEqual(self.flag('--network'), 'none')
        self.assertEqual(self.flag('--user'), '65534:65534')
        self.assertEqual(self.flag('--cap-drop'), 'ALL')
        self.assertEqual(self.flag('--security-opt'), 'no-new-privileges')
        self.assertEqual(self.flag('--runtime'), 'runsc')
        self.assertIn('--read-only', self.cmd)
        self.assertIn('/w/pr:/work:ro', self.cmd)
        for limit in ('--pids-limit', '--memory', '--cpus'):
            self.assertIn(limit, self.cmd)

    def test_no_host_environment_is_forwarded(self) -> None:
        envs = [self.cmd[i + 1] for i, a in enumerate(self.cmd)
                if a == '--env']
        self.assertEqual(sorted(envs), ['HOME=/tmp', 'MPLCONFIGDIR=/tmp'])
        self.assertNotIn('--env-file', self.cmd)

    def test_code_is_the_last_argument_after_the_image(self) -> None:
        i = self.cmd.index(CFG.image)
        self.assertEqual(self.cmd[i + 1:], ['-I', '-c', 'print(1)'])


class TestArgs(unittest.TestCase):

    def test_only_dash_c_code(self) -> None:
        self.assertEqual(sandbox.parse_args(['-c', 'x=1']), 'x=1')
        for argv in ([], ['x.py'], ['-c'], ['-c', 'a', 'b'],
                     ['--image', 'evil', '-c', 'x'], ['-m', 'http.server']):
            with self.subTest(argv=argv):
                with self.assertRaises(sandbox.SandboxError):
                    sandbox.parse_args(argv)

    def test_rejects_huge_code(self) -> None:
        with self.assertRaises(sandbox.SandboxError):
            sandbox.parse_args(['-c', 'x' * (sandbox.MAX_CODE_BYTES + 1)])

    def test_unconfigured_sandbox_refuses(self) -> None:
        self.assertEqual(sandbox.main(['-c', 'x'], env={}), 2)


class TestCallLog(unittest.TestCase):

    def test_refused_call_is_logged_and_counted(self) -> None:
        from ai_review import pipeline
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / pipeline.SANDBOX_LOG
            env = {sandbox.ENV_LOG: str(log)}  # unconfigured: refused
            self.assertEqual(sandbox.main(['-c', 'print(1)'], env=env), 2)
            sandbox.log_call(str(log), 'x = 1', 0, 1.25)
            stats = pipeline.sandbox_stats(log)
        self.assertEqual(stats, {'sandbox_calls': 2,
                                 'sandbox_failed_calls': 1,
                                 'sandbox_seconds': 1.2})

    def test_no_log_means_zero(self) -> None:
        from ai_review import pipeline
        self.assertEqual(pipeline.sandbox_stats(Path('/nonexistent'))
                         ['sandbox_calls'], 0)


class TestAgentWiring(unittest.TestCase):

    def test_sandbox_tool_only_when_configured(self) -> None:
        bare = agent.AgentConfig('m', 'high', 5, 60)
        cmd = agent.command(bare, Path('/p'), {}, 't')
        self.assertNotIn(agent.SANDBOX_TOOL, cmd)
        armed = agent.AgentConfig('m', 'high', 5, 60, sandbox_env={
            sandbox.ENV_IMAGE: 'img', sandbox.ENV_CHECKOUT: '/w'})
        self.assertIn(agent.SANDBOX_TOOL, agent.command(armed, Path('/p'),
                                                        {}, 't'))
        env = agent.environment(armed, {'PATH': '/usr/bin'})
        self.assertTrue(env['PATH'].startswith(str(agent.SANDBOX_BIN)))
        self.assertEqual(env[sandbox.ENV_IMAGE], 'img')

    def test_entry_point_exists_and_is_executable(self) -> None:
        entry = agent.SANDBOX_BIN / 'px4-sandbox-python'
        self.assertTrue(entry.is_file())
        self.assertTrue(entry.stat().st_mode & 0o111)


class FakeRunner:

    def __init__(self, fail: Sequence[str] = (), stdout: str = '') -> None:
        self.fail = fail
        self.stdout = stdout
        self.calls: List[List[str]] = []

    def __call__(self, cmd: Sequence[str]
                 ) -> 'subprocess.CompletedProcess[str]':
        self.calls.append(list(cmd))
        bad = any(f in ' '.join(cmd) for f in self.fail)
        return subprocess.CompletedProcess(list(cmd), 1 if bad else 0,
                                           self.stdout, 'err' if bad else '')


class TestHarden(unittest.TestCase):

    def test_imds_blocked_for_host_and_containers(self) -> None:
        cmds = [' '.join(c) for c in harden.imds_block_commands()]
        self.assertIn('sudo iptables -I OUTPUT 1 -d 169.254.169.254 '
                      '-j REJECT', cmds)
        self.assertIn('sudo iptables -I DOCKER-USER 1 -d 169.254.169.254 '
                      '-j REJECT', cmds)

    def test_ipv4_output_block_failure_is_fatal(self) -> None:
        with self.assertRaises(harden.HardenError):
            harden.block_imds(FakeRunner(fail=['iptables -I OUTPUT']))

    def test_reachable_metadata_service_is_fatal(self) -> None:
        with self.assertRaises(harden.HardenError):
            harden.block_imds(FakeRunner(stdout='200'))

    def test_self_test_requires_registered_runtime(self) -> None:
        with self.assertRaises(harden.HardenError):
            harden.self_test(CFG, FakeRunner(stdout='{"runc":{}}'))

    def test_self_test_requires_no_network_result(self) -> None:
        runner = FakeRunner(stdout='{"runsc":{}} FAIL: metadata reachable')
        with self.assertRaises(harden.HardenError):
            harden.self_test(CFG, runner)

    def test_checksum_mismatch_is_fatal(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp) / 'x'
            f.write_bytes(b'tampered')
            with self.assertRaises(harden.HardenError):
                harden.verify_sha512(f, harden.GVISOR_SHA512)


if __name__ == '__main__':
    unittest.main()
