"""The agent reads untrusted PR content while holding Bedrock credentials.
These tests pin the sandboxing flags, the environment it gets, and that
any failure is an error rather than an empty (clean-looking) review."""

import json
import subprocess
import unittest
from pathlib import Path
from typing import Any, Dict, List, Mapping

from ai_review import agent

CFG = agent.AgentConfig(model='global.anthropic.claude-opus-5-5',
                        effort='xhigh', max_turns=10, timeout_s=60)


class FakeRunner:

    def __init__(self, stdout: str, returncode: int = 0) -> None:
        self.stdout = stdout
        self.returncode = returncode
        self.calls: List[Dict[str, Any]] = []

    def __call__(self, cmd: List[str], stdin: str, cwd: Path,
                 env: Mapping[str, str],
                 timeout: float) -> 'subprocess.CompletedProcess[str]':
        self.calls.append({'cmd': cmd, 'stdin': stdin, 'env': dict(env)})
        return subprocess.CompletedProcess(cmd, self.returncode, self.stdout,
                                           'boom')


class TestCommand(unittest.TestCase):

    def setUp(self) -> None:
        self.cmd = agent.command(CFG, Path('/p.md'), {'type': 'object'},
                                 'task')

    def test_sandbox_flags(self) -> None:
        for flag in ('--bare', '--restricted', '--strict-mcp-config'):
            self.assertIn(flag, self.cmd)
        i = self.cmd.index('--permission-mode')
        self.assertEqual(self.cmd[i + 1], 'dontAsk')

    def test_only_read_only_tools(self) -> None:
        i = self.cmd.index('--tools')
        self.assertEqual(self.cmd[i + 1], 'Read,Bash')
        j = self.cmd.index('--allowedTools')
        allowed = self.cmd[j + 1:self.cmd.index('--effort')]
        self.assertEqual(allowed, list(agent.ALLOWED_TOOLS))
        for entry in allowed:
            # nothing that writes, runs code or reaches the network
            self.assertRegex(entry, r'^(Read|Bash\((grep|ls|git (log|blame|'
                                    r'show)) \*\))$')


class TestRun(unittest.TestCase):

    def run_agent(self, runner: FakeRunner) -> agent.AgentResult:
        return agent.run(CFG, Path('/p.md'), {}, 'task', 'big context',
                         Path('.'), runner=runner)

    def test_parses_structured_output_and_cost(self) -> None:
        runner = FakeRunner(json.dumps({
            'structured_output': {'findings': []}, 'total_cost_usd': 1.5,
            'usage': {'input_tokens': 10}}))
        result = self.run_agent(runner)
        self.assertEqual(result.output, {'findings': []})
        self.assertEqual(result.cost_usd, 1.5)
        self.assertEqual(runner.calls[0]['stdin'], 'big context')

    def test_environment_selects_bedrock_and_drops_github_token(
            self) -> None:
        env = agent.environment(CFG, {'GITHUB_TOKEN': 'x', 'GH_TOKEN': 'y',
                                      'PATH': '/bin'})
        self.assertEqual(env['CLAUDE_CODE_USE_BEDROCK'], '1')
        self.assertEqual(env['ANTHROPIC_MODEL'], CFG.model)
        self.assertEqual(env['ANTHROPIC_DEFAULT_HAIKU_MODEL'], CFG.model)
        self.assertNotIn('GITHUB_TOKEN', env)
        self.assertNotIn('GH_TOKEN', env)

    def test_failures_raise(self) -> None:
        cases = [FakeRunner('{}', returncode=1),
                 FakeRunner('not json'),
                 FakeRunner(json.dumps({'is_error': True})),
                 FakeRunner(json.dumps({'result': 'text only'}))]
        for runner in cases:
            with self.subTest(stdout=runner.stdout):
                with self.assertRaises(agent.AgentError):
                    self.run_agent(runner)


if __name__ == '__main__':
    unittest.main()
