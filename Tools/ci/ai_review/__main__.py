"""Command line entry point: python3 -m ai_review {run,render} ...

Run from Tools/ci (the workflow sets working-directory) so the package is
importable.
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional

from . import (agent, findings, harden, models, pipeline, render, sandbox,
               trigger)

# Model aliases, profiles, labels and per-role defaults: models.json
REGISTRY = models.load()
EFFORTS = models.EFFORTS
SECRET_ENV = ('AWS_ACCESS_KEY_ID', 'AWS_SECRET_ACCESS_KEY',
              'AWS_SESSION_TOKEN', 'GITHUB_TOKEN', 'GH_TOKEN')


def _sandbox(args: argparse.Namespace) -> Optional[sandbox.SandboxConfig]:
    if not args.sandbox_image:
        return None
    return sandbox.SandboxConfig(image=args.sandbox_image,
                                 checkout=str(Path(args.checkout).resolve()),
                                 runtime=args.sandbox_runtime or None)


def _sandbox_env(args: argparse.Namespace) -> Dict[str, str]:
    cfg = _sandbox(args)
    if cfg is None:
        return {}
    env = {sandbox.ENV_IMAGE: cfg.image, sandbox.ENV_CHECKOUT: cfg.checkout,
           sandbox.ENV_LOG: str(Path(args.work_dir).resolve()
                                / pipeline.SANDBOX_LOG)}
    if cfg.runtime:
        env[sandbox.ENV_RUNTIME] = cfg.runtime
    return env


def _settings(args: argparse.Namespace) -> pipeline.Settings:
    if args.model == 'default':
        args.model = REGISTRY.defaults['reviewer'].model
    if args.effort == 'default':
        args.effort = REGISTRY.defaults['reviewer'].effort
    reviewer_model = REGISTRY.get(args.model)
    model_id, label = reviewer_model.bedrock_profile, reviewer_model.label
    validator_id = REGISTRY.get(args.validator_model).bedrock_profile
    sandbox_env = _sandbox_env(args)
    return pipeline.Settings(
        repo=args.repo,
        pr_number=args.pr,
        trusted_root=Path(args.trusted_root).resolve(),
        checkout=Path(args.checkout).resolve(),
        work_dir=Path(args.work_dir).resolve(),
        out_dir=Path(args.out_dir).resolve(),
        reviewer=agent.AgentConfig(model=model_id, effort=args.effort,
                                   max_turns=args.max_turns,
                                   timeout_s=args.timeout,
                                   sandbox_env=sandbox_env),
        validator=agent.AgentConfig(model=validator_id,
                                    effort=args.validator_effort,
                                    max_turns=20, timeout_s=600,
                                    sandbox_env=sandbox_env),
        model_label=label,
        report_url=_run_url(),
    )


def _run_url() -> str:
    """This Actions run, whose job summary carries the full report."""
    parts = [os.environ.get(k, '') for k in
             ('GITHUB_SERVER_URL', 'GITHUB_REPOSITORY', 'GITHUB_RUN_ID')]
    if not all(parts):
        return ''
    server, repo, run_id = parts
    return f'{server}/{repo}/actions/runs/{run_id}'


def _guard_leaks(out_dir: Path, work_dir: Path) -> None:
    """Delete the outputs and fail if any credential appears in them."""
    files = [out_dir / 'manifest.json', out_dir / 'comments.json',
             work_dir / 'report.md']
    texts = [p.read_text(encoding='utf-8') for p in files if p.exists()]
    secrets = [os.environ.get(k, '') for k in SECRET_ENV]
    leaks = render.find_leaks(texts, secrets)
    if leaks:
        for p in files:
            p.unlink(missing_ok=True)
        sys.exit('error: review output contains credentials '
                 f'({", ".join(leaks)}); nothing was written')


def _write_step_summary(report: str) -> None:
    path = os.environ.get('GITHUB_STEP_SUMMARY')
    if path:
        with open(path, 'a', encoding='utf-8') as fh:
            fh.write(report)


def _annotation(level: str, title: str, message: str) -> str:
    """A GitHub Actions workflow command, escaped per the runner's rules."""
    def esc(text: str) -> str:
        return (text.replace('%', '%25').replace('\r', '%0D')
                .replace('\n', '%0A'))
    return f'::{level} title={esc(title).replace(",", "%2C")}::{esc(message)}'


def cmd_run(args: argparse.Namespace) -> int:
    s = _settings(args)
    data = pipeline.run(s)
    _guard_leaks(s.out_dir, s.work_dir)
    _write_step_summary(data['report'])
    print(data['report'])
    refusal = data.get('refusal')
    if refusal is not None:
        # not a tooling failure, but nobody may mistake it for a review
        print(_annotation('warning', 'AI review not done',
                          f'{refusal.model} declined ({refusal.category}); '
                          f'request {refusal.request_id or "n/a"}. '
                          f'{refusal.reason}'))
    return 0


def cmd_models_check(args: argparse.Namespace) -> int:
    failures = models.check(REGISTRY, args.region)
    for m in REGISTRY.models.values():
        state = 'FAIL' if any(f.startswith(f'{m.alias} ') for f in failures) \
            else 'ok'
        print(f'{state:4} {m.alias:8} {m.bedrock_profile}  ({m.label})')
    for f in failures:
        print(f'  {f}', file=sys.stderr)
    return 1 if failures else 0


def cmd_harden(args: argparse.Namespace) -> int:
    cfg = _sandbox(args)
    if cfg is None:
        sys.exit('error: --sandbox-image is required')
    try:
        harden.harden(cfg)
    except harden.HardenError as e:
        sys.exit(f'error: {e}')
    print(f'runner hardened: sandbox {cfg.image} '
          f'(runtime {cfg.runtime or "default"}), metadata service blocked')
    return 0


def cmd_render(args: argparse.Namespace) -> int:
    """Re-render saved findings without calling any model."""
    s = _settings(args)
    saved = json.loads(Path(args.findings).read_text(encoding='utf-8'))
    pairs = [(findings.parse_finding(p['finding']),
              findings.parse_verdict(p['verdict'])) for p in saved['pairs']]
    diff_text = Path(args.diff).read_text(encoding='utf-8')
    checklist = (findings.parse_checklist(saved['checklist'])
                 if saved.get('checklist') else None)
    data = pipeline.finish(s, args.pr, args.head_sha, diff_text,
                           saved.get('summary', ''), pairs, {}, checklist)
    _guard_leaks(s.out_dir, s.work_dir)
    print(data['report'])
    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='ai_review', description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)

    def common(sp: argparse.ArgumentParser) -> None:
        sp.add_argument('--repo', default=os.environ.get(
            'GITHUB_REPOSITORY', 'PX4/PX4-Autopilot'))
        sp.add_argument('--pr', type=int, required=True)
        sp.add_argument('--trusted-root', required=True,
                        help='checkout of the base branch (prompts, rules)')
        sp.add_argument('--checkout', required=True,
                        help='checkout of the PR head (data only)')
        sp.add_argument('--work-dir', default='ai-review-work')
        sp.add_argument('--out-dir', default='pr-review')
        aliases = sorted(REGISTRY.models)
        rev = REGISTRY.defaults['reviewer']
        val = REGISTRY.defaults['validator']
        # 'default' lets the workflow defer to models.json explicitly
        sp.add_argument('--model', choices=['default', *aliases],
                        default=rev.model)
        sp.add_argument('--effort', choices=['default', *EFFORTS],
                        default=rev.effort)
        sp.add_argument('--validator-model', choices=aliases,
                        default=val.model)
        sp.add_argument('--validator-effort', choices=EFFORTS,
                        default=val.effort)
        sp.add_argument('--max-turns', type=int, default=80)
        sp.add_argument('--timeout', type=float, default=1500,
                        help='base reviewer timeout; grows with PR size')
        sandbox_args(sp)

    def sandbox_args(sp: argparse.ArgumentParser) -> None:
        sp.add_argument('--sandbox-image', default='',
                        help='container image for model-written code; '
                        'without it the agent cannot run code')
        sp.add_argument('--sandbox-runtime', default='',
                        help='container runtime, e.g. runsc (gVisor)')

    run = sub.add_parser('run', help='review a PR end to end')
    common(run)
    run.set_defaults(func=cmd_run)

    trig = sub.add_parser('trigger', help='decide from the workflow event '
                          'whether to run a review (writes GITHUB_OUTPUT)')
    trig.set_defaults(func=lambda a: trigger.main([]))
    rct = sub.add_parser('react', help='react to the triggering comment')
    rct.add_argument('comment_id')
    rct.add_argument('content', choices=['eyes', 'rocket', 'confused'])
    rct.set_defaults(func=lambda a: trigger.main(
        ['react', a.comment_id, a.content]))
    sts = sub.add_parser('status', help='set the final commit status on the '
                         'PR head from the review job result')
    sts.add_argument('sha')
    sts.add_argument('result')
    sts.set_defaults(func=lambda a: trigger.main(
        ['status', a.sha, a.result]))

    chk = sub.add_parser('models-check', help='send a one-token request to '
                         'every model in models.json with the current '
                         'credentials; run before changing models')
    chk.add_argument('--region', default=os.environ.get('AWS_REGION',
                                                        'us-west-2'))
    chk.set_defaults(func=cmd_models_check)

    hard = sub.add_parser('harden', help='prepare a CI runner: install '
                          'gVisor, block the metadata service, self-test')
    hard.add_argument('--checkout', required=True)
    sandbox_args(hard)
    hard.set_defaults(func=cmd_harden)

    rend = sub.add_parser('render', help='re-render saved findings offline')
    common(rend)
    rend.add_argument('--findings', required=True)
    rend.add_argument('--diff', required=True)
    rend.add_argument('--head-sha', required=True)
    rend.set_defaults(func=cmd_render)
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = parser().parse_args(argv)
    result: int = args.func(args)
    return result


if __name__ == '__main__':
    sys.exit(main())
