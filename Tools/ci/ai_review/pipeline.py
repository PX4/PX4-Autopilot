"""Review pipeline: gather, review, validate each finding, route, render.

Trusted inputs (prompts, review criteria, repository instructions) are
read from the base checkout. The PR checkout is only ever data the agent
reads.
"""

import dataclasses
import fnmatch
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from . import agent, context, diff, findings, render, route

PROMPTS = Path(__file__).resolve().parent / 'prompts'
# one JSON line per px4-sandbox-python call, written by sandbox.py
SANDBOX_LOG = 'sandbox-calls.jsonl'
CRITERIA = Path('.agents/skills/review-pr/review-criteria.md')
INSTRUCTIONS = Path('.github/instructions')
APPLY_TO_RE = re.compile(r'^applyTo:\s*"([^"]*)"', re.MULTILINE)


@dataclass(frozen=True)
class Settings:
    repo: str
    pr_number: int
    trusted_root: Path
    checkout: Path
    work_dir: Path
    out_dir: Path
    reviewer: agent.AgentConfig
    validator: agent.AgentConfig
    model_label: str


def matching_instructions(trusted_root: Path, files: List[str]) -> List[Path]:
    """Instruction files whose applyTo globs match any changed file."""
    matched = []
    for path in sorted((trusted_root / INSTRUCTIONS).glob(
            '*.instructions.md')):
        m = APPLY_TO_RE.search(path.read_text(encoding='utf-8'))
        if not m:
            continue
        globs = [g.strip() for g in m.group(1).split(',') if g.strip()]
        if any(fnmatch.fnmatch(f, g) for f in files for g in globs):
            matched.append(path)
    return matched


def _sandbox_note(sandbox: bool) -> List[str]:
    # only mention code execution when the agent actually has it
    if not sandbox:
        return []
    return [(PROMPTS / 'sandbox.md').read_text(encoding='utf-8')]


def system_prompt(trusted_root: Path, files: List[str],
                  sandbox: bool = False) -> str:
    parts = [(PROMPTS / 'reviewer.md').read_text(encoding='utf-8'),
             *_sandbox_note(sandbox),
             (trusted_root / CRITERIA).read_text(encoding='utf-8'),
             # process checks are CI-only; the interactive skill stays lean
             (PROMPTS / 'contribution.md').read_text(encoding='utf-8'),
             (trusted_root / 'AGENTS.md').read_text(encoding='utf-8')]
    for path in matching_instructions(trusted_root, files):
        rel = path.relative_to(trusted_root)
        parts.append(f'## Repository instructions: {rel}\n\n'
                     + path.read_text(encoding='utf-8'))
    return '\n\n---\n\n'.join(parts)


def validator_prompt(sandbox: bool = False) -> str:
    parts = [(PROMPTS / 'validator.md').read_text(encoding='utf-8'),
             *_sandbox_note(sandbox)]
    return '\n\n---\n\n'.join(parts)


def _validate_all(s: Settings, pr: context.PrContext,
                  review: findings.Review,
                  usage: Dict[str, Any]) -> List[Tuple[findings.Finding,
                                                       findings.Verdict]]:
    prompt_file = s.work_dir / 'validator-prompt.md'
    prompt_file.write_text(validator_prompt(bool(s.validator.sandbox_env)),
                           encoding='utf-8')
    pairs = []
    for f in review.findings:
        try:
            result = agent.run(
                s.validator, prompt_file, findings.VERDICT_SCHEMA,
                agent.VALIDATE_TASK,
                context.validator_input(f.to_json(), pr),
                s.checkout)
            verdict = findings.parse_verdict(result.output)
            _add_usage(usage, result)
        except agent.AgentRefusal as e:
            # an unchecked finding never reaches the PR inline; say why
            verdict = findings.Verdict(
                keep=True, confidence='low',
                reason=f'not validated: the validator model declined '
                       f'({e.category}): {e.reason}')
            usage['validator_refusals'] = \
                usage.get('validator_refusals', 0) + 1
        except (agent.AgentError, findings.ContractError) as e:
            # an unchecked finding never reaches the PR inline
            verdict = findings.Verdict(keep=True, confidence='low',
                                       reason=f'validation failed: {e}')
        pairs.append((f, verdict))
    return pairs


# (changed lines up to, extra turns, extra seconds) on top of the base
# budget: bigger PRs need more reading, and the turn limit stays the cap
SIZE_STEPS = ((300, 0, 0), (1500, 40, 900), (6000, 80, 1800),
              (None, 120, 2700))


def scaled_for_size(cfg: agent.AgentConfig,
                    changed_lines: int) -> agent.AgentConfig:
    for limit, turns, seconds in SIZE_STEPS:
        if limit is None or changed_lines <= limit:
            return dataclasses.replace(cfg, max_turns=cfg.max_turns + turns,
                                       timeout_s=cfg.timeout_s + seconds)
    return cfg


def _add_usage(usage: Dict[str, Any], result: agent.AgentResult) -> None:
    usage['calls'] = usage.get('calls', 0) + 1
    usage['cost_usd'] = usage.get('cost_usd', 0.0) + result.cost_usd
    for key, value in result.usage.items():
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            usage[key] = usage.get(key, 0) + value
    # per-model split, so mixed runs show what each model cost
    by_model = usage.setdefault('by_model', {})
    for model, mu in result.model_usage.items():
        if not isinstance(mu, dict):
            continue
        entry = by_model.setdefault(model, {})
        for key in ('costUSD', 'outputTokens', 'cacheReadInputTokens',
                    'cacheCreationInputTokens', 'inputTokens'):
            value = mu.get(key)
            if isinstance(value, (int, float)):
                entry[key] = entry.get(key, 0) + value


def run(s: Settings) -> Dict[str, Any]:
    """Run the whole pipeline. Returns the report data it also writes."""
    s.work_dir.mkdir(parents=True, exist_ok=True)
    pr = context.gather(s.repo, s.pr_number)
    usage: Dict[str, Any] = {'changed_lines': pr.changed_lines}

    # Every PR is reviewed regardless of size; only one made entirely of
    # generated files (lock files, translations) has nothing to review.
    if not pr.reviewed:
        review = findings.Review(
            summary='Not reviewed: the PR only changes generated files.',
            findings=[])
        pairs: List[Tuple[findings.Finding, findings.Verdict]] = []
    else:
        prompt_file = s.work_dir / 'system-prompt.md'
        prompt_file.write_text(
            system_prompt(s.trusted_root, pr.files,
                          sandbox=bool(s.reviewer.sandbox_env)),
            encoding='utf-8')
        reviewer = scaled_for_size(s.reviewer, pr.changed_lines)
        usage['max_turns'] = reviewer.max_turns
        try:
            result = agent.run(reviewer, prompt_file, findings.REVIEW_SCHEMA,
                               agent.REVIEW_TASK, context.reviewer_input(pr),
                               s.checkout)
        except agent.AgentRefusal as e:
            return refused(s, pr, e, usage)
        _add_usage(usage, result)
        # keep the raw output so a parse failure never loses a paid review
        (s.work_dir / 'reviewer-output.json').write_text(
            json.dumps(result.output, indent=2))
        review = findings.parse_review(result.output)
        pairs = _validate_all(s, pr, review, usage)

    (s.work_dir / 'findings.json').write_text(json.dumps({
        'summary': review.summary,
        'checklist': {k: e.__dict__ for k, e in review.checklist.items()},
        'pairs': [{'finding': f.to_json(), 'verdict': v.to_json()}
                  for f, v in pairs],
        'rejected': [{'item': item, 'reason': why}
                     for item, why in review.rejected],
    }, indent=2))
    usage['rejected_findings'] = len(review.rejected)
    data = finish(s, pr.number, pr.head_sha, pr.diff, review.summary, pairs,
                  usage, review.checklist)
    return data


def refused(s: Settings, pr: context.PrContext, e: agent.AgentRefusal,
            usage: Dict[str, Any]) -> Dict[str, Any]:
    """The reviewer model declined: no review, and say exactly why.

    Nothing is presented as a review. The artifact carries only the
    notice, with the model's own reason, category and request ID, so the
    author and maintainers know the PR was not reviewed and why.
    """
    usage['refused'] = {'model': e.model, 'category': e.category,
                        'request_id': e.request_id}
    artifact = render.write_refusal(s.out_dir, pr.number, pr.head_sha, e,
                                    s.model_label)
    report = render.refusal_report(e, artifact, usage)
    (s.work_dir / 'report.md').write_text(report, encoding='utf-8')
    (s.work_dir / 'usage.json').write_text(json.dumps(usage, indent=2))
    return {'routed': None, 'artifact': artifact, 'report': report,
            'usage': usage, 'refusal': e}


def sandbox_stats(log: Path) -> Dict[str, Any]:
    """How the reviewer used code execution, from sandbox.py's call log."""
    calls = []
    if log.exists():
        for line in log.read_text(encoding='utf-8').splitlines():
            try:
                calls.append(json.loads(line))
            except ValueError:
                continue
    return {'sandbox_calls': len(calls),
            'sandbox_failed_calls': sum(1 for c in calls
                                        if c.get('returncode') != 0),
            'sandbox_seconds': round(sum(float(c.get('seconds') or 0)
                                         for c in calls), 1)}


def finish(s: Settings, pr_number: int, head_sha: str, diff_text: str,
           model_summary: str,
           pairs: List[Tuple[findings.Finding, findings.Verdict]],
           usage: Dict[str, Any],
           checklist: Optional[Dict[str, findings.ChecklistEntry]] = None
           ) -> Dict[str, Any]:
    """Deterministic tail of the pipeline, also used by `render`."""
    s.work_dir.mkdir(parents=True, exist_ok=True)
    diff_map = diff.parse(diff_text)
    head_files = route.load_head_files(
        s.checkout, sorted({f.path for f, _ in pairs if f.path}))
    routed = route.route(pairs, diff_map, head_files)
    artifact = render.write_artifact(s.out_dir, pr_number, head_sha, routed,
                                     model_summary, s.model_label, checklist)
    usage.update(sandbox_stats(s.work_dir / SANDBOX_LOG))
    report = render.report_markdown(routed, artifact, usage, checklist)
    (s.work_dir / 'report.md').write_text(report, encoding='utf-8')
    (s.work_dir / 'usage.json').write_text(json.dumps(usage, indent=2))
    return {'routed': routed, 'artifact': artifact, 'report': report,
            'usage': usage}
