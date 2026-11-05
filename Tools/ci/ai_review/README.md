# AI pull request review

Automated review of PX4 pull requests by Claude (Opus 5.5 by default) on
Amazon Bedrock. The goal is a useful first pass on every PR, including PRs
from outside contributors, that maintainers can trust enough not to
ignore: few comments, each one actionable, and an explicit statement of
what the model is unsure about.

Status: **pilot**. The workflow (`.github/workflows/pr-ai-review.yml`)
posts its result on the PR as a COMMENT review through
`pr-review-poster.yml` (never approve or request changes). Each run marks
the previous AI review as superseded instead of adding another, and a
summary-only review (or a refusal notice) is posted too. The full report
is also in the job summary and the `ai-review-report` artifact. Start it
in either of two ways:

- **Comment on the PR** with `!ai-review` as the entire comment,
  optionally `!ai-review model=sonnet effort=high`. A comment that says
  anything else as well does not trigger. Only people with write, maintain
  or admin access can trigger it: `trigger.py` looks up the commenter's
  repository permission through the API (not `author_association`, which
  can be "MEMBER" for read-only org members). The comment gets 👀 when
  accepted, then 🚀 when the review finished or 😕 when it failed. A
  malformed command (unknown model, extra words) does nothing.
- **Run the workflow manually** (Actions, AI PR Review, Run workflow) with
  the PR number.

Either way, the PR head gets an "AI PR Review" commit status linking to
the run: pending while it works, then success or error. The workflow runs
from `main`, so without it the review would not show among the PR's
checks. The status is informational and must never be made a required
check.

## Pipeline

1. **Gather** (`context.py`): PR metadata, linked issues, commits, CI
   status, per-file patches, existing human review comments and this
   tool's own earlier findings, via `gh`. Every PR is reviewed whatever
   its size. Generated files (lock files, Crowdin translations) are listed
   but not diffed. When the diff exceeds the prompt budget, the reviewer
   gets the remaining files with their stats and reads them from the
   checkout, and its turn budget and timeout grow with the PR's size.
2. **Review** (`agent.py`, `prompts/reviewer.md`): Claude Code runs headless
   on the PR head checkout with read-only tools and returns findings as
   JSON (`findings.py` defines the schema). Each finding carries a severity,
   a kind (`code`, or `process` for test evidence, description, upgrade
   notes and docs), the one-sentence comment that gets posted, and the
   evidence behind it: the trigger, a concrete suggestion, and what the
   model could not confirm.
3. **Validate** (`prompts/validator.md`): a separate call per finding sees
   only that finding and its file's diff, checks it against the code, and
   returns keep or drop plus a confidence. Confidence comes from this
   independent check, not from the reviewer grading itself.
4. **Route** (`route.py`): code, not the model, decides where findings go.
   Nits are never posted. A code finding goes inline only when the
   validator kept it with high confidence, its suggestion is more than
   "please verify", and its lines are in the diff; at most 10. PR-level
   code findings kept with high or medium confidence are listed as
   must-fix, and the remaining code findings as worth checking, one
   non-blocker per file. Process findings get a one-line list of their
   own, at most 4, and never change the verdict. A GitHub suggestion block
   is attached only when the replacement fully fixes the issue, is at most
   6 lines, changes something, and keeps the indentation.
5. **Render** (`render.py`): writes the `pr-review` artifact in the format
   `pr-review-poster.yml` consumes. The review body is the verdict (don't
   merge, merge after fixes, or no code issues found) with the title of
   the finding that decides it, then one sentence per posted finding. The
   model's summary, the checklist, nits and each finding's evidence and
   validator reasoning go to the report in the job summary, which the
   review links to.

The review criteria themselves live in
`.agents/skills/review-pr/review-criteria.md`, shared with the interactive
`review-pr` skill, plus the `.github/instructions/*.instructions.md` files
whose `applyTo` globs match the changed paths. Process checks (test
evidence, an accurate description, upgrade notes and docs) are CI-only, in
`prompts/contribution.md`: an unattended review can afford to raise them,
while the interactive skill stays focused on substance.

## Trust model

The PR is untrusted input: its code, title, description and comments are
written by anyone who can open a PR.

- **The agent cannot be configured by the PR.** Claude Code runs with
  `--bare --restricted --strict-mcp-config`, so nothing in the checkout
  (CLAUDE.md, AGENTS.md, settings, hooks, skills, plugins, MCP servers) is
  loaded. Trusted prompts and rules come from the base checkout.
- **The agent can only read the checkout.** The only tools are Read and
  Bash, and only `Read`, `grep` and `ls` are allowed; `dontAsk` refuses
  everything else. Claude Code checks each Bash call, so chained commands,
  other programs (`printenv`, `curl`) and paths outside the checkout are
  refused, and `--restricted` confines Read to the checkout. Nothing from
  the PR is built or executed.
- **Model-written code runs only in the sandbox.** The one exception to
  read-only is `px4-sandbox-python -c CODE` (`sandbox.py`), so the model
  can check its math. It runs the code in the pinned px4-dev image with no
  network, no host environment, the checkout read-only, a non-root user,
  all capabilities dropped, CPU/memory/process/time limits, and in CI the
  gVisor runtime. The command accepts nothing but the code; chaining,
  substitution, redirection, environment prefixes and direct `docker` or
  `python3` calls are refused by Claude Code's permission check.
- **The runner's instance role is out of reach.** RunsOn runners carry an
  EC2 role that can write the CI cache bucket. Before the review starts,
  `python3 -m ai_review harden` installs gVisor (pinned, SHA-512 checked),
  blocks the EC2 metadata service for the host and containers, and
  self-tests the sandbox; any failure fails the job. Bedrock access comes
  from GitHub OIDC credentials that need no metadata service.
- **No GitHub write access in the review job.** The workflow token is
  read-only and is removed from the agent's environment. Posting is left
  to `pr-review-poster.yml`, which validates the artifact and only posts
  `COMMENT` reviews.
- **Bedrock credentials are short-lived and narrow.** The job assumes the
  `px4-ai-review` IAM role through GitHub OIDC. The role trusts only this
  workflow file on `main` and can only invoke two models. If the review
  output contains any credential from the environment, nothing is written
  and the job fails.
- **PR text is fenced as data** in tagged blocks, and closing tags inside
  it are escaped.

What a prompt injection can still do, worst first:

- Steer a one-click suggestion block toward harmful code. It still has to
  pass the independent validator, the suggestion checks (high confidence,
  at most 6 lines, matching indentation) and a human before merge.
- Produce a misleading review (a false "clean" or a fake blocker), or
  embarrassing text posted as `github-actions[bot]`.
- Waste one job's model budget within its turn limit and timeout.
- Escape the sandbox only by chaining a gVisor bug with a host kernel bug,
  and then still find the metadata service blocked. A root escape could
  remove that block; the planned fix is a dedicated runner role with no
  CI permissions in the RunsOn v3 rebuild.

## Updating models

Models live in one place, `models.json`: stable aliases (`opus`,
`sonnet`) mapped to a Bedrock global inference profile and a display
label, plus the default model and effort for the reviewer and the
validator. The workflow, CLI and prompts only use the aliases.

To move to a new version (say Opus 5.6):

1. **AWS, once per new model:** accept its Marketplace agreement in the
   px4-ci account (Bedrock console, Model access, or
   `aws bedrock create-foundation-model-agreement`). The `px4-ai-review`
   role allows every `global.anthropic.claude-opus-*` and
   `claude-sonnet-*` profile, so no IAM change is needed; other families
   and regional profiles stay denied.
2. **Check it:** `python3 -m ai_review models-check` sends a one-token
   request to every model in `models.json` with your credentials and
   reports missing agreements, missing profiles or IAM denials.
3. **Change one line** in `models.json` (`bedrock_profile` and `label`)
   and open a PR. Unit tests reject profiles outside the allowed families.
4. If the new model needs a newer Claude Code, bump `CLAUDE_CODE_VERSION`
   in `.github/workflows/pr-ai-review.yml` in the same PR.

Before switching the default, run the pilot comparison on a few PRs with
`--model` set to the new alias (add it under a temporary alias such as
`opus-next` first) and compare findings and cost against the current one.

## Next

- **Flight log evidence (TODO next).** When a PR links logs.px4.io logs as
  test evidence, the gather step (not the agent) downloads them with a size
  cap and runs `flight-review analyze` from
  [flight-review-rs](https://github.com/PX4/flight-review-rs), passing the
  JSON to the reviewer so it can check what the log shows. Needs a tagged
  flight-review-rs release binary to pin.
- **Related and conflicting PRs.** Read-only `gh` search for the agent.

## Running locally

From `Tools/ci`, with AWS credentials that may invoke the Bedrock models
and `gh` authenticated:

```sh
git -C /tmp/pr fetch origin pull/<N>/head && git -C /tmp/pr checkout FETCH_HEAD
python3 -m ai_review run --pr <N> --trusted-root ../.. --checkout /tmp/pr \
    --model sonnet --effort high
```

Re-render saved findings without calling a model (useful when changing
routing or formatting):

```sh
python3 -m ai_review render --pr <N> --trusted-root ../.. --checkout /tmp/pr \
    --findings ai-review-work/findings.json --diff pr.diff --head-sha <sha>
```

## Tests

`python3 -m unittest discover -s ai_review -t .` from `Tools/ci`; CI runs
them with `mypy --strict` and `flake8` in `python_checks.yml`.

| Test | Risk it covers |
|---|---|
| `test_findings.py` | Malformed or out-of-contract model output reaching a comment |
| `test_diff.py` | Comments on lines GitHub rejects, wrong size accounting |
| `test_route.py` | The confidence policy: what may go inline, caps, suggestion blocks |
| `test_render.py` | Artifacts the poster rejects; credentials in output |
| `test_agent.py` | Sandbox flags regressing; failures looking like a clean review |
| `test_context.py` | PR text escaping its data block; oversized diffs truncated silently |
| `test_pipeline.py` | Missing repository rules in the prompt; offline render path |
