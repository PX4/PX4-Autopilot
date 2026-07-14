---
name: review-pr
description: Substance-focused review of a PX4 pull request — merit, first-principles correctness, architecture fit. Produces a debrief for the user and a draft review comment.
argument-hint: "<PR number or URL>"
allowed-tools: Bash, Read, Glob, Grep, Agent
---

# PX4 Pull Request Review

Goal: brief the user on a PR before they spend their own time on it, and draft a review comment they can choose to post. The review is by Claude and is presented as such — write from Claude's frame of reference, not the user's, unless the user explicitly requests a different voice.

Focus on substance only. Do not review commit messages, do not recommend a merge strategy, do not audit formatting (CI enforces it). Be extremely thorough — the user wants a deep understanding of the problem, not just verdicts — but scale effort with impact: roughly proportional to lines/files changed and how flight-critical the touched code is. For diffs too large to hold at once, fan out Explore agents per subsystem.

## Gather

In parallel:

- `gh pr view <PR> --json number,title,body,author,baseRefName,files,reviews`
- `gh pr diff <PR>` — on HTTP 406 (huge diff) do not retry; use `gh api repos/{owner}/{repo}/pulls/<PR>/files --paginate` and fetch key patches selectively
- `gh pr checks <PR>` (exit code 8 just means checks are pending)
- existing feedback: `gh api repos/{owner}/{repo}/pulls/<PR>/comments --paginate --jq '.[] | {user: .user.login, path, body}'` (inline) and `gh api repos/{owner}/{repo}/issues/<PR>/comments --paginate --jq '.[] | {user: .user.login, body}'` (conversation)

Read any linked issue — the claimed problem is the baseline for judging merit. For full post-change file contents without touching the working tree: `git fetch origin pull/<PR>/head`, then `git show FETCH_HEAD:<path>`.

## Review

Never judge a hunk from the diff alone. Read the enclosing function and file, and grep for callers and consumers — a PR is always reviewed in the context of the surrounding code.

- **Merit and need.** Is this solving a real problem or papering over one? Trace the failure mechanism; check the fix addresses the root cause at the right layer. Ask whether a simpler change (different default, existing parameter, deleting code) would achieve the same.
- **Physics and math.** If the change involves physics, estimation, control, or nontrivial math, do a first-principles analysis: re-derive the result and check the implementation against your derivation, not against the PR description. Verify units, reference frames (FRD/NED, body/earth), signs, singularities and edge cases (division by zero, gimbal lock, low airspeed), timestep/discretization dependence, and numerical robustness in the given representation (NaN propagation, cancellation, float32 precision limits).
- **Architecture and maintainability.** Does the change follow the patterns of the code around it (uORB usage, scheduling, param handling, data ownership)? Does it reimplement something an existing library provides (AlphaFilter, SlewRate, mathlib)? Does the logic live in the module that owns the data? Watch for hidden coupling — a change that silently breaks an assumption in another module; if cross-module coupling is genuinely necessary, expect a unit test that codifies the new contract and fails if it is later broken. Embedded reality: flash is scarce, no dynamic allocation in flight code, mind CPU cost.
- **Alternatives.** Identify plausible alternative implementations and judge each: better, worse, or not worth mentioning. Raise only the ones the author should genuinely consider, and say why.
- **Correctness and reliability.** Edge cases, overflow, unsigned arithmetic, initialization, bounds, races, resource exhaustion, and failure-path handling. Flag real defects, not style.
- **Compatibility.** Changes to `msg/`, params, or MAVLink: impact on QGC, uLog tooling, and third-party integrations. uORB: `timestamp` vs `timestamp_sample`, `device_id`. Every new parameter is configuration burden on users — challenge it.
- **New board** (`boards/<mfr>/<board>/`): require a logs.px4.io flight log for the vehicle type, a docs page in `docs/en/flight_controller/`, the manufacturer's own USB VID/PID, a unique board_id in `boards.json`, CI build coverage, flash fit, and no copy-pasted template leftovers or board-local forks of common drivers.

Also, briefly: the PR title must be `type(scope): description` (it becomes the commit subject on squash); note CI status and read failure logs only when plausibly related to the change; do not repeat feedback other reviewers already gave — build on it or stay silent.

## Deliver

End with two things, then stop — do not post anything.

**1. Debrief for the user.** Lead with the findings — the user wants actionable information, not a play-by-play of the diff. Give only the framing they need to judge the change: whether the claimed problem is real, scope and risk (subsystems touched, flight-critical or not), and what CI and other reviewers already said. Restate what the change actually does and why only when the PR description is missing, ambiguous, or misleading. Order findings by severity (blocker / concern / nit) with `file:line`, and end with a one-or-two-sentence overall take.

**2. Draft review comment**, shown verbatim in a fenced block. First line: `**Claude review on behalf of @<login>**` (login via `gh api user --jq .login`). Then terse, actionable findings only — what is wrong, why it matters, what to do, with file:line references. Frame each as an objective engineering tradeoff, not a moral judgment. Never guess: if you cannot demonstrate a flaw via a concrete code path or a first-principles derivation, stay silent. No praise or "verified OK" notes unless they change what the author should do, no meta sections, no conversational filler, nothing already raised by others.

If the user asks to post it, post the draft verbatim with `gh pr comment <PR> --body "$(cat <<'EOF' ... EOF)"`.

PR to review: $ARGUMENTS
