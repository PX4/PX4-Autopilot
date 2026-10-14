---
name: review-pr
description: Substance-focused review of a PX4 pull request — merit, first-principles correctness, architecture fit. Produces a debrief for the user and a draft review comment.
argument-hint: "<PR number or URL>"
allowed-tools: Bash Read Glob Grep Agent
---

# PX4 Pull Request Review

Goal: brief the user on a PR before they spend their own time on it, and draft a review comment they can choose to post. The review is by the assistant and is presented as such — write from its own frame of reference, not the user's, unless the user explicitly requests a different voice.

Focus on substance only. Do not review commit messages, do not recommend a merge strategy, do not audit formatting (CI enforces it). Be extremely thorough — the user wants a deep understanding of the problem, not just verdicts — but scale effort with impact: roughly proportional to lines/files changed and how flight-critical the touched code is. For diffs too large to hold at once, fan out subagents per subsystem when the client has them.

## Gather

In parallel:

- `gh pr view <PR> --json number,title,body,author,baseRefName,files,reviews`
- `gh pr diff <PR>` — on HTTP 406 (huge diff) do not retry; use `gh api repos/{owner}/{repo}/pulls/<PR>/files --paginate` and fetch key patches selectively
- `gh pr checks <PR>` (exit code 8 just means checks are pending)
- existing feedback: `gh api repos/{owner}/{repo}/pulls/<PR>/comments --paginate --jq '.[] | {user: .user.login, path, body}'` (inline) and `gh api repos/{owner}/{repo}/issues/<PR>/comments --paginate --jq '.[] | {user: .user.login, body}'` (conversation)

Read any linked issue — the claimed problem is the baseline for judging merit. For full post-change file contents without touching the working tree: `git fetch origin pull/<PR>/head`, then `git show FETCH_HEAD:<path>`.

## History (optional)

When the PR changes existing behavior rather than adding something new, check whether the author has done their homework on the area:

- Open issues and PRs touching the same code or symptom: `gh issue list --search`, `gh pr list --search`. Is this already being worked on, or duplicated?
- `git log -L` / `git log --follow -p` and `git blame` on the changed lines, then the PRs and issues those commits reference. Was the current behavior intentional, and was the reasoning sound?
- Deferred work: TODOs, issue threads, or review comments where the change was considered and deliberately postponed or rejected.

Report only what bears on the PR: a reverted decision the author does not acknowledge, conflicting in-flight work, or an unaddressed reason the code is the way it is.

## Review

Apply the criteria in [review-criteria.md](review-criteria.md). Read it before reviewing.

## Deliver

End with two things, then stop — do not post anything.

**1. Debrief for the user.** Lead with the findings — the user wants actionable information, not a play-by-play of the diff. Give only the framing they need to judge the change: whether the claimed problem is real, scope and risk (subsystems touched, flight-critical or not), and what CI and other reviewers already said. Restate what the change actually does and why only when the PR description is missing, ambiguous, or misleading. Order findings by severity (blocker / concern / nit) with `file:line`.

**2. Draft review comment**, shown verbatim in a fenced block. First line: `**<assistant> review on behalf of @<login>**` (e.g. `Claude`, `Codex`) (login via `gh api user --jq .login`). Then terse, actionable findings only — what is wrong, why it matters, what to do, with file:line references. Frame each as an objective engineering tradeoff, not a moral judgment. Never guess: if you cannot demonstrate a flaw via a concrete code path or a first-principles derivation, stay silent. No praise or "verified OK" notes unless they change what the author should do, no meta sections, no conversational filler, nothing already raised by others.

If the user asks to post it, post the draft verbatim with `gh pr comment <PR> --body "$(cat <<'EOF' ... EOF)"`.
