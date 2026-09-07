---
name: review-pr
description: Review a PX4 pull request for engineering merit, first-principles correctness, and architecture fit; deliver a debrief and an unpublished draft.
---

# PX4 Pull Request Review

Read `.claude/skills/review-pr/SKILL.md` from the repository root and follow
its workflow with these substitutions:

- Input is the PR number or URL in the user's request, not literal `$ARGUMENTS`.
- Use the current client's tools. Delegate independent subsystem investigations
  only when subagents are available and the scope warrants them; otherwise
  investigate directly.
- Write from the actual assistant's perspective, not Claude's or the user's.
  In Codex, the draft's first line is
  `**Codex review on behalf of @<login>**`. Use the actual client name elsewhere;
  obtain the login with `gh api user --jq .login` as in the source workflow.
- Read the applicable `.github/instructions/` guidance referenced by `AGENTS.md`.
- Deliver both the debrief and fenced draft. Do not post the draft unless
  explicitly requested.
