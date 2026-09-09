---
name: pr
description: Create a PX4 pull request with a conventional title and concise Summary, Problem, and Solution sections.
---

# PX4 Pull Request

Read `.claude/skills/pr/SKILL.md` from the repository root and follow its
workflow with these substitutions:

- Input is the user's request accompanying this skill, not literal `$ARGUMENTS`.
- Use the current client's tools rather than Claude-specific tool names.
- Apply the source's ban on Claude attribution to generated-by footers for
  any assistant. Keep disclosure in commit trailers, not the PR body.
- If a firmware build is needed, read `.agents/skills/build-px4/SKILL.md`.
  Choose a target affected by the change; do not build for documentation-only
  changes or imply that a build proves flight safety.
- Use `gh` for GitHub operations and return the created PR URL.
