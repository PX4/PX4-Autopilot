---
name: commit
description: Create a PX4 conventional commit from reviewed changes with a topic-based scope and assistant disclosure.
---

# PX4 Conventional Commit

Read `.claude/skills/commit/SKILL.md` from the repository root and follow its
workflow with these substitutions:

- Input is the user's request accompanying this skill, not literal `$ARGUMENTS`.
- Use the current client's tools rather than Claude-specific tool names.
- Replace `Assisted-by: Claude:<model-id>` with
  `Assisted-by: Codex:<model-id>` when running in Codex. Use the actual client
  name in other clients. If the model ID is unavailable, omit the colon and
  model ID rather than inventing one.
- Preserve the user's authorship and the source workflow's DCO review
  requirement. Follow any higher-priority client requirements for trailers.
- Stage only changes the user has authorized; never include unrelated dirty
  submodules or untracked files.
