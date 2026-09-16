---
name: rebase-onto-main
description: Rebase a PX4 branch onto main, handling squash-merged parent branches without replaying inherited commits.
---

# Rebase Branch onto Main

Read `.claude/skills/rebase-onto-main/SKILL.md` from the repository root and
follow its workflow with these adaptations:

- Input is the branch in the user's request, defaulting to the current branch.
  Do not interpret `$ARGUMENTS` as a shell variable.
- Use available tools; ignore claims about Claude's Bash execution environment.
  Explicitly check checkout and rebase failures regardless of shell behavior.
- Inspect the working tree and worktrees first. Preserve unrelated changes and
  run in the worktree that owns the branch. Do not automatically stash user work.
- If `main` is checked out in another worktree, fetch `origin main` and use
  `origin/main` as the new base rather than updating that checked-out branch.
- Record the old head and the unique-commit boundary before rewriting history.
  For a squash-merged parent, compare only the branch's unique commits in
  `git range-diff <first-unique-commit>^..<old-head> <new-base>..<new-head>`.
  Inherited commits intentionally excluded from the replay are not lost work.
  Investigate any changed, missing, or added unique patch before proceeding.
- Keep a backup ref until the result has been reviewed. Ask before force-pushing
  and use only `--force-with-lease`. Do not rewrite or push dependent branches
  without authorization.
