# Agent instructions

`AGENTS.md` files hold repository and directory guidance; `.agents/skills/` holds the shared skills. Keep skills client-agnostic: no argument variables or client-specific attribution.

Claude Code reads neither location directly. Each `CLAUDE.md` is a one-line `@AGENTS.md` import, and each `.claude/skills/<name>` is a symlink to `.agents/skills/<name>`; add both when adding a nested `AGENTS.md` or a skill.

| Skill | Use |
| --- | --- |
| `commit` | Conventional commit with `Assisted-by:` trailer |
| `pr` | Pull request with Summary/Problem/Solution body |
| `review-pr` | PR review debrief and draft comment |
| `rebase-onto-main` | Rebase, including onto a squash-merged parent |
| `build-px4-linux` | Board or SITL build on a Linux host |
| `build-px4-macos` | Board build on macOS, in the `px4-dev` container |

Create worktrees outside the repository, e.g. `../PX4-Autopilot-worktrees/`.
