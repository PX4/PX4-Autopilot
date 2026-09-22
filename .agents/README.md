# Agent instructions

`AGENTS.md` files hold repository and directory guidance; `.agents/skills/` holds the shared skills. Keep skills client-agnostic: no client-specific tool names, argument variables, or attribution.

| Skill | Use |
| --- | --- |
| `commit` | Conventional commit with `Assisted-by:` trailer |
| `pr` | Pull request with Summary/Problem/Solution body |
| `review-pr` | PR review debrief and draft comment |
| `rebase-onto-main` | Rebase, including onto a squash-merged parent |
| `build-px4-linux` | Board or SITL build on a Linux host |
| `build-px4-macos` | Board build on macOS, in the `px4-dev` container |

Agent-created worktrees go under `.agents/worktrees/`, which is ignored.
