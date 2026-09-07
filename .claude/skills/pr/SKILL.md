---
name: pr
description: Create a pull request with conventional commit title and description
argument-hint: "[optional: target branch or description]"
allowed-tools: Bash, Read, Glob, Grep
---

# PX4 Pull Request

**The user is the author: no `Co-Authored-By`, no "Generated with Claude"
footers. AI disclosure lives in the commit trailers (`Assisted-by:`), not in
the PR body.**

## Steps

1. Check branch. If on `main`, create a feature branch `<username>/<description>`
   where `<username>` comes from `gh api user --jq .login`.
2. Gather context: `git status`, `git log --oneline main..HEAD`,
   `git diff main...HEAD --stat`, check for remote tracking branch.
3. Sanity-build **one** target the change can actually affect — a board for
   firmware changes, `px4_sitl` for POSIX-only or simulation changes. Skip the
   build entirely when the diff cannot reach any target (submodule pointer
   bumps, docs, ROMFS). Fix any build errors before opening the PR.
4. PR **title:** `type(scope): description` — under 72 chars, covers the
   overall change across all commits. This becomes the squash-merge commit
   message.
5. PR **body:** as short as it can be while still landing the point — a
   reviewer should take it in at a glance, and a long description is one
   nobody reads. Exactly three sections, in order: `## Summary`, `## Problem`,
   `## Solution`, a sentence or two each. Do not restate the diff (no
   file-changed lists, no code snippets), do not mention CI, and do not repeat
   what the title already says. If the PR closes an issue, the first line of
   `## Summary` is `fixes #<N>`, then a blank line, then the summary. No
   `## Test plan` section, no boilerplate, no Claude attribution. Never state
   testing that did not happen: ask the user what they actually ran, report
   exactly that, and say plainly when something is untested.

6. Push with `-u` if needed, then `gh pr create`. Default base is `main`
   unless user says otherwise.
7. Return the PR URL.

If the user provided arguments, use them as context: $ARGUMENTS
