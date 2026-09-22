---
name: pr
description: Create a pull request with conventional commit title and description
argument-hint: "[optional: target branch or description]"
allowed-tools: Bash Read Glob Grep
---

# PX4 Pull Request

**The user is the author: no `Co-Authored-By`, no "Generated with <assistant>"
footers. Disclose AI assistance as the last line of the PR body, italicized:
`*Assisted-by: <tool>:<model-id>*`, matching the commit trailer.**

## Steps

1. Check branch. If on `main`, create a feature branch `<username>/<description>`
   where `<username>` comes from `gh api user --jq .login`.
2. Gather context: `git status`, `git log --oneline main..HEAD`,
   `git diff main...HEAD --stat`, check for remote tracking branch.
3. Sanity-build **one** target the change can actually affect — a board for
   firmware changes, `px4_sitl` for POSIX-only or simulation changes. Skip the
   build entirely when the diff cannot reach any target (submodule pointer
   bumps, docs, ROMFS). Build with the `build-px4-linux` or `build-px4-macos`
   skill. Fix any build errors before opening the PR.
4. PR **title:** `type(scope): description` — under 72 chars, covers the
   overall change across all commits. This becomes the squash-merge commit
   message.
5. PR **body:** as short as it can be while still landing the point — a
   reviewer should take it in at a glance, and a long description is one
   nobody reads. Sections, in order: `## Summary`, `## Problem`,
   `## Solution`, a sentence or two each. Do not restate the diff (no
   file-changed lists, no code snippets), do not mention CI, and do not repeat
   what the title already says. If the PR closes an issue, the first line of
   `## Summary` is `fixes #<N>`, then a blank line, then the summary. No
   boilerplate.
6. Optional `## Testing` section after `## Solution`, only for substantial
   testing: SITL scenarios, synthetic or replayed data, hardware-in-the-loop,
   bench or flight tests. Building and unit tests are not testing; never
   mention them. State what was run and what it showed, as tersely as the
   other sections. Never report testing that did not happen: ask the user
   what they actually ran.
7. Push with `-u` if needed, then `gh pr create`. Default base is `main`
   unless user says otherwise.
8. Return the PR URL.
