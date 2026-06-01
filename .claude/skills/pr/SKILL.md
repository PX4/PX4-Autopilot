---
name: pr
description: Create a pull request with conventional commit title and description
argument-hint: "[optional: target branch or description]"
allowed-tools: Bash, Read, Glob, Grep
---

# PX4 Pull Request

**No Claude attribution anywhere (no Co-Authored-By, no "Generated with Claude").**

## Steps

1. Check branch. If on `main`, create a feature branch `<username>/<description>`
   where `<username>` comes from `gh api user --jq .login`.
2. Gather context: `git status`, `git log --oneline main..HEAD`,
   `git diff main...HEAD --stat`, check for remote tracking branch.
3. Sanity-build the targets we care about. Fix any build errors before opening
   the PR:
   - `make px4_fmu-v6x` — hardware target
   - `make px4_sitl` — simulation
4. PR **title:** `type(scope): description` — under 72 chars, covers the
   overall change across all commits. This becomes the squash-merge commit
   message.
5. PR **body:** start with a plain leading paragraph explaining what the PR does and why. No headings (`## Summary`, `## Test plan`, etc.), no boilerplate, no Claude attribution. Use bullet lists only to enumerate discrete changes; don't force prose into bullets. Describe testing inline if relevant, no separate test plan section. Use markdown (links, code blocks, lists) only when warranted. Keep it concise and well-formatted.

6. Push with `-u` if needed, then `gh pr create`. Default base is `main`
   unless user says otherwise.
7. Return the PR URL.

If the user provided arguments, use them as context: $ARGUMENTS
