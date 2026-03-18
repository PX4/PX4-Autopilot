---
name: pr
description: Create a pull request with conventional commit title and description
disable-model-invocation: true
argument-hint: "[optional: target branch or description]"
allowed-tools: Bash, Read, Glob, Grep
---

# PX4 Pull Request

**No Claude attribution anywhere (no Co-Authored-By, no "Generated with Claude").**

## Steps

1. Check branch. If on `main`, create a feature branch. Use `<username>/<description>` format where `<username>` comes from `gh api user --jq .login`. If unavailable, just use `<description>`.
2. Gather context: `git status`, `git log --oneline main..HEAD`, `git diff main...HEAD --stat`, check if remote tracking branch exists.
3. PR **title**: `type(scope): description` — under 72 chars, describes the overall change across all commits. This becomes the squash-merge commit message.
4. PR **body**: brief summary + bullet points for key changes. No filler.
5. Push with `-u` if needed, then `gh pr create`. Default base is `main` unless user says otherwise.
6. Return the PR URL.

If the user provided arguments, use them as context: $ARGUMENTS
