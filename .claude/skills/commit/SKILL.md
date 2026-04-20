---
name: commit
description: Create a conventional commit for PX4 changes
argument-hint: "[optional: description of changes]"
allowed-tools: Bash, Read, Glob, Grep
---

# PX4 Conventional Commit

Create a git commit: `type(scope): description`

**NEVER add Co-Authored-By lines. No Claude attribution in commits.**

Follow [CONTRIBUTING.md](../../CONTRIBUTING.md) for full project conventions.

## Steps

1. **Read [CONTRIBUTING.md](../../CONTRIBUTING.md)** for commit message format, types, scopes, and conventions.
2. Check branch (`git branch --show-current`). If on `main`, create a feature branch. Use `<username>/<description>` format where `<username>` comes from `gh api user --jq .login`. If unavailable, just use `<description>`.
3. Run `git status` and `git diff --staged`. If nothing staged, ask what to stage.
4. Follow the commit message convention from CONTRIBUTING.md: pick the correct **type** and **scope**, write a concise imperative description. The scope table is not exhaustive — derive the scope from the directory path of the changed files.
5. Body (if needed): explain **why**, not what.
6. Run `make format` or `./Tools/astyle/fix_code_style.sh <file>` on changed C/C++ files before committing.
7. Check if GPG signing is available: `git config --get user.signingkey`. If set, use `git commit -S -s`. Otherwise, use `git commit -s`.
8. Stage and commit. No `Co-Authored-By`.

If the user provided arguments, use them as context: $ARGUMENTS
