---
name: commit
description: Create a conventional commit for PX4 changes
disable-model-invocation: true
argument-hint: "[optional: description of changes]"
allowed-tools: Bash, Read, Glob, Grep
---

# PX4 Conventional Commit

Create a git commit: `type(scope): description`

**NEVER add Co-Authored-By lines. No Claude attribution in commits.**

Follow [CONTRIBUTING.md](../../CONTRIBUTING.md) for full project conventions.

## Steps

1. Check branch (`git branch --show-current`). If on `main`, create a feature branch. Use `<username>/<description>` format where `<username>` comes from `gh api user --jq .login`. If unavailable, just use `<description>`.
2. Run `git status` and `git diff --staged`. If nothing staged, ask what to stage.
3. Pick **type**: `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `revert`
4. Pick **scope** from changed file paths (e.g. `ekf2`, `mavlink`, `sensors`).
5. Write concise imperative description, lowercase, no period, under 72 chars.
6. Breaking changes: `type(scope)!: description`
7. Body (if needed): explain **why**, not what.
8. Run `make format` or `./Tools/astyle/fix_code_style.sh <file>` on changed C/C++ files before committing.
9. Stage and commit. No `Co-Authored-By`.

## Examples

```
feat(ekf2): add height fusion timeout
fix(mavlink): correct BATTERY_STATUS_V2 parsing
refactor(navigator): simplify RTL altitude logic
```

If the user provided arguments, use them as context: $ARGUMENTS
