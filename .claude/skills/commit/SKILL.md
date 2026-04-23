---
name: commit
description: Create a conventional commit for PX4 changes
argument-hint: "[optional: description of changes]"
allowed-tools: Bash, Read, Glob, Grep
---

# PX4 Conventional Commit

Create a git commit in conventional-commit format: `type(scope): description`.

- **type:** `feat`, `fix`, `refactor`, `perf`, `docs`, `style`, `test`,
  `build`, `ci`, `chore`, `revert`. Append `!` before `:` for breaking changes.
- **scope:** the module/driver/area affected — derive from the directory
  path of the changed files (`src/modules/ekf2/` → `ekf2`,
  `src/drivers/imu/invensense/icm42688p/` → `drivers/icm42688p`,
  `.github/workflows/` → `ci`).
- **description:** imperative, concise, ≥5 chars.

**NEVER add Co-Authored-By Claude Code. No Claude attribution.**

## Steps

1. Check branch (`git branch --show-current`). If on `main`, create a feature
   branch `<username>/<description>` where `<username>` comes from
   `gh api user --jq .login`.
2. Run `git status` and `git diff --staged`. If nothing staged, ask what to stage.
3. Run `make format` (or `./Tools/astyle/fix_code_style.sh <file>`) on changed
   C/C++ files.
4. Body (if needed): explain **why**, not what.
5. Check GPG signing: `git config --get user.signingkey`. If set,
   `git commit -S -s`; else `git commit -s`.

If the user provided arguments, use them as context: $ARGUMENTS
