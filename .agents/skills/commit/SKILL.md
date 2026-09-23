---
name: commit
description: Create a conventional commit for PX4 changes
argument-hint: "[optional: description of changes]"
allowed-tools: Bash Read Glob Grep
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

**The user is the author. Never add `Co-Authored-By` naming an AI. End the
commit body with the disclosure trailer `Assisted-by: <tool>:<model-id>`
(e.g. `Assisted-by: Claude:claude-fable-5`). Omit `:<model-id>` rather than
invent one.**

## Steps

1. Check branch (`git branch --show-current`). If on `main`, create a feature
   branch `<username>/<description>` where `<username>` comes from
   `gh api user --jq .login`.
2. Run `git status` and `git diff --staged`. If nothing staged, ask what to stage.
3. Run `make format` (or `./Tools/astyle/fix_code_style.sh <file>`) on changed
   C/C++ files.
4. Body (if needed): explain **why**, not what.
5. Commit with `git commit -s`. Do not pass `-S`: GPG signing is the user's
   git config decision (`commit.gpgsign`), not the skill's. The `-s` sign-off
   is the user's DCO certification of changes they have reviewed: never
   commit with `-s` work the user has not seen.
6. Add new commits rather than amending pushed ones, and never force-push
   unless the user asks or the branch is being rebased. PRs are squash-merged,
   so the branch history costs nothing on `main` and shows reviewers the
   decision chain.
7. If the branch has an upstream (`git rev-parse --abbrev-ref @{u}`), push.
8. If the branch has an open PR (`gh pr view --json state,body`), update its
   description with `gh pr edit --body` when the commit changes what the
   Summary, Problem, Solution or Testing sections say, following the `pr`
   skill's format. Leave it alone otherwise.
