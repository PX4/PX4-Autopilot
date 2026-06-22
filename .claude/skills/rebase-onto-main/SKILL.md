---
name: rebase-onto-main
description: Rebase a branch onto main, handling squash-merged parent branches cleanly
argument-hint: "[optional: branch name, defaults to current branch]"
allowed-tools: Bash, Read, Glob, Grep, Agent
---

# Rebase Branch onto Main

Rebase the current (or specified) branch onto `main`, correctly handling the case where the branch was built on top of another branch that has since been squash-merged into `main`.

## Background

When a parent branch is squash-merged, its individual commits become a single new commit on `main` with a different hash. A normal `git rebase main` will try to replay the parent's original commits, causing messy conflicts. The fix is to **cherry-pick only the commits unique to this branch** onto a fresh branch from `main`.

## Steps

1. **Identify the branch.** Use `$ARGUMENTS` if provided, otherwise use the current branch.

2. **Fetch and update main:**
   ```
   git fetch origin main:main
   ```

3. **Find the merge base** between the branch and `main`:
   ```
   git merge-base <branch> main
   ```

4. **List all commits** on the branch since the merge base:
   ```
   git log --oneline <merge-base>..<branch>
   ```

5. **Identify which commits are unique to this branch** vs. inherited from a parent branch. Look for:
   - Squash-merged commits on `main` that correspond to a group of commits at the bottom of the branch's history (check PR titles, commit message keywords).
   - The boundary commit: the first commit that belongs to *this* branch's work, not the parent's.
   - If ALL commits are unique (no parent branch), just do a normal `git rebase main` and skip the rest.

6. **Create a fresh branch from `main`:**
   ```
   git checkout -b <branch>-rebase main
   ```

7. **Cherry-pick only the unique commits** (oldest first):
   ```
   git cherry-pick <first-unique-commit>^..<branch>
   ```
   The `A^..B` range means "from the parent of A through B inclusive."

8. **Handle conflicts** if any arise during cherry-pick. Resolve and `git cherry-pick --continue`.

9. **Replace the old branch:**
   ```
   git branch -m <branch> <branch>-old
   git branch -m <branch>-rebase <branch>
   ```

10. **Verify** the result:
    ```
    git log --oneline main..<branch>
    ```
    Confirm only the expected commits are present.

11. **Ask the user** before force-pushing. When approved:
    ```
    git push origin <branch> --force-with-lease
    ```

12. **Clean up** the old branch:
    ```
    git branch -D <branch>-old
    ```
