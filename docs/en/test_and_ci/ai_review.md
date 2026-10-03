# AI Pull Request Review

PX4 maintainers can ask an AI model to review a pull request.
The review is posted on the PR as a regular GitHub review comment, with inline comments on specific lines where it found problems.

::: warning
This is a trial.
Reviews can be wrong, and the tool may change as we learn what works.
Feedback from maintainers is welcome in the maintainers' channel or as an issue.
:::

## Who Can Use It

Only people with write, maintain or admin access to the PX4-Autopilot repository can start a review.
In practice this is the [Dev Team](https://github.com/orgs/PX4/teams/dev-team) (see [Maintainers](../contribute/maintainers.md)).
A request from anyone else is ignored.

## Starting a Review

Post a comment on the pull request that contains only this text:

```txt
!ai-review
```

The comment must contain nothing else.
A sentence that mentions the command, a quote of it, or the command with a typo does not start a review.

To pick the model or the effort level, add options on the same line:

```txt
!ai-review model=sonnet effort=high
```

- `model`: `opus` (the default) or `sonnet`.
- `effort`: `low`, `medium`, `high`, `xhigh` (the default) or `max`.

Within a few seconds your comment gets a 👀 reaction, which means the review has started.
When it finishes, the comment gets 🚀, or 😕 if the run failed.
A review takes about 5 to 15 minutes, depending on the size of the PR.
While it runs, an "AI PR Review" entry in the PR's checks shows its progress and links to the workflow run.
This check is informational and never blocks a merge.

You can also start a review from the **Actions** tab: open the [AI PR Review](https://github.com/PX4/PX4-Autopilot/actions/workflows/pr-ai-review.yml) workflow, select **Run workflow** and enter the PR number.

::: tip
Use reviews where a second opinion matters most: flight-critical code, new boards, large changes, and contributions from people new to the project.
Every review costs money (see [Cost](#cost)).
:::

## What the Review Contains

For an example, see [PR #28901](https://github.com/PX4/PX4-Autopilot/pull/28901).

- Inline comments on specific lines, for findings that a second, independent model pass confirmed with high confidence.
  When a small change fully fixes the problem, the comment includes a suggestion that the author can apply with one click.
- A "Before merge" list for issues with the pull request as a whole, such as missing test evidence, a description that does not match the change, or missing upgrade notes or documentation.
- A checklist covering whether the problem is real, test evidence, the description, compatibility, and documentation.
- A collapsed list of lower-confidence findings that may still be worth checking.
- The cost of the review, the model used and how long it took, at the bottom.

The review is always a comment.
It never approves a pull request or requests changes: maintainers decide.

Running the review again on the same pull request replaces the previous review, which is marked as superseded.

If the model's safety filter declines to review a pull request, which occasionally happens, the bot posts that no review was done and quotes the reason it was given.

## Cost

Each review is billed to the PX4 project.
So far, reviews have cost between about $0.60 for a small change and $5 to $8 for a large board addition with Claude Opus, and roughly an eighth of that with Claude Sonnet.
The exact cost is shown at the bottom of every review.

Claude Sonnet is faster and cheaper, but in testing it missed real problems that Claude Opus found, so Opus is the default.

## How It Works

1. A small job checks that the comment is exactly the command and that the commenter has write access, using the GitHub API.
   If either check fails, nothing else runs.
2. The review job runs _Claude Code_ with Claude Opus on Amazon Bedrock, on a PX4 CI runner.
   It reads the pull request, the surrounding code, linked issues, commit messages, CI status and git history.
   It reviews against the same criteria as the `review-pr` skill used by maintainers locally, plus the contribution requirements (test evidence, an accurate description, upgrade notes and documentation).
3. A second model pass checks each finding against the code.
   Only findings it confirms with high confidence are posted as inline comments.
4. The result is handed to the existing PR review poster workflow, which posts it on the pull request.

The pull request is treated as untrusted input:

- The reviewer can only read files and run a few read-only commands.
  It never builds or runs the pull request's code.
- When it needs to check a calculation, it runs Python in an isolated `px4-dev` container with no network access and no credentials.
- The review job has no write access to GitHub; only the separate posting workflow does.
- The review code always comes from `main`.
  A pull request cannot change how it is reviewed, and changes to the review tool only take effect once they are merged.

## Changing the Review

The code is in [`Tools/ci/ai_review/`](https://github.com/PX4/PX4-Autopilot/tree/main/Tools/ci/ai_review), a Python package with its own [README](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/ci/ai_review/README.md).
The workflow is [`.github/workflows/pr-ai-review.yml`](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/pr-ai-review.yml).

- The models and their default effort levels are set in `Tools/ci/ai_review/models.json`.
- The review criteria shared with the `review-pr` skill are in `.agents/skills/review-pr/review-criteria.md`.
- The CI-only prompts are in `Tools/ci/ai_review/prompts/`.

Pull requests to improve the review are welcome.
