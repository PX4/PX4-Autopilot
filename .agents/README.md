# GPT-based coding agents

This repository supports OpenAI Codex alongside the existing Claude Code setup.
Instruction discovery belongs to the client, not the model: choosing a GPT
model in another client does not make that client read Codex configuration.

Codex reads the root `AGENTS.md` and discovers skills under `.agents/skills/`.
No global configuration changes, plugins, or copied personal permissions are
required. Start Codex from this checkout and use `/skills` to browse, or invoke
a skill directly:

```text
$commit describe the staged changes
$pr target main
$review-pr <PR number or URL>
$rebase-onto-main <branch>
$build-px4 px4_fmu-v6xrt_default
```

Skills can also be selected from a matching natural-language request. GitHub
workflows require an authenticated `gh` CLI; container builds require Docker.
No model is pinned, so the instructions work with the GPT model selected in
your client. Restart Codex if newly added skills do not appear.

## Shared guidance

| Entry point | Source of workflow guidance |
| --- | --- |
| `AGENTS.md` | Repository conventions and references to `.github/instructions/` |
| `skills/commit/SKILL.md` | `.claude/skills/commit/SKILL.md` |
| `skills/pr/SKILL.md` | `.claude/skills/pr/SKILL.md` |
| `skills/review-pr/SKILL.md` | `.claude/skills/review-pr/SKILL.md` |
| `skills/rebase-onto-main/SKILL.md` | `.claude/skills/rebase-onto-main/SKILL.md` |
| `skills/build-px4/SKILL.md` | Self-contained container build workflow |
| `../docs/scripts/get_mode_requirements/AGENTS.md` | The generator's existing local `CLAUDE.md` |

The four adapters load the existing tracked workflow instead of maintaining
duplicate copies. Keep workflow changes in those shared source files; keep
Codex-specific invocation, tool, and attribution adjustments in the adapters.
References to `.claude/` are ordinary repository file reads, not a dependency
on having Claude Code installed.

The build skill is self-contained because the existing local Claude build
skill is not tracked. It does not depend on local Claude settings, plans,
worktrees, or permissions. Agent-created build worktrees go under the ignored
`.agents/worktrees/` directory.

Nested `AGENTS.md` files bridge existing directory-specific guidance without
duplicating it. The Claude entry points remain unchanged.

See the official Codex documentation for
[AGENTS.md discovery](https://developers.openai.com/codex/guides/agents-md) and
[skills](https://developers.openai.com/codex/skills).
