# PX4-Autopilot

Safety-critical C/C++ flight control firmware for autopilots, plus SITL
simulation and Python tooling.

- **Commits:** use the `commit` skill. Conventional commit format with
  topic-based scope: `type(scope): description`.
- **Pull requests:** use the `pr` skill.
- **Builds:** use the `build-px4-linux` or `build-px4-macos` skill for the
  host OS.
- **Attribution:** disclose AI assistance only in the `Assisted-by:` commit
  trailer. No `Co-Authored-By` naming an AI, no generated-by footer in PR
  bodies.
- **Style:** run `make format` on changed C/C++ before committing; CI
  enforces `make check_format`.
- **Docs:** edit only `docs/en`. `docs/ko`, `docs/zh` and `docs/uk` are
  Crowdin output regenerated from it; revert any change a sweep makes there.
- **Scoped guidance:** before editing or reviewing, read only the
  `.github/instructions/*.instructions.md` files whose `applyTo` patterns
  match the affected paths, plus any nested `AGENTS.md` along those paths.
