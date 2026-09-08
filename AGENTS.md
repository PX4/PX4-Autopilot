# PX4-Autopilot

Safety-critical C/C++ flight control firmware for autopilots, plus SITL
simulation and Python tooling.

- **Commits:** use the `$commit` skill. Conventional commit format with
  topic-based scope: `type(scope): description`.
- **Pull requests:** use the `$pr` skill.
- **Attribution:** use the actual assistant identity, never Claude's when
  running another client. No generated-by footer in PR bodies.
- **Style:** run `make format` on changed C/C++ before committing; CI
  enforces `make check_format`.
- **Scoped guidance:** before editing or reviewing, read only the
  `.github/instructions/*.instructions.md` files whose `applyTo` patterns
  match the affected paths, plus any nested `AGENTS.md` along those paths.
