# get_mode_requirements

Before changing files in this directory, read the sibling
[CLAUDE.md](CLAUDE.md) and follow its generator-specific instructions.
That file is the shared source of truth for parser behavior, generated-content
boundaries, metadata ordering, and golden-file tests; it applies regardless of
which assistant or model is being used.

In particular, do not hand-edit generated sentinel blocks in `docs/en/`.
Change the generator inputs instead, and follow the documented test and
golden-file update workflow for intentional behavior changes.
