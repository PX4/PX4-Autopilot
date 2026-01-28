# PX4 Continuous Integration

PX4 uses GitHub Actions for continuous integration, with different workflows handling code builds, testing, and documentation.

## Documentation CI

The documentation pipeline handles building, deploying, and translating the PX4 User Guide.
It consists of two workflows that work together to keep docs.px4.io up to date.

### Docs CI Workflow

**Workflow file:** [`docs-orchestrator.yml`](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/docs-orchestrator.yml)

This is the main documentation workflow. It runs on pull requests and pushes to `main` and `release/**` branches, performing different jobs depending on the trigger event.

#### Pull Request Flow

When a PR modifies files in `docs/**`, the workflow validates the changes:

```
PR Event
    │
    ▼
┌─────────────────────────────────────┐
│ Link Check (~30 sec)                │
│  • Validates markdown links         │
│  • Posts results as PR comment      │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ Build Site (~7-10 min)              │
│  • Builds VitePress site            │
│  • Verifies no build errors         │
└─────────────────┬───────────────────┘
                  │
                  ▼
                DONE
```

| Job | Duration | Description |
|-----|----------|-------------|
| **Link Check** | ~30s | Checks for broken links in changed markdown files and posts a sticky comment to the PR |
| **Build Site** | ~7-10m | Builds the VitePress site to verify there are no build errors |

#### Push Flow (main/release branches)

When changes are pushed to `main` or `release/**` branches, the workflow regenerates metadata, builds, and deploys:

```
Push Event
    │
    ▼
┌─────────────────────────────────────┐
│ Regenerate Metadata (~10-15 min)    │
│  • Builds px4_sitl_default          │
│  • Generates parameter/airframe/    │
│    module documentation             │
│  • Builds failsafe web simulator    │
│  • Auto-commits if changes detected │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ Build Site (~7-10 min)              │
│  • Builds VitePress site            │
│  • Uploads build artifact           │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ Deploy to AWS (~3 min)              │
│  • Syncs to S3 bucket               │
│  • HTML: 60s cache                  │
│  • Assets: 24h cache                │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ Crowdin Upload (~4 min)             │
│  • Uploads English sources          │
│  • Enables translation workflow     │
└─────────────────┬───────────────────┘
                  │
                  ▼
                DONE
```

| Job | Duration | Description |
|-----|----------|-------------|
| **Regenerate Metadata** | ~10-15m | Rebuilds PX4 SITL and regenerates all documentation metadata |
| **Build Site** | ~7-10m | Builds the VitePress documentation site |
| **Deploy to AWS** | ~3m | Deploys the built site to AWS S3 |
| **Crowdin Upload** | ~4m | Uploads English source files to Crowdin for translation |

#### Generated Metadata

The metadata regeneration job creates the following auto-generated documentation:

| Type | Output | Description |
|------|--------|-------------|
| Parameters | `docs/en/advanced_config/parameter_reference.md` | Complete parameter reference |
| Airframes | `docs/en/airframes/airframe_reference.md` | Airframe configurations |
| Modules | `docs/en/modules/*.md` | Module documentation |
| Messages | `docs/en/msg_docs/*.md` | uORB message documentation |
| uORB Graphs | `docs/public/middleware/*.json` | Topic dependency graphs |
| Failsafe Simulator | `docs/public/config/failsafe/*` | Interactive failsafe simulator (WebAssembly) |

::: warning
Do not manually edit the auto-generated files listed above. They are overwritten on every push to main.
:::

#### Path Triggers

The workflow triggers on changes to these paths:

| Path | Reason |
|------|--------|
| `docs/**` | Documentation source files |
| `src/**` | Source code changes affect metadata |
| `msg/**` | Message definitions affect metadata |
| `ROMFS/**` | ROMFS files affect metadata |
| `Tools/module_config/**` | Module configuration affects metadata |

### Crowdin Download Workflow

**Workflow file:** [`docs_crowdin_download.yml`](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/docs_crowdin_download.yml)

This scheduled workflow downloads completed translations from Crowdin and creates pull requests.

| Setting | Value |
|---------|-------|
| **Schedule** | Every Sunday at 00:00 UTC |
| **Target Languages** | Korean (ko), Ukrainian (uk), Chinese Simplified (zh-CN) |

**Process:**

1. Downloads translations for each target language from Crowdin
2. Creates a separate PR for each language with new translations
3. PRs are labeled "Documentation" and assigned for review

### Caching Strategy

The workflows use caching to speed up builds:

| Cache | Size | Purpose |
|-------|------|---------|
| ccache | 250MB | C++ compilation cache for SITL builds |
| node_modules | ~26MB | Node.js dependencies for VitePress |

### Infrastructure

All jobs run on [runs-on](https://runs-on.com/) self-hosted runners with S3 cache:

| Job | Runner |
|-----|--------|
| Link Check | 2 CPU |
| Regenerate Metadata | 4 CPU (with px4-dev container) |
| Build Site | 4 CPU |
| Deploy to AWS | 2 CPU |
| Crowdin Upload | 2 CPU |
