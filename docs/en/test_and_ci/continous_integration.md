# PX4 Continuous Integration

PX4 uses GitHub Actions for continuous integration, with different workflows handling code builds, testing, and documentation.

## Documentation CI

The documentation pipeline handles building, deploying, and translating the PX4 User Guide.
All documentation CI is consolidated into a single orchestrator workflow organized in tiers.

### Docs Orchestrator

**Workflow file:** [`docs-orchestrator.yml`](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/docs-orchestrator.yml)

This is the main documentation workflow. It runs on pull requests and pushes to `main` and `release/**` branches, performing different jobs depending on the trigger event.
Jobs are organized in tiers, where each tier depends on the previous one completing successfully.

#### Tier Structure

| Tier | Job | PR | Push | Description |
|------|-----|----|------|-------------|
| 0 | Detect Changed Paths | Yes | — | Checks if source code files changed (triggers metadata regen) |
| 1 | Metadata Generation | Yes (conditional) | Yes | Builds PX4 SITL and regenerates all auto-generated docs |
| 2 | Link Validation | Yes | — | Checks for broken links in changed files, posts PR comment |
| 3 | Build Site | Yes (gated on Tier 2) | Yes (after Tier 1) | Builds the VitePress documentation site |
| 4 | Deploy + Crowdin Upload | — | Yes | Deploys to AWS S3 and uploads sources to Crowdin |

#### Pull Request Flow

When a PR modifies files in `docs/**`, the workflow validates the changes:

```
PR Event
    │
    ▼
┌─────────────────────────────────────┐
│ Tier 0: Detect Changed Paths       │
│  • Checks if src/msg/ROMFS changed  │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ Tier 1: PR Metadata Regen          │
│  (only if source files changed)     │
│  • Builds px4_sitl_default          │
│  • Generates parameter/airframe/    │
│    module documentation             │
│  • Builds failsafe web simulator    │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ Tier 2: Link Validation (~30 sec)  │
│  • Detects changed docs/en/*.md     │
│  • Runs filtered link check         │
│  • Posts sticky PR comment (flaws)  │
│  • Runs full link check (artifact)  │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ Tier 3: Build Site (~7-10 min)     │
│  (only if link check passed)        │
│  • Builds VitePress site            │
│  • Verifies no build errors         │
└─────────────────┬───────────────────┘
                  │
                  ▼
                DONE
```

| Job | Duration | Description |
|-----|----------|-------------|
| **Detect Changed Paths** | ~10s | Determines if metadata regeneration is needed |
| **PR Metadata Regen** | ~10-15m | Rebuilds PX4 SITL and regenerates all metadata (conditional) |
| **Link Validation** | ~30s | Checks for broken links in changed markdown files and posts a sticky comment to the PR |
| **Build Site** | ~7-10m | Builds the VitePress site to verify there are no build errors. Gated on link check passing. |

#### Push Flow (main/release branches)

When changes are pushed to `main` or `release/**` branches, the workflow regenerates metadata, builds, and deploys:

```
Push Event
    │
    ▼
┌─────────────────────────────────────┐
│ Tier 1: Regenerate Metadata         │
│  (~10-15 min)                       │
│  • Builds px4_sitl_default          │
│  • Generates parameter/airframe/    │
│    module documentation             │
│  • Builds failsafe web simulator    │
│  • Formats with Prettier            │
│  • Auto-commits if changes detected │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ Tier 3: Build Site (~7-10 min)     │
│  • Builds VitePress site            │
│  • Uploads build artifact           │
└─────────────────┬───────────────────┘
                  │
          ┌───────┴───────┐
          ▼               ▼
┌──────────────┐  ┌──────────────────┐
│ Tier 4:      │  │ Tier 4:          │
│ Deploy to    │  │ Crowdin Upload   │
│ AWS (~3 min) │  │ (~4 min)         │
│ • S3 sync    │  │ • Upload sources │
└──────────────┘  └──────────────────┘
```

| Job | Duration | Description |
|-----|----------|-------------|
| **Regenerate Metadata** | ~10-15m | Rebuilds PX4 SITL, regenerates all metadata, formats with Prettier, auto-commits |
| **Build Site** | ~7-10m | Builds the VitePress documentation site |
| **Deploy to AWS** | ~3m | Syncs built site to AWS S3 (HTML: 60s cache, assets: 24h cache) |
| **Crowdin Upload** | ~4m | Uploads English source files to Crowdin for translation (main branch only) |

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
| ccache | 1GB | C++ compilation cache for SITL builds |
| node_modules | ~26MB | Node.js dependencies for VitePress |

### Infrastructure

Jobs run on [runs-on](https://runs-on.com/) self-hosted runners with S3 cache:

| Job | Runner |
|-----|--------|
| Detect Changed Paths | ubuntu-latest |
| Link Validation | ubuntu-latest |
| Metadata Regen | 4 CPU (with px4-dev container) |
| Build Site | 4 CPU |
| Deploy to AWS | ubuntu-latest |
| Crowdin Upload | ubuntu-latest |
