# Безперервна інтеграція

PX4 uses GitHub Actions for continuous integration, with different workflows handling code builds, testing, and documentation.

## Documentation CI

The documentation pipeline handles building, deploying, and translating the PX4 User Guide.
All documentation CI is consolidated into a single orchestrator workflow organized in tiers.

### Docs Orchestrator

**Workflow file:** [`docs-orchestrator.yml`](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/docs-orchestrator.yml)

This is the main documentation workflow. It runs on pull requests, pushes to `main` and `release/**` branches, and manual `workflow_dispatch` triggers, performing different jobs depending on the trigger event.
Jobs are organized in tiers, where each tier depends on the previous one completing successfully.

#### Tier Structure

| Tier | Job            | PR                                              | Push / Dispatch                   | Опис                                                                             |
| ---- | -------------- | ----------------------------------------------- | --------------------------------- | -------------------------------------------------------------------------------- |
| T1   | Detect Changes | Так                                             | —                                 | Checks if source code files changed (triggers metadata regen) |
| T2   | PR Metadata    | Yes (conditional)            | —                                 | Builds PX4 SITL and regenerates all auto-generated docs                          |
| T2   | Metadata Sync  | —                                               | Так                               | Builds PX4 SITL, regenerates metadata, auto-commits                              |
| T2   | Link Check     | Так                                             | —                                 | Checks for broken links in changed files, posts PR comment                       |
| T3   | Build Site     | Yes (if docs/source changed) | Yes (after T2) | Builds the VitePress documentation site                                          |
| T4   | Deploy         | —                                               | Так                               | Deploys to AWS S3                                                                |

#### Pull Request Flow

When a PR modifies files in `docs/**` or the orchestrator workflow file itself, the workflow validates the changes:

```txt
PR Event
    │
    ▼
┌─────────────────────────────────────┐
│ T1: Detect Changes                  │
│  • Checks if src/msg/ROMFS changed  │
└─────────────────┬───────────────────┘
                  │
          ┌───────┴───────┐
          ▼               ▼
┌──────────────────┐ ┌─────────────────────────┐
│ T2: PR Metadata  │ │ T2: Link Check (~30s)   │
│ (conditional)    │ │  • Detects changed .md   │
│  • Builds SITL   │ │  • Runs filtered check   │
│  • Generates     │ │  • Posts PR comment      │
│    metadata      │ │  • Runs full check       │
│  • Builds        │ └────────────┬────────────┘
│    failsafe web  │              │
└────────┬─────────┘              │
         └───────────┬────────────┘
                     ▼
┌─────────────────────────────────────┐
│ T3: Build Site (~7-10 min)         │
│  (skipped if only workflow YAML     │
│   changed — no docs/source changes) │
│  • Builds VitePress site            │
│  • Verifies no build errors         │
└─────────────────┬───────────────────┘
                  │
                  ▼
                DONE
```

| Job                                    | Duration                | Опис                                                                                                                                                                                       |
| -------------------------------------- | ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **T1: Detect Changes** | ~10s    | Determines if metadata regeneration is needed                                                                                                                                              |
| **T2: PR Metadata**    | ~10-15m | Rebuilds PX4 SITL and regenerates all metadata (only if source files changed)                                                                                           |
| **T2: Link Check**     | ~30s    | Checks for broken links in changed markdown files and posts a sticky comment to the PR (skipped on fork PRs)                                                            |
| **T3: Build Site**     | ~7-10m  | Builds the VitePress site to verify there are no build errors. Skipped when only the workflow YAML changed (no docs or source changes). |

#### Push / Dispatch Flow (main/release branches)

When changes are pushed to `main` or `release/**` branches (or a `workflow_dispatch` is triggered), the workflow regenerates metadata, builds, and deploys.
Only `main` and `release/*` branches are accepted for deploy — other branches will fail with a clear error.

```txt
Push / Dispatch Event
    │
    ▼
┌─────────────────────────────────────┐
│ T2: Metadata Sync (~10-15 min)     │
│  • Builds px4_sitl_default          │
│  • Generates parameter/airframe/    │
│    module documentation             │
│  • Builds failsafe web simulator    │
│  • Formats with Prettier            │
│  • Auto-commits if changes detected │
│    (with [skip ci])                 │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ T3: Build Site (~7-10 min)         │
│  • Builds VitePress site            │
│  • Uploads build artifact           │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│ T4: Deploy (~3 min)                │
│  • Syncs to AWS S3                  │
│  • HTML: 60s cache                  │
│  • Assets: 24h immutable cache      │
└─────────────────────────────────────┘
```

| Job                                   | Duration                | Опис                                                                                                               |
| ------------------------------------- | ----------------------- | ------------------------------------------------------------------------------------------------------------------ |
| **T2: Metadata Sync** | ~10-15m | Rebuilds PX4 SITL, regenerates all metadata, formats with Prettier, auto-commits with `[skip ci]`                  |
| **T3: Build Site**    | ~7-10m  | Builds the VitePress documentation site                                                                            |
| **T4: Deploy**        | ~3m     | Syncs built site to AWS S3 (HTML: 60s cache, assets: 24h cache) |

Crowdin upload is handled by a separate workflow (see below).

#### Generated Metadata

The metadata regeneration job creates the following auto-generated documentation:

| Тип                | Output                                           | Опис                                                            |
| ------------------ | ------------------------------------------------ | --------------------------------------------------------------- |
| Параметри          | `docs/en/advanced_config/parameter_reference.md` | Complete parameter reference                                    |
| Планери            | `docs/en/airframes/airframe_reference.md`        | Airframe configurations                                         |
| Модулі             | `docs/en/modules/*.md`                           | Module documentation                                            |
| Messages           | `docs/en/msg_docs/*.md`                          | uORB message documentation                                      |
| uORB Graphs        | `docs/public/middleware/*.json`                  | Topic dependency graphs                                         |
| Failsafe Simulator | `docs/public/config/failsafe/*`                  | Interactive failsafe simulator (WebAssembly) |

:::warning
Do not manually edit the auto-generated files listed above. They are overwritten on every push to main.
:::

#### Path Triggers

The workflow triggers on different paths depending on the event:

**Push** (main/release branches):

| Path                     | Reason                                |
| ------------------------ | ------------------------------------- |
| `docs/**`                | Documentation source files            |
| `src/**`                 | Source code changes affect metadata   |
| `msg/**`                 | Message definitions affect metadata   |
| `ROMFS/**`               | ROMFS files affect metadata           |
| `Tools/module_config/**` | Module configuration affects metadata |

**Pull Request:**

| Path                                      | Reason                         |
| ----------------------------------------- | ------------------------------ |
| `docs/**`                                 | Documentation source files     |
| `.github/workflows/docs-orchestrator.yml` | Changes to the workflow itself |

Source-only changes on PRs are detected at runtime by the T1: Detect Changes job using [dorny/paths-filter](https://github.com/dorny/paths-filter), which conditionally triggers the T2: PR Metadata job.

### Crowdin Download Workflow

**Workflow file:** [`docs_crowdin_download.yml`](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/docs_crowdin_download.yml)

This scheduled workflow downloads completed translations from Crowdin and creates pull requests.

| Налаштування         | Значення                                                                                                         |
| -------------------- | ---------------------------------------------------------------------------------------------------------------- |
| **Schedule**         | Every Sunday at 00:00 UTC                                                                        |
| **Target Languages** | Korean (ko), Ukrainian (uk), Chinese Simplified (zh-CN) |

**Process:**

1. Downloads translations for each target language from Crowdin
2. Creates a separate PR for each language with new translations
3. PRs are labeled "Documentation" and assigned for review

### Caching Strategy

The workflows use caching to speed up builds:

| Cache                             | Розмір                | Ціль                                               |
| --------------------------------- | --------------------- | -------------------------------------------------- |
| ccache                            | 1GB                   | C++ compilation cache for SITL builds              |
| node_modules | ~26MB | Node.js dependencies for VitePress |

### Infrastructure

Jobs run on [runs-on](https://runs-on.com/) self-hosted runners with S3 cache:

| Job                                | Runner                                            |
| ---------------------------------- | ------------------------------------------------- |
| T1: Detect Changes | ubuntu-latest                                     |
| T2: PR Metadata    | 4 CPU (with px4-dev container) |
| T2: Metadata Sync  | 4 CPU (with px4-dev container) |
| T2: Link Check     | ubuntu-latest                                     |
| T3: Build Site     | 4 CPU                                             |
| T4: Deploy         | ubuntu-latest                                     |
