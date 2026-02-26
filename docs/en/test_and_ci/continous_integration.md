# PX4 Continuous Integration

PX4 uses GitHub Actions for continuous integration, with different workflows handling code builds, testing, and documentation.

## Code CI

PX4 builds and testing are performed using GitHub Actions with a waterfall/staged pipeline architecture to optimize costs and provide fast developer feedback.

### CI Architecture Overview

PX4 uses a **4-tier waterfall pipeline** that ensures expensive AWS-hosted integration tests only run after basic checks pass. This architecture prevents costly runs when simple issues like formatting errors are present.

#### Pipeline Structure

```
┌─────────────────────────────────────────────┐
│ TIER 1: Gate Checks (2-5 min)              │
│ • Format checks, shellcheck, Python linting │
│ • GitHub-hosted runners (free/cheap)        │
│ • fail-fast: stops on first failure         │
└─────────────────┬───────────────────────────┘
                  │ ALL PASS
                  ▼
┌─────────────────────────────────────────────┐
│ TIER 2: Builds, Analysis & Platform Checks │
│   (10-30 min)                               │
│ • SITL build + cache seed (PX4 + Gazebo)    │
│ • Unit tests, static analysis, EKF checks   │
│ • Ubuntu/macOS builds, ITCM, flash analysis │
│ • Failsafe web simulator (Emscripten)       │
│ • Mixed runners (AWS 4cpu + GitHub)          │
└─────────────────┬───────────────────────────┘
                  │ ALL PASS
                  ▼
┌─────────────────────────────────────────────┐
│ TIER 3: Integration Tests (30-45 min)      │
│ • SITL tests (Gazebo + MAVSDK, 20x speed)   │
│ • ROS2 integration, MAVROS tests             │
│ • ROS translation node (humble + jazzy)      │
│ • AWS Self-hosted 8cpu runners               │
└─────────────────┬───────────────────────────┘
                  │ ALL PASS
                  ▼
┌─────────────────────────────────────────────┐
│ TIER 4: Full Build Matrix (30-60 min)      │
│ • Build all board targets                   │
│ • AWS 8cpu runners (most expensive)         │
│ • Only on main/stable/beta/release/tags     │
└─────────────────────────────────────────────┘
```

### Active Workflows

#### Core CI

**[ci-orchestrator.yml](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/ci-orchestrator.yml)** - Main CI Pipeline
- Runs on all PRs and pushes
- Implements Tiers 1-4 with cascading dependencies
- Fails fast to save costs and provide quick feedback
- Uses concurrency control to cancel outdated runs

**[build_all_targets.yml](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/build_all_targets.yml)** - Full Board Build Matrix
- Triggered by orchestrator completion on PRs
- Runs independently on tagged releases
- Builds all board configurations
- Uploads artifacts to S3 and GitHub Releases

#### Infrastructure Workflows

- **dev_container.yml** - Docker container builds for development environment
- **fuzzing.yml** - Daily fuzzing tests for security

#### Maintenance Workflows

- **ekf_update_change_indicator.yml** - Track EKF functional changes
- **label.yml** - Auto-label PRs based on modified files
- **stale.yml** - Mark and close stale issues/PRs
- **sync_to_px4_msgs.yml** - Sync ROS message definitions to px4_msgs repo

### CI Tier Details

#### Tier 1: Gate Checks (2-5 minutes)

**Purpose:** Catch common errors quickly before spinning up expensive resources.

**Jobs:**
- Format checks (`check_format`)
- Newline checks (`check_newlines`)
- Shellcheck for bash scripts
- Python linting (mypy, flake8) for MAVSDK tests

**Runner:** GitHub-hosted `ubuntu-latest`

**Behavior:** `fail-fast: true` - stops all jobs immediately on first failure

#### Tier 2: Builds, Analysis & Platform Checks (10-30 minutes)

**Purpose:** Validate code compiles, passes unit tests, and builds on all platforms. This tier runs all build and analysis jobs in parallel to minimize wall-clock time.

**Jobs:**
- **Build SITL Cache Seed** - Builds `px4_sitl_default` and Gazebo Classic plugins, seeds the ccache for downstream SITL test jobs
- **Basic Tests** - Unit tests, module configuration validation, module documentation checks, and EKF functional change detection (PRs only — checks `src/modules/ekf2/test/change_indication` for uncommitted diff after running EKF tests)
- **Clang-tidy** static analysis (16cpu runner); incremental on PRs (changed files only), full scan on push to main/stable/beta/release
- **Clang-tidy PR Annotations** - Posts inline line-level review comments with one-click fix suggestions via `platisd/clang-tidy-pr-comments`; same-repo PRs only, informational (does not block merge)
- **Ubuntu builds** (22.04, 24.04) on AWS 4cpu runners
- **macOS build** on GitHub macOS runners
- **ITCM memory checks** for 4 NuttX targets (fmu-v5x, fmu-v6xrt, tropic variants)
- **Flash analysis** using bloaty for 2 targets (fmu-v5x, fmu-v6x), with PR comment
- **Failsafe web simulator** build (Emscripten, cached SDK)

**Runners:** Mixed (AWS 4cpu-linux-x64, 16cpu for clang-tidy, 2cpu for clang-tidy annotations, GitHub macos-latest)

**Behavior:** Only runs if Tier 1 passes completely; `fail-fast: false` to let all jobs attempt

#### Tier 3: Integration Tests (30-45 minutes)

**Purpose:** Run expensive simulation and integration tests.

**Jobs:**
- **SITL Tests** - Gazebo Classic simulation with MAVSDK test suite at 20x speed factor (iris, tailsitter, standard_vtol models)
- **ROS2 Integration** - Build and test ROS2 interface library
- **MAVROS Mission Tests** - Mission execution tests
- **MAVROS Offboard Tests** - Offboard control tests
- **ROS Translation Node** - Tests for humble and jazzy distributions

**Runners:** AWS Self-hosted 8cpu-linux-x64

**Behavior:** Only runs if all Tier 2 jobs pass; `fail-fast: false` to collect all test results

**Timeout:** 45 minutes per job

#### Tier 4: Full Build Matrix (30-60 minutes)

**Purpose:** Build all board targets for firmware distribution.

**Trigger:**
- Automatically after orchestrator succeeds (for PRs)
- Independently on main/stable/beta/release branches
- Independently on version tags (v*)

**Process:**
1. Scan and group board targets by architecture
2. Build targets in parallel on AWS 8cpu runners
3. Package and upload artifacts

**Artifacts:**
- Uploaded to S3 bucket (px4-travis) for QGroundControl
- Uploaded to GitHub Releases for version tags
- Draft releases created for manual review before publishing

**Runners:** AWS Self-hosted 8cpu-linux-x64 (most expensive)

### Runner Types

#### GitHub-Hosted Runners

Used for Tier 1 gate checks and macOS builds.

**Benefits:**
- Fast startup (5-10 seconds)
- Free for public repos (2,000 minutes/month)
- Low cost when paying (~$0.008/min for Linux)
- Reliable provisioning

**Used For:**
- Format/lint checks
- Python linting
- Shellcheck
- macOS builds

#### AWS Self-Hosted Runners (RunsOn)

Used for Tiers 2-4 builds and tests.

**Configuration:**
- `1cpu-linux-x64` - Utility jobs (flash analysis comment publishing)
- `2cpu-linux-x64` - Clang-tidy PR annotation posting
- `4cpu-linux-x64` - SITL cache seed, basic tests, EKF checks, platform builds, flash analysis, ITCM checks, failsafe sim
- `8cpu-linux-x64` - SITL integration tests, ROS integration tests, full build matrix
- `16cpu-linux-x64` - Clang-tidy static analysis
- `spot=false` - On-demand instances (can be changed to spot for 60-70% savings)

**Used For:**
- SITL simulation tests (8cpu for Gazebo physics at 20x speed)
- ROS integration tests (8cpu for parallel compilation of xrce-dds and ROS2 libraries)
- Full board compilation (8cpu)
- Platform builds and analysis (4cpu)

### Cost Optimization Features

#### 1. Cascading Dependencies

Each tier only executes if the previous tier passes completely. This prevents expensive AWS runners from spinning up when basic checks fail.

**Example:**
- Format error detected in 2 minutes (Tier 1)
- Pipeline stops immediately
- AWS runners never start
- Cost: ~$0.01 vs ~$5 for full run

#### 2. Fail-Fast Strategy

Tier 1 uses `fail-fast: true` to stop all parallel jobs on the first failure, providing immediate feedback to developers.

#### 3. Concurrency Control

All workflows use `cancel-in-progress: true` to automatically stop outdated runs when developers push new commits.

#### 4. Branch-Aware Execution

The full build matrix (Tier 4) only runs on important branches:
- main, stable, beta
- release/* branches
- Version tags (v*)

Feature branch PRs skip the expensive full build after validation in Tiers 1-3.

#### 5. Path-Based Filtering

Most CI workflows ignore changes to `docs/**` paths to avoid unnecessary builds when only documentation is modified.

### Developer Experience

#### Quick Feedback Loop

The waterfall architecture provides progressively detailed feedback:

| Issue Type | Detection Time | Tier | Cost Impact |
|------------|---------------|------|-------------|
| Format error | 2 minutes | Tier 1 | ~$0.01 |
| Unit test / build failure | 15 minutes | Tier 2 | ~$0.50 |
| Integration test failure | 45 minutes | Tier 3 | ~$2.00 |
| Board target failure | 90 minutes | Tier 4 | ~$5.00 |

#### Before/After Comparison

**Before (28 separate workflows):**
- All workflows start simultaneously on every PR
- Format error detected at 2 min, but expensive tests run for 45+ min anyway
- Total wasted compute: ~43 minutes of AWS 4cpu + 8cpu runners
- Cost per failed PR: ~$5-10

**After (Orchestrator + Build All Targets):**
- Format error detected at 2 min
- Pipeline stops immediately after Tier 1
- Total wasted compute: ~2 minutes of GitHub-hosted runners
- Cost per failed PR: ~$0.01
- **Savings: ~99% on early failures, ~40-60% overall**

### Caching Strategy

The CI orchestrator uses several caching mechanisms to avoid redundant work across jobs and runs.

#### ccache (C++ compilation cache)

ccache caches compiled object files so unchanged source files skip recompilation on subsequent runs.

**Cache keys follow this fallback pattern:**

```
ccache-{scope}-{branch}-{sha}     # exact match (never hits, since sha is unique)
ccache-{scope}-{branch}-          # same branch, most recent commit
ccache-{scope}-{base_branch}-     # base branch (e.g. main), for PR first runs
ccache-{scope}-                   # any branch, last resort
```

The exact-match key (`{sha}`) is used as the **save** key. Since GitHub Actions cache is immutable (write-once), each commit saves a new cache entry. The restore step falls through to the most recent cache from the same branch, or the base branch.

**Cache scopes and sizes:**

| Scope | Key prefix | Max size | Contents | Saved by |
|-------|-----------|----------|----------|----------|
| `ccache-sitl` | `ccache-sitl-` | 400M | PX4 SITL firmware + Gazebo Classic plugins | `build-sitl` (cache seed job) |
| `ccache-clang-tidy` | `ccache-clang-tidy-` | 250M | Clang-tidy analysis objects | `clang-tidy` |
| `ccache-ubuntu` | `ccache-ubuntu-{container}-` | 250M | Ubuntu build objects (per container version) | `ubuntu-builds` |
| `ccache-macos` | `ccache-macos-` | 400M | macOS build objects | `macos-build` |
| `ccache-ros-integration` | `ccache-ros-integration-` | 500M | PX4 + xrce-dds + Gazebo + ROS2 libraries | `ros-integration-tests` |
| `ccache-ros-translation-{ros}` | `ccache-ros-translation-{ros_version}-` | 250M | ROS translation node build objects (per ROS distro) | `ros-translation-node` |
| `ccache-flash-{target}-current` | `ccache-flash-{target}-current-` | 250M | Flash analysis objects for PR HEAD (per board target) | `flash-analysis` |
| `ccache-flash-{target}-baseline` | `ccache-flash-{target}-baseline-` | 250M | Flash analysis objects for baseline commit (per board target) | `flash-analysis` |
| `px4-ros2-ws` | `px4-ros2-ws-v1-galactic-{image}-{msg-hash}` | — | PX4 ROS 2 Interface Library workspace at `/opt/px4_ws` (keyed on msg hash) | `ros-integration-tests` |

**Cache seed pattern:** The `build-sitl` job acts as a cache seed for all downstream SITL-related jobs. It builds both `px4_sitl_default` and `sitl_gazebo-classic`, then saves the combined ccache. Downstream jobs (`basic-tests`, `ekf-functional-check`, `sitl-tests`) restore this cache using `actions/cache/restore` (read-only) and get near-100% hit rates without needing to save their own caches.

**ccache configuration (all jobs):**

| Setting | Value | Purpose |
|---------|-------|---------|
| `base_dir` | `${GITHUB_WORKSPACE}` | Normalize paths for cache portability |
| `compression` | `true` | Reduce cache storage size |
| `compression_level` | `6` | Balance compression ratio vs speed |
| `hash_dir` | `false` | Ignore directory paths in hash (portability) |
| `compiler_check` | `content` | Hash compiler binary content, not path/mtime |

#### Emscripten SDK cache

The failsafe web simulator job caches the Emscripten SDK directory to avoid re-cloning and installing it on every run.

| Key | Path | Contents |
|-----|------|----------|
| `emsdk-4.0.15` | `_emscripten_sdk` | Full emsdk installation (pinned to version 4.0.15) |

This is a simple version-pinned key. Updating the emsdk version in the workflow automatically invalidates the cache.

#### macOS Homebrew and pip caches

The macOS build job caches Homebrew packages and pip installations to avoid re-downloading dependencies.

#### Why separate cache scopes?

Different jobs use different compilers, flags, and build targets. Sharing a single ccache across all jobs would cause constant eviction as incompatible objects compete for space. Separate scopes ensure each job's cache stays warm with relevant objects.

#### CI Status

Check which tier failed by looking at the job names in the GitHub Actions UI:

- Tier 1 failure (T1 prefix) - Gate checks (format, lint) - Fix formatting/style issues
- Tier 2 failure (T2 prefix) - Builds/analysis (tests, static analysis, platform builds) - Fix compilation or test failures
- Tier 3 failure (T3 prefix) - Integration tests - Fix SITL/ROS test failures
- All tiers passed - Ready for merge (after approvals)

### Manual Workflow Triggers

All workflows support manual execution via GitHub Actions UI or CLI:

**Via GitHub UI:**
1. Go to Actions tab
2. Select the workflow
3. Click "Run workflow"
4. Choose branch and click "Run"

**Via GitHub CLI:**
```bash
gh workflow run ci-orchestrator.yml
gh workflow run build_all_targets.yml
```

This is useful for:
- Re-running specific workflows for debugging
- Testing workflow changes
- Manually triggering builds on branches

### Troubleshooting

#### Workflow Not Triggering

**Possible causes:**
- PR only modifies `docs/**` paths (ignored by most workflows)
- Orchestrator must complete successfully for `build_all_targets` to trigger (Tier 4)
- Workflow was canceled by a newer commit (concurrency control)

**Solution:**
- Check the Actions tab for workflow status
- Verify files modified are not in ignored paths
- Wait for orchestrator to complete before expecting Tier 4

#### Unexpected CI Failures

**Debugging steps:**
1. Check which tier failed by looking at job name prefixes (T1, T2, T3)
2. Review the specific failed job logs in that tier
3. Ensure your branch is rebased on latest main
4. Look for infrastructure issues (runner availability, network problems)
5. Try re-running failed jobs using "Re-run failed jobs" button

#### Common Issues

**Clang-tidy failures:**
- On PRs, only changed files are analyzed. Run `python3 Tools/ci/run-clang-tidy-pr.py origin/main` locally to reproduce the incremental check
- For a full scan (matches push-to-main behavior): `make clang-tidy`
- Fix any warnings or use `// NOLINT` comments for false positives
- Inline fix suggestions are posted as PR review comments by the `post-clang-tidy-comments` job (same-repo PRs only)

**SITL test timeouts:**
- Tests have a 45-minute timeout
- Check for deadlocks or infinite loops in simulation code
- Review MAVSDK test logs in failed artifacts

**AWS runner unavailable:**
- RunsOn provisions runners on-demand
- Rarely, provisioning can fail due to AWS capacity
- Re-run the workflow or wait a few minutes and try again

**Checkout errors in workflow_run:**
- The `build_all_targets` workflow (Tier 4) uses `workflow_run` trigger
- It automatically checks out the correct commit SHA
- If issues persist, check GitHub Actions permissions

### CI Best Practices for Contributors

#### Before Pushing

1. **Run format checks locally:**
   ```bash
   make check_format
   make shellcheck_all
   ```

2. **Run unit tests:**
   ```bash
   make tests
   ```

3. **Build your target configuration:**
   ```bash
   make px4_fmu-v5_default  # or your target
   ```

#### During PR Review

- Monitor CI status in the GitHub UI
- Address failures starting from the earliest tier
- Don't push new commits while CI is running if possible (cancels previous run)
- Use draft PRs to prevent expensive builds until ready

#### When CI Fails

1. Check which tier failed by looking at job name prefixes (T1, T2, T3)
2. Read the failed job logs carefully
3. Reproduce the issue locally if possible
4. Fix the root cause, not just the symptom
5. Consider if your changes affect other platforms/configurations

### Future Optimization Opportunities

#### 1. Enable Spot Instances

Change `spot=false` to `spot=true` for PR testing to reduce costs by 60-70%. This requires accepting occasional (~5%) provisioning failures.

#### 2. Path-Based Test Selection

Only run ROS tests if ROS-related files (`msg/**`, `src/modules/uxrce_dds_client/**`) are modified.

#### 3. Skip Tests on Draft PRs

Add `if: github.event.pull_request.draft == false` to expensive jobs, allowing draft PRs to skip CI entirely.

#### 4. Pre-built Container Images for ROS

Bake xrce-dds, ROS2 libraries, and Gazebo into updated container images to eliminate build-from-source overhead in integration test jobs (currently ~5-8 minutes per job).

### Migration History

This waterfall architecture was introduced to replace 28 independent workflows that all ran simultaneously. The migration:

- Reduced workflow files from 28 to 14
- Eliminated 1,057 lines of redundant YAML
- Decreased CI costs by an estimated 40-60%
- Improved developer feedback time for common errors
- Maintained full test coverage with smarter execution

The previous workflows that were consolidated:
- `checks.yml` -> Tiers 1 & 2
- `python_checks.yml` -> Tier 1
- `clang-tidy.yml` -> Tier 2
- `compile_macos.yml` -> Tier 2
- `compile_ubuntu.yml` -> Tier 2
- `sitl_tests.yml` -> Tier 3
- `ros_integration_tests.yml` -> Tier 3
- `mavros_mission_tests.yml` -> Tier 3
- `mavros_offboard_tests.yml` -> Tier 3
- `ros_translation_node.yml` -> Tier 3
- `itcm_check.yml` -> Tier 2
- `flash_analysis.yml` -> Tier 2
- `failsafe_sim.yml` -> Tier 2
- `nuttx_env_config.yml` -> Tier 2
- `ekf_functional_change_indicator.yml` -> Tier 2

The original 5-tier design was subsequently optimized to 4 tiers by merging platform builds (old Tier 3) into Tier 2, since they had no data dependency on basic test results and could run in parallel. This reduced wall-clock time by ~20 minutes on successful runs.

## Documentation CI

The documentation pipeline handles building, deploying, and translating the PX4 User Guide.
All documentation CI is consolidated into a single orchestrator workflow organized in tiers.

### Docs Orchestrator

**Workflow file:** [`docs-orchestrator.yml`](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/docs-orchestrator.yml)

This is the main documentation workflow. It runs on pull requests, pushes to `main` and `release/**` branches, and manual `workflow_dispatch` triggers, performing different jobs depending on the trigger event.
Jobs are organized in tiers, where each tier depends on the previous one completing successfully.

#### Tier Structure

| Tier | Job            | PR                           | Push / Dispatch | Description                                                   |
| ---- | -------------- | ---------------------------- | --------------- | ------------------------------------------------------------- |
| T1   | Detect Changes | Yes                          | -               | Checks if source code files changed (triggers metadata regen) |
| T2   | PR Metadata    | Yes (conditional)            | -               | Builds PX4 SITL and regenerates all auto-generated docs       |
| T2   | Metadata Sync  | -                            | Yes             | Builds PX4 SITL, regenerates metadata, auto-commits           |
| T2   | Link Check     | Yes                          | -               | Checks for broken links in changed files, posts PR comment    |
| T3   | Build Site     | Yes (if docs/source changed) | Yes (after T2)  | Builds the VitePress documentation site                       |
| T4   | Deploy         | -                            | Yes             | Deploys to AWS S3                                             |

#### Pull Request Flow

When a PR modifies files in `docs/**` or the orchestrator workflow file itself, the workflow validates the changes:

```txt
PR Event
    |
    v
┌─────────────────────────────────────┐
│ T1: Detect Changes                  │
│  • Checks if src/msg/ROMFS changed  │
└─────────────────┬───────────────────┘
                  │
          ┌───────┴───────┐
          v               v
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
                     v
┌─────────────────────────────────────┐
│ T3: Build Site (~7-10 min)         │
│  (skipped if only workflow YAML     │
│   changed - no docs/source changes) │
│  • Builds VitePress site            │
│  • Verifies no build errors         │
└─────────────────┬───────────────────┘
                  │
                  v
                DONE
```

| Job                    | Duration | Description                                                                                                                             |
| ---------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| **T1: Detect Changes** | ~10s     | Determines if metadata regeneration is needed                                                                                           |
| **T2: PR Metadata**    | ~10-15m  | Rebuilds PX4 SITL and regenerates all metadata (only if source files changed)                                                           |
| **T2: Link Check**     | ~30s     | Checks for broken links in changed markdown files and posts a sticky comment to the PR (skipped on fork PRs)                            |
| **T3: Build Site**     | ~7-10m   | Builds the VitePress site to verify there are no build errors. Skipped when only the workflow YAML changed (no docs or source changes). |

#### Push / Dispatch Flow (main/release branches)

When changes are pushed to `main` or `release/**` branches (or a `workflow_dispatch` is triggered), the workflow regenerates metadata, builds, and deploys.
Only `main` and `release/*` branches are accepted for deploy -- other branches will fail with a clear error.

```txt
Push / Dispatch Event
    |
    v
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
                  v
┌─────────────────────────────────────┐
│ T3: Build Site (~7-10 min)         │
│  • Builds VitePress site            │
│  • Uploads build artifact           │
└─────────────────┬───────────────────┘
                  │
                  v
┌─────────────────────────────────────┐
│ T4: Deploy (~3 min)                │
│  • Syncs to AWS S3                  │
│  • HTML: 60s cache                  │
│  • Assets: 24h immutable cache      │
└─────────────────────────────────────┘
```

| Job                   | Duration | Description                                                                                       |
| --------------------- | -------- | ------------------------------------------------------------------------------------------------- |
| **T2: Metadata Sync** | ~10-15m  | Rebuilds PX4 SITL, regenerates all metadata, formats with Prettier, auto-commits with `[skip ci]` |
| **T3: Build Site**    | ~7-10m   | Builds the VitePress documentation site                                                           |
| **T4: Deploy**        | ~3m      | Syncs built site to AWS S3 (HTML: 60s cache, assets: 24h cache)                                   |

Crowdin upload is handled by a separate workflow (see below).

#### Generated Metadata

The metadata regeneration job creates the following auto-generated documentation:

| Type               | Output                                           | Description                                  |
| ------------------ | ------------------------------------------------ | -------------------------------------------- |
| Parameters         | `docs/en/advanced_config/parameter_reference.md` | Complete parameter reference                 |
| Airframes          | `docs/en/airframes/airframe_reference.md`        | Airframe configurations                      |
| Modules            | `docs/en/modules/*.md`                           | Module documentation                         |
| Messages           | `docs/en/msg_docs/*.md`                          | uORB message documentation                   |
| uORB Graphs        | `docs/public/middleware/*.json`                  | Topic dependency graphs                      |
| Failsafe Simulator | `docs/public/config/failsafe/*`                  | Interactive failsafe simulator (WebAssembly) |

::: warning
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

| Setting              | Value                                                   |
| -------------------- | ------------------------------------------------------- |
| **Schedule**         | Every Sunday at 00:00 UTC                               |
| **Target Languages** | Korean (ko), Ukrainian (uk), Chinese Simplified (zh-CN) |

**Process:**

1. Downloads translations for each target language from Crowdin
2. Creates a separate PR for each language with new translations
3. PRs are labeled "Documentation" and assigned for review

### Caching Strategy

The workflows use caching to speed up builds:

| Cache        | Size  | Purpose                               |
| ------------ | ----- | ------------------------------------- |
| ccache       | 1GB   | C++ compilation cache for SITL builds |
| node_modules | ~26MB | Node.js dependencies for VitePress    |

### Infrastructure

Jobs run on [runs-on](https://runs-on.com/) self-hosted runners with S3 cache:

| Job                | Runner                         |
| ------------------ | ------------------------------ |
| T1: Detect Changes | ubuntu-latest                  |
| T2: PR Metadata    | 4 CPU (with px4-dev container) |
| T2: Metadata Sync  | 4 CPU (with px4-dev container) |
| T2: Link Check     | ubuntu-latest                  |
| T3: Build Site     | 4 CPU                          |
| T4: Deploy         | ubuntu-latest                  |

## Contact

For CI-related questions or issues:
- GitHub Issues: Tag the CI maintainer
- Slack: #infrastructure channel
- Dev Call: Bring up during weekly meeting

All workflows can be found in [.github/workflows/](https://github.com/PX4/PX4-Autopilot/tree/main/.github/workflows).
