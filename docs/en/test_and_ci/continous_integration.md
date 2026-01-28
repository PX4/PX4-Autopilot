# PX4 Continuous Integration

PX4 builds and testing are performed using GitHub Actions with a waterfall/staged pipeline architecture to optimize costs and provide fast developer feedback.

## CI Architecture Overview

PX4 uses a **5-tier waterfall pipeline** that ensures expensive AWS-hosted integration tests only run after basic checks pass. This architecture prevents costly runs when simple issues like formatting errors are present.

### Pipeline Structure

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
│ TIER 2: Basic Builds (10-15 min)           │
│ • Unit tests, static analysis, EKF checks   │
│ • GitHub-hosted runners                     │
└─────────────────┬───────────────────────────┘
                  │ ALL PASS
                  ▼
┌─────────────────────────────────────────────┐
│ TIER 3: Platform Builds (15-30 min)        │
│ • Ubuntu/MacOS builds, hardware checks      │
│ • Mixed runners (AWS + GitHub)              │
└─────────────────┬───────────────────────────┘
                  │ ALL PASS
                  ▼
┌─────────────────────────────────────────────┐
│ TIER 4: Integration Tests (30-45 min)      │
│ • SITL, ROS2, MAVROS tests                  │
│ • AWS Self-hosted runners (expensive)       │
└─────────────────┬───────────────────────────┘
                  │ ALL PASS
                  ▼
┌─────────────────────────────────────────────┐
│ TIER 5: Full Build Matrix (60+ min)        │
│ • Build all board targets                   │
│ • AWS 8cpu runners (most expensive)         │
│ • Only on main/stable/beta/release/tags     │
└─────────────────────────────────────────────┘
```

## Active Workflows

### Core CI

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

### Infrastructure Workflows

- **dev_container.yml** - Docker container builds for development environment
- **fuzzing.yml** - Daily fuzzing tests for security

### Documentation Workflows

- **docs_deploy.yml** - Deploy documentation to GitHub Pages
- **docs_deploy_aws.yml** - Deploy documentation to AWS S3
- **docs_flaw_checker.yml** - Check documentation for broken links
- **docs_pr_comment.yml** - Post documentation check results to PRs
- **docs_crowdin_download.yml** - Download translations from Crowdin
- **docs_crowdin_upload.yml** - Upload English docs to Crowdin

### Maintenance Workflows

- **ekf_update_change_indicator.yml** - Track EKF functional changes
- **label.yml** - Auto-label PRs based on modified files
- **stale.yml** - Mark and close stale issues/PRs
- **sync_to_px4_msgs.yml** - Sync ROS message definitions to px4_msgs repo

## CI Tier Details

### Tier 1: Gate Checks (2-5 minutes)

**Purpose:** Catch common errors quickly before spinning up expensive resources.

**Jobs:**
- Format checks (`check_format`)
- Newline checks (`check_newlines`)
- Shellcheck for bash scripts
- Python linting (mypy, flake8) for MAVSDK tests

**Runner:** GitHub-hosted `ubuntu-latest`

**Behavior:** `fail-fast: true` - stops all jobs immediately on first failure

### Tier 2: Basic Builds (10-15 minutes)

**Purpose:** Validate code compiles and passes unit tests.

**Jobs:**
- Unit tests (`tests`)
- Module configuration validation
- Module documentation checks
- Clang-tidy static analysis
- EKF functional change detection (PRs only)
- NuttX environment configuration

**Runner:** GitHub-hosted `ubuntu-latest`

**Behavior:** Only runs if Tier 1 passes completely

### Tier 3: Platform Builds (15-30 minutes)

**Purpose:** Verify builds on different platforms and perform hardware-specific checks.

**Jobs:**
- Ubuntu builds (22.04, 24.04) on AWS 4cpu runners
- MacOS builds (fmu-v5, sitl) on GitHub macOS runners
- ITCM memory checks for 4 targets (fmu-v5x, fmu-v6xrt, tropic variants)
- Flash analysis using bloaty for 2 targets (fmu-v5x, fmu-v6x)
- Failsafe web simulator build (Emscripten)

**Runners:** Mixed (AWS 4cpu-linux-x64, GitHub macos-latest)

**Behavior:** Only runs if Tier 2 passes; `fail-fast: false` to let all platforms attempt

### Tier 4: Integration Tests (30-45 minutes)

**Purpose:** Run expensive simulation and integration tests.

**Jobs:**
- **SITL Tests** - Gazebo simulation with MAVSDK test suite (iris model for PRs)
- **ROS2 Integration** - Build and test ROS2 interface library with Galactic
- **MAVROS Mission Tests** - Mission execution tests with ROS1 Noetic
- **MAVROS Offboard Tests** - Offboard control tests
- **ROS Translation Node** - Tests for humble and jazzy distributions

**Runners:** AWS Self-hosted 4cpu-linux-x64

**Behavior:** Only runs if Tier 3 passes; `fail-fast: false` to collect all test results

**Timeout:** 45 minutes per job

### Tier 5: Full Build Matrix (60+ minutes)

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

## Runner Types

### GitHub-Hosted Runners

Used for Tiers 1-2 and MacOS builds.

**Benefits:**
- Fast startup (5-10 seconds)
- Free for public repos (2,000 minutes/month)
- Low cost when paying (~$0.008/min for Linux)
- Reliable provisioning

**Used For:**
- Format/lint checks
- Unit tests
- Static analysis
- MacOS builds

### AWS Self-Hosted Runners (RunsOn)

Used for Tiers 3-4 and all Tier 5 builds.

**Configuration:**
- `1cpu-linux-x64` - Utility jobs (branch detection, artifact management)
- `4cpu-linux-x64` - Integration tests and platform builds
- `8cpu-linux-x64` - Full build matrix (parallel compilation)
- `spot=false` - On-demand instances (can be changed to spot for 60-70% savings)

**Used For:**
- SITL simulation tests
- ROS integration tests
- Full board compilation
- Resource-intensive builds

## Cost Optimization Features

### 1. Cascading Dependencies

Each tier only executes if the previous tier passes completely. This prevents expensive AWS runners from spinning up when basic checks fail.

**Example:**
- Format error detected in 2 minutes (Tier 1)
- Pipeline stops immediately
- AWS runners never start
- Cost: ~$0.01 vs ~$5 for full run

### 2. Fail-Fast Strategy

Tier 1 uses `fail-fast: true` to stop all parallel jobs on the first failure, providing immediate feedback to developers.

### 3. Concurrency Control

All workflows use `cancel-in-progress: true` to automatically stop outdated runs when developers push new commits.

### 4. Branch-Aware Execution

The full build matrix (Tier 5) only runs on important branches:
- main, stable, beta
- release/* branches
- Version tags (v*)

Feature branch PRs skip the expensive full build after validation in Tiers 1-4.

### 5. Path-Based Filtering

Most CI workflows ignore changes to `docs/**` paths to avoid unnecessary builds when only documentation is modified.

## Developer Experience

### Quick Feedback Loop

The waterfall architecture provides progressively detailed feedback:

| Issue Type | Detection Time | Tier | Cost Impact |
|------------|---------------|------|-------------|
| Format error | 2 minutes | Tier 1 | ~$0.01 |
| Unit test failure | 10 minutes | Tier 2 | ~$0.10 |
| Platform build issue | 25 minutes | Tier 3 | ~$0.50 |
| Integration test failure | 45 minutes | Tier 4 | ~$2.00 |
| Board target failure | 90 minutes | Tier 5 | ~$5.00 |

### Before/After Comparison

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

### CI Status Summary

The orchestrator includes a summary job that reports which tier failed:

- ❌ **Tier 1 failure** - Gate checks (format, lint) - Fix formatting/style issues
- ❌ **Tier 2 failure** - Basic builds (tests, static analysis) - Fix compilation or test failures
- ❌ **Tier 3 failure** - Platform builds - Fix platform-specific issues
- ❌ **Tier 4 failure** - Integration tests - Fix SITL/ROS test failures
- ✅ **All tiers passed** - Ready for merge (after approvals)

## Manual Workflow Triggers

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

## Troubleshooting

### Workflow Not Triggering

**Possible causes:**
- PR only modifies `docs/**` paths (ignored by most workflows)
- Orchestrator must complete successfully for `build_all_targets` to trigger
- Workflow was canceled by a newer commit (concurrency control)

**Solution:**
- Check the Actions tab for workflow status
- Verify files modified are not in ignored paths
- Wait for orchestrator to complete before expecting Tier 5

### Unexpected CI Failures

**Debugging steps:**
1. Check the CI Summary job to identify which tier failed
2. Review the specific failed job logs in that tier
3. Ensure your branch is rebased on latest main
4. Look for infrastructure issues (runner availability, network problems)
5. Try re-running failed jobs using "Re-run failed jobs" button

### Common Issues

**Clang-tidy failures:**
- Run `make clang-tidy` locally to reproduce
- Fix any warnings or use `// NOLINT` comments for false positives

**SITL test timeouts:**
- Tests have a 45-minute timeout
- Check for deadlocks or infinite loops in simulation code
- Review MAVSDK test logs in failed artifacts

**AWS runner unavailable:**
- RunsOn provisions runners on-demand
- Rarely, provisioning can fail due to AWS capacity
- Re-run the workflow or wait a few minutes and try again

**Checkout errors in workflow_run:**
- The `build_all_targets` workflow uses `workflow_run` trigger
- It automatically checks out the correct commit SHA
- If issues persist, check GitHub Actions permissions

## CI Best Practices for Contributors

### Before Pushing

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

### During PR Review

- Monitor CI status in the GitHub UI
- Address failures starting from the earliest tier
- Don't push new commits while CI is running if possible (cancels previous run)
- Use draft PRs to prevent expensive builds until ready

### When CI Fails

1. Check which tier failed (CI Summary job)
2. Read the failed job logs carefully
3. Reproduce the issue locally if possible
4. Fix the root cause, not just the symptom
5. Consider if your changes affect other platforms/configurations

## Future Optimization Opportunities

The following optimizations are planned or under consideration:

### 1. Enable Spot Instances

Change `spot=false` to `spot=true` for PR testing to reduce costs by 60-70%. This requires accepting occasional (~5%) provisioning failures.

### 2. Reduce Integration Test Matrix

Currently PRs run only the `iris` SITL model. Additional models (tailsitter, standard_vtol) could be restricted to main branch only.

### 3. Path-Based Test Selection

Only run ROS tests if ROS-related files (`msg/**`, `src/modules/uxrce_dds_client/**`) are modified.

### 4. Skip Tests on Draft PRs

Add `if: github.event.pull_request.draft == false` to expensive jobs, allowing draft PRs to skip CI entirely.

### 5. Coverage Analysis on Main Only

Move coverage builds (`tests_coverage`, Coverage SITL) to main-only execution, reducing PR runtime.

## Migration History

This waterfall architecture was introduced to replace 28 independent workflows that all ran simultaneously. The migration:

- Reduced workflow files from 28 to 14
- Eliminated 1,057 lines of redundant YAML
- Decreased CI costs by an estimated 40-60%
- Improved developer feedback time for common errors
- Maintained full test coverage with smarter execution

The previous workflows that were consolidated:
- `checks.yml` → Tiers 1 & 2
- `python_checks.yml` → Tier 1
- `clang-tidy.yml` → Tier 2
- `compile_macos.yml` → Tier 3
- `compile_ubuntu.yml` → Tier 3
- `sitl_tests.yml` → Tier 4
- `ros_integration_tests.yml` → Tier 4
- `mavros_mission_tests.yml` → Tier 4
- `mavros_offboard_tests.yml` → Tier 4
- `ros_translation_node.yml` → Tier 4
- `itcm_check.yml` → Tier 3
- `flash_analysis.yml` → Tier 3
- `failsafe_sim.yml` → Tier 3
- `nuttx_env_config.yml` → Tier 2
- `ekf_functional_change_indicator.yml` → Tier 2

All workflows can be found in [.github/workflows/](https://github.com/PX4/PX4-Autopilot/tree/main/.github/workflows).

## Contact

For CI-related questions or issues:
- GitHub Issues: Tag the CI maintainer
- Slack: #infrastructure channel
- Dev Call: Bring up during weekly meeting
