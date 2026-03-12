# Contributing to PX4-Autopilot

We follow the [GitHub flow](https://guides.github.com/introduction/flow/) development model.

## Fork the project, then clone your repo

First [fork and clone](https://help.github.com/articles/fork-a-repo) the project.

## Create a feature branch

Always branch off `main` for new features.

```
git checkout -b mydescriptivebranchname
```

## Edit and build the code

The [developer guide](https://docs.px4.io/main/en/development/development.html) explains how to set up the development environment on Mac OS, Linux or Windows.

### Coding standards

All C/C++ code must follow the [PX4 coding style](https://docs.px4.io/main/en/contribute/code.html). Formatting is enforced by [astyle](http://astyle.sourceforge.net/) in CI (`make check_format`). Code quality checks run via [clang-tidy](https://clang.llvm.org/extra/clang-tidy/). Pull requests that fail either check will not be merged.

Python code is checked with [mypy](https://mypy-lang.org/) and [flake8](https://flake8.pycqa.org/).

## Commit message convention

PX4 uses [conventional commits](https://www.conventionalcommits.org/) for all commit messages and PR titles.

### Format

```
type(scope): short description of the change
```

| Part | Rule |
|------|------|
| **type** | Category of change (see types table below) |
| **scope** | The module, driver, board, or area of PX4 affected |
| **`!`** (optional) | Append before `:` to mark a breaking change |
| **description** | What the change does, at least 5 characters, written in imperative form |

### Types

| Type | Description |
|------|-------------|
| `feat` | A new feature |
| `fix` | A bug fix |
| `docs` | Documentation only changes |
| `style` | Formatting, whitespace, no code change |
| `refactor` | Code change that neither fixes a bug nor adds a feature |
| `perf` | Performance improvement |
| `test` | Adding or correcting tests |
| `build` | Build system or external dependencies |
| `ci` | CI configuration files and scripts |
| `chore` | Other changes that don't modify src or test files |
| `revert` | Reverts a previous commit |

### Scopes

The scope identifies which part of PX4 is affected. Common scopes:

| Scope | Area |
|-------|------|
| `ekf2` | Extended Kalman Filter (state estimation) |
| `mavlink` | MAVLink messaging protocol |
| `commander` | Commander and mode management |
| `navigator` | Mission, RTL, Land, and other navigation modes |
| `sensors` | Sensor drivers and processing |
| `drivers` | Hardware drivers |
| `boards/px4_fmu-v6x` | Board-specific changes (use the board name) |
| `mc_att_control` | Multicopter attitude control |
| `mc_pos_control` | Multicopter position control |
| `fw_att_control` | Fixed-wing attitude control |
| `vtol` | VTOL-specific logic |
| `actuators` | Mixer and actuator output |
| `battery` | Battery monitoring and estimation |
| `logger` | On-board logging |
| `param` | Parameter system |
| `simulation` | SITL, Gazebo, SIH |
| `ci` | Continuous integration and workflows |
| `docs` | Documentation |
| `build` | CMake, toolchain, build system |
| `uorb` | Inter-module messaging |

For changes spanning multiple subsystems, use the primary one affected. Look at the directory path of the files you changed to find the right scope: `src/modules/ekf2/` uses `ekf2`, `src/drivers/imu/` uses `drivers/imu`, `.github/workflows/` uses `ci`.

### Breaking changes

Append `!` before the colon to indicate a breaking change:

```
feat(ekf2)!: remove deprecated height fusion API
```

### Good commit messages

```
feat(ekf2): add height fusion timeout
fix(mavlink): correct BATTERY_STATUS_V2 parsing
refactor(navigator): simplify RTL altitude logic
ci(workflows): migrate to reusable workflows
docs(ekf2): update tuning guide
feat(boards/px4_fmu-v6x)!: remove deprecated driver API
perf(mc_rate_control): reduce loop latency
```

### Commits to avoid

These will be flagged by CI and should be squashed or reworded before merging:

```
fix                                    # too vague, no type or scope
update                                 # too vague, no type or scope
ekf2: fix something                   # missing type prefix
apply suggestions from code review     # squash into parent commit
do make format                         # squash into parent commit
WIP: trying something                  # not ready for main
oops                                   # not descriptive
```

### PR titles

The PR title follows the same `type(scope): description` format. This is enforced by CI and is especially important because the PR title becomes the commit message when a PR is squash-merged.

### Merge policy

Commits should be atomic and independently revertable. Squash at reviewer discretion for obvious cases (multiple WIP commits, messy review-response history). When your commits are clean and logical, they will be preserved as individual commits on `main`.

### Cleaning up commits

If CI flags your commit messages, you can fix them with an interactive rebase:

```bash
# Squash all commits into one:
git rebase -i HEAD~N   # replace N with the number of commits
# mark all commits except the first as 'squash' or 'fixup'
# reword the remaining commit to follow the format
git push --force-with-lease

# Or reword specific commits:
git rebase -i HEAD~N
# mark the bad commits as 'reword'
git push --force-with-lease
```

## Test your changes

PX4 is safety-critical software. All contributions must include adequate testing where practical:

- **New features** must include unit tests and/or integration tests that exercise the new functionality, where practical. Hardware-dependent changes that cannot be tested in SITL should include bench test or flight test evidence.
- **Bug fixes** must include a regression test where practical. When automated testing is not feasible (hardware-specific issues, race conditions, etc.), provide a link to a flight log demonstrating the fix and the reproduction steps for the original bug.
- **Reviewers** will verify that tests or test evidence exist before approving a pull request.

### Types of tests

| Test type | When to use | How to run |
|-----------|-------------|------------|
| **Unit tests** (gtest) | Module-level logic, math, parsing | `make tests` |
| **SITL integration tests** (MAVSDK) | Flight behavior, failsafes, missions | `test/mavsdk_tests/` |
| **Bench tests / flight logs** | Hardware-dependent changes | Upload logs to [Flight Review](https://logs.px4.io) |

Since we care about safety, we will regularly ask you for test results. Best is to do a test flight (or bench test where it applies) and upload the log file from it (on the microSD card in the logs directory) to Google Drive or Dropbox and share the link.

## Push your changes

Push changes to your repo and send a [pull request](https://github.com/PX4/PX4-Autopilot/compare/).

Make sure to provide some testing feedback and if possible the link to a flight log file. Upload flight log files to [Flight Review](http://logs.px4.io) and link the resulting report.
