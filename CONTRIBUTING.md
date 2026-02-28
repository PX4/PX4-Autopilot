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

The [developer guide](https://docs.px4.io/main/en/development/development.html) explains how to set up the development environment on Mac OS, Linux or Windows. Please take note of our [coding style](https://docs.px4.io/main/en/contribute/code.html) when editing files.

## Commit message convention

PX4 uses the `subsystem: description` format for all commit messages and PR titles.

### Format

```
subsystem: short description of the change
```

| Part | Rule |
|------|------|
| **subsystem** | The module, driver, board, or area of PX4 that the change affects |
| **`:`** + space | Required separator |
| **description** | What the change does, at least 5 characters, written in imperative form |

### Subsystem examples

Common subsystem prefixes used in PX4:

| Subsystem | Area |
|-----------|------|
| `ekf2` | Extended Kalman Filter (state estimation) |
| `mavlink` | MAVLink messaging protocol |
| `navigator` | Mission, RTL, Land, and other navigation modes |
| `sensors` | Sensor drivers and processing |
| `drivers` | Hardware drivers |
| `boards/px4_fmu-v6x` | Board-specific changes (use the board name) |
| `multicopter` | Multicopter-specific control and estimation |
| `fixedwing` | Fixed-wing-specific control and estimation |
| `vtol` | VTOL-specific logic |
| `actuators` | Mixer and actuator output |
| `battery` | Battery monitoring and estimation |
| `logger` | On-board logging |
| `param` | Parameter system |
| `simulation` | SITL, Gazebo, SIH |
| `CI` | Continuous integration and workflows |
| `docs` | Documentation |
| `build` | CMake, toolchain, build system |
| `uORB` | Inter-module messaging |

For changes spanning multiple subsystems, use the primary one affected.

### Good commit messages

```
ekf2: fix height fusion timeout
mavlink: add BATTERY_STATUS_V2 support
boards/px4_fmu-v6x: enable UAVCAN
navigator: fix RTL altitude calculation
CI: migrate to reusable workflows
docs: update EKF tuning guide
```

### Commits to avoid

These will be flagged by CI and should be squashed or reworded before merging:

```
fix                                    # too vague, no subsystem
update                                 # too vague, no subsystem
apply suggestions from code review     # squash into parent commit
do make format                         # squash into parent commit
WIP: trying something                  # not ready for main
oops                                   # not descriptive
```

### PR titles

The PR title follows the same `subsystem: description` format. This is especially important because the PR title becomes the commit message when a PR is squash-merged.

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

Since we care about safety, we will regularly ask you for test results. Best is to do a test flight (or bench test where it applies) and upload the log file from it (on the microSD card in the logs directory) to Google Drive or Dropbox and share the link.

## Push your changes

Push changes to your repo and send a [pull request](https://github.com/PX4/PX4-Autopilot/compare/).

Make sure to provide some testing feedback and if possible the link to a flight log file. Upload flight log files to [Flight Review](http://logs.px4.io) and link the resulting report.
