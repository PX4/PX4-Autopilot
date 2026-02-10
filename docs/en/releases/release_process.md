# Release Process

This page documents the PX4 release process for maintainers. It covers the steps from preparing a release candidate through to the final announcement.

## Overview

PX4 uses a branch-based release workflow:

- **`release/X.Y`** (e.g., `release/1.17`, `release/2.0`) - The active development branch for a specific release where changes are merged and tested
- **`stable`** - Points to the latest stable release tag; reset after each release
- **`beta`** - For flight testers to validate release candidates

Releases are tracked on the [PX4 Release Project Board](https://github.com/orgs/PX4/projects/45), which is reused and renamed for each release cycle.

### Release Tag Progression

Release branches go through the following tag stages:

| Stage | Example | Branch Status | What Gets Merged |
|-------|---------|---------------|------------------|
| **Alpha** | `v1.18.0-alpha1` | Open for fixes | Bug fixes and regression fixes only |
| **Beta** | `v1.18.0-beta1` | Open for fixes | Bug fixes and regression fixes only |
| **RC** | `v1.18.0-rc1` | Frozen | Nothing (unless veto vote passes) |
| **Stable** | `v1.18.0` | Open for fixes | Bug fixes and regression fixes (for point releases) |

::: info
New features are never merged to release branches. Once a release branch is created from `main`, only bug fixes and regression fixes are accepted. New features must target `main` and will be included in the next release cycle.
:::

## Starting a New Release Cycle

A new release cycle begins immediately after a stable release is published. This involves two votes during the [Weekly Community Q&A Call](../contribute/dev_call.md) when approving a stable release:

1. **Vote to publish the current release** - Approve the stable release for publication
2. **Vote on the next release name/number** - Decide the version number for the next release (e.g., v1.18 or v2.0)

### Create the Release Branch

Once the next release version is decided, create the new release branch from `main`:

```sh
# Ensure main is up to date
git checkout main
git pull

# Create the new release branch
git checkout -b release/1.18
git push origin release/1.18
```

::: warning
Once the release branch is created, it only accepts bug fixes and regression fixes. All new feature development continues on `main`.
:::

### Create the Alpha Tag

Immediately after creating the release branch, create the first alpha tag to mark the beginning of the release cycle:

```sh
# On the new release branch
git checkout release/1.18

# Create the signed alpha tag
git tag -s v1.18.0-alpha1 -m "v1.18.0-alpha1"
git push origin v1.18.0-alpha1
```

Subsequent alpha tags can be created as fixes are merged (e.g., `v1.18.0-alpha2`, `v1.18.0-alpha3`).

### Create Documentation Early

Create the release notes page as soon as the release branch is created:

- Create the new release notes file (e.g., `docs/en/releases/1.18.md`)
- Add the initial structure based on existing release notes
- Add the entry to `SUMMARY.md` and `releases/index.md`

::: tip
Community members are encouraged to help document changes early in the release cycle. As fixes are merged into the release branch, contributors can add them to the release notes. This distributes the documentation workload and ensures changes are documented while they're fresh.
:::

## Release Steps

### 1. Review and Merge PRs (Alpha Phase)

During the alpha phase, review pull requests targeting the release branch:

- Review PRs according to the project's [contribution guidelines](../contribute/code.md)
- Merge **bug fixes and regression fixes only**
- New features must target `main`, not the release branch
- Create additional alpha tags as fixes are merged (e.g., `v1.18.0-alpha2`)

### 2. Alpha Testing

The _Dronecode Test Team_ (provided by [Ascend Engineering](https://www.ascendengineer.com) — Dronecode Silver member) conducts initial flight testing of alpha builds using:

- The [test cards](../test_and_ci/test_flights.md) documented in the PX4 guide
- A hardware matrix covering supported flight controllers and airframes

### 3. Create Beta Tag

Once alpha testing is successful, create the first beta tag:

```sh
# On the release branch
git checkout release/1.18
git pull

# Create the signed beta tag
git tag -s v1.18.0-beta1 -m "v1.18.0-beta1"
git push origin v1.18.0-beta1
```

### 4. Beta Testing

The test team conducts more extensive testing of beta builds:

- Full test card coverage across the hardware matrix
- Focus on stability and regression testing
- Community testing is encouraged during this phase

During beta, continue to:

- Review and merge **bug fixes and regression fixes only**
- Create additional beta tags as fixes are merged (e.g., `v1.18.0-beta2`, `v1.18.0-beta3`)

### 5. Create Release Candidate Tag

Once beta testing is successful, create the first release candidate:

```sh
# On the release branch
git checkout release/1.18
git pull

# Create the signed RC tag
git tag -s v1.18.0-rc1 -m "v1.18.0-rc1"
git push origin v1.18.0-rc1
```

::: warning
The branch is now frozen. No PRs will be merged unless a veto vote passes (see below).
:::

::: tip
Tags must be GPG-signed. Ensure your GitHub account is [configured for GPG signing](https://docs.github.com/en/authentication/managing-commit-signature-verification). Only maintainers with registered GPG keys can create release tags.
:::

### 6. RC Testing and Branch Freeze

During RC testing, the release branch is frozen:

- No PRs are merged by default
- Testing focuses on final validation before stable release
- Any issues found are documented and evaluated

#### Veto Vote for Critical Fixes

If a critical issue is discovered during RC testing that must be fixed before release:

1. Raise the issue during the [Weekly Community Q&A Call](../contribute/dev_call.md)
2. Present the case for why the fix is critical
3. Maintainers vote on whether to merge the fix
4. If approved, merge the fix and create a new RC tag (e.g., `v1.18.0-rc2`)

### 7. Release Vote

The release vote takes place during the [Weekly Community Q&A Call](../contribute/dev_call.md):

- Present the release status and test results
- **Vote 1:** Core maintainers vote on whether to publish the release
- **Vote 2:** Decide the name/number of the next release version
- The call happens Wednesdays at 17:00 CET on [Discord](https://discord.gg/BDYmr6FA6Q)

### 8. Create and Push Release Tag

After a successful vote, create the final release tag:

```sh
# On the release branch
git checkout release/1.18
git pull

# Create the signed release tag
git tag -s v1.18.0 -m "v1.18.0"
git push origin v1.18.0
```

### 9. Update the Stable Branch

Reset the `stable` branch to point to the new release tag:

```sh
git checkout stable
git reset --hard v1.18.0
git push --force origin stable
```

::: warning
This is a force push that rewrites the stable branch history. Ensure you have the correct tag before executing.
:::

### 10. GitHub Release (Automated)

When a version tag is pushed, the [build_all_targets.yml](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/build_all_targets.yml) GitHub Actions workflow automatically:

1. Builds firmware for all supported targets
2. Uploads firmware artifacts to AWS S3 (`Firmware/vX.Y.Z/` and updates `Firmware/stable/`)
3. Creates a **draft release** on GitHub with all `.px4` firmware files attached

### 11. Publish GitHub Release

Edit the draft release on GitHub:

- Add a brief description of major changes
- Link to the full release notes in the documentation (e.g., `releases/1.18.md`)
- Publish the release

### 12. Finalize Documentation

Review and finalize the release notes that have been developed throughout the release cycle:

- Ensure all noteworthy changes, new features, and upgrade guides are documented
- Review for completeness and accuracy
- See existing [release notes](../releases/index.md) for the expected format

### 13. Announce the Release

Announce the release through official channels:

- [PX4 Discuss Forum](https://discuss.px4.io/)
- [PX4 Discord](https://discord.gg/dronecode)
- Social media (Twitter/X, LinkedIn)
- Dronecode newsletter

### 14. Start Next Release Cycle

Immediately after publishing, start the next release cycle:

- Create the new release branch from `main` (see [Create the Release Branch](#create-the-release-branch))
- Create the alpha tag for the new release
- Create the initial documentation for the next release
- Rename and prepare the project board (see [Project Board Management](#project-board-management))

## Point Releases

After a stable release is published, the release branch reopens for bug fixes and regression fixes to support point releases (e.g., `v1.18.1`).

For patch releases:

1. Merge bug fixes and regression fixes to the release branch
2. Create beta tags for testing if needed (e.g., `v1.18.1-beta1`)
3. Create RC tags for final validation (e.g., `v1.18.1-rc1`)
4. After testing and vote, tag the point release
5. Update stable branch and publish

## Project Board Management

The [PX4 Release Project Board](https://github.com/orgs/PX4/projects/45) is reused across release cycles to track issues and PRs for the current release.

### After a Release

Once a release is published:

1. **Rename the project board** to reflect the next release version (e.g., "v1.17 Release" → "v1.18 Release")
2. **Review remaining items**:
   - Close items that are no longer relevant
   - Move incomplete items that should carry over to the next release
   - Remove items that were descoped or deferred indefinitely
3. **Update the board description** if needed to reflect the new release target

## See Also

- [Source Code Management](../contribute/code.md) - Branching model and contribution guidelines
- [Test Flights](../test_and_ci/test_flights.md) - Flight test procedures and test cards
- [Weekly Community Q&A Call](../contribute/dev_call.md) - Where release votes happen
