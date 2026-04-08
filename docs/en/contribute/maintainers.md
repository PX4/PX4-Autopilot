# Maintainer Role

Dronecode maintainers provide technical leadership for PX4 and for other ecosystem components such as MAVLink, MAVSDK, QGroundControl, and others. Some maintainers take responsibility for specific areas of the project, while others help across the project more broadly.
The maintainer role is defined by the community with help and supervision from the [Dronecode Foundation](https://dronecode.org/).

To find the most up-to-date maintainers list, see [`MAINTAINERS.md`](https://github.com/PX4/PX4-Autopilot/blob/main/MAINTAINERS.md) in the PX4-Autopilot repository.

## Maintainer Types

PX4 recognizes two types of maintainers. Both are full members of the maintainer team, have write access via the [`Dev Team`](https://github.com/orgs/PX4/teams/dev-team) GitHub team, and participate in maintainer decisions.

- **Code Owners** are responsible for a specific category of the project (for example, State Estimation, Multirotor, or CI). They have final say on changes to their area, are the primary reviewers for that code, and help shape its roadmap. This is the role described in detail below.
- **Reviewers** help maintain PX4 across the project without ownership of a specific category. They review, triage, and contribute wherever their interests and time allow. Reviewers have the same access and voting rights as Code Owners, but no on-call responsibility for any specific area of the codebase.

The Reviewer role is a good fit for contributors who want to help steward the project without committing to a specific component up front. A Reviewer may later become a Code Owner for a category by mutual agreement with the existing Code Owners of that category and sign-off from the maintainer team.

## Recruitment Process

If you would like to join the PX4 maintainers team or if you want to nominate someone else follow the steps below:

1. Read the [role description](#dronecode-maintainer-role-description), and make sure you understand the responsibilities of the role.
2. To nominate yourself, reach out to one of the maintainers (see the complete list in [`MAINTAINERS.md`](https://github.com/PX4/PX4-Autopilot/blob/main/MAINTAINERS.md)), and seek their sponsorship.
3. Express your interest in becoming a maintainer, and specify whether you are applying as a **Code Owner** (for a specific category) or as a **Reviewer** (helping across the project without a fixed category).
4. The sponsoring maintainer needs to bring this up for discussion in one of the [weekly developer calls](dev_call.md).
   The maintainer team will vote on the call to determine whether to accept you as a maintainer.

A Reviewer may later transition to a Code Owner role for a specific category. This requires agreement from the existing Code Owners of that category and sign-off from the maintainer team, following the same discussion and vote on the weekly developer call.

### Adding a new maintainer

Once the maintainer team has agreed to add a new maintainer, the change is landed via a pull request to [`MAINTAINERS.md`](https://github.com/PX4/PX4-Autopilot/blob/main/MAINTAINERS.md). The process is intentionally simple:

1. A current maintainer opens a PR adding the new maintainer to the appropriate table (**Code Owners** with a category, or **Reviewers**).
2. The PR must be approved by at least one other current maintainer.
3. If the new maintainer is being added as a Code Owner or sub-owner of a specific component, the existing Code Owner of that component must be among the approvers.

Once the PR is merged, the new maintainer proceeds through the [Onboarding Process](#onboarding-process) below.

## Onboarding Process

Once accepted every maintainers will go through the following process:

1. **Discord** server admin will grant you the `dev team` role, which gives you:
   1. Basic admin privileges on discord.
   2. Access to the private `#maintainers` channel for internal maintainer discussion.
2. You will be given access to the GitHub team: "[`Dev Team`](https://github.com/orgs/PX4/teams/dev-team)" which grants you:
   1. Permission to merge the PR of any of PX4 workspace repositories after it's approved
   2. Permission to trigger GitHub actions when a new contributor opens a PR.
   3. Permission to edit Issue/PR contents.
3. **Add your info to official PX4 channels**:
   1. Add your information to the internal Dronecode database of maintainers to keep you in sync.
   2. Community introduction to the new maintainer in the form of a forum post, which is promoted through ever growing official channels

## Dronecode Maintainer Role Description

The responsibilities and qualifications below describe the **Code Owner** role in detail. **Reviewers** share the same spirit of technical stewardship, community guidance, and participation in maintainer meetings, without being tied to a specific category. Reviewers are expected to review and triage across the project where their expertise and interest apply.

### Summary

Maintainers lead/manage the development of a **specific category (referred to as category below)** of any Open Source Projects hosted within the Dronecode Foundation, such as the PX4 Autopilot.

### Responsibilities (Code Owner)

1. Take charge of overseeing the development in their category.
2. Provide guidance/advice on community members in their category.
3. Review relevant pull requests and issues from the community on GitHub.
4. Coordinate with the maintainer group.
5. Keep regular attendance on [weekly meetings](dev_call.md).
6. Help create and maintain a roadmap for the project your represent.
7. Uphold the [Code of Conduct](https://github.com/Dronecode/foundation/blob/main/CODE-OF-CONDUCT.md) of our community.

### Responsibilities (Reviewer)

1. Review relevant pull requests and issues across the project where your expertise applies.
2. Help triage issues and guide community contributors.
3. Coordinate with the maintainer group.
4. Keep regular attendance on [weekly meetings](dev_call.md).
5. Uphold the [Code of Conduct](https://github.com/Dronecode/foundation/blob/main/CODE-OF-CONDUCT.md) of our community.

### Qualifications

1. Proven track record of valuable contributions.
2. Domain expertise in the category field (for Code Owners) or broad working knowledge of the project (for Reviewers).
3. Good overview of the project you are applying to.
4. You need to manage approval from your employer when relevant.

### Perks

1. **Official recognition** as the maintainer in Dronecode/PX4 website, documentation, community and social media.
2. **Github & Discord privileges** (described in the [onboarding process](#onboarding-process)).
3. Priority placement to the yearly **PX4 Developer Summit** scholarship which helps you with travel reimbursement.

### Tools we Provide to Assist You

Dronecode will provide the following tools to help you:

1. **Community survey**: If you need any insight into the community's opinion, we will send out social media posts, mailing lists, announcements in Discord server to get that answer for you.
2. **Workflow automation**: We will provide workflow for PR/Issue review & tagging process to help you.

And as always, don't hesitate to reach out if you need help with anything.
We are here for you!

### Point of Contact

Regarding questions about the maintainer role, please contact the maintainer team.
