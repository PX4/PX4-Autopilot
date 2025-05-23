# GIT 示例

<a id="contributing_code"></a>

## 为 PX4 贡献代码

Adding a feature to PX4 follows a defined workflow. In order to share your contributions on PX4, you can follow this example. 为了在 px4 上分享您的贡献, 您可以遵循此示例。

- [Sign up](https://github.com/join) for github if you haven't already

- Fork the PX4-Autopilot repo (see [here](https://docs.github.com/en/get-started/quickstart/fork-a-repo))

- 将分支克隆到本地计算机

  ```sh
  cd ~/wherever/
  git clone https://github.com/<your git name>/PX4-Autopilot.git
  ```

- Go into the new directory, initialize and update the submodules, and add the original upstream Firmware

  ```sh
  cd PX4-Autopilot
  git submodule update --init --recursive
  git remote add upstream https://github.com/PX4/PX4-Autopilot.git
  ```

- You should have now two remote repositories: One repository is called `upstream` that points to PX4/PX4-Autopilot, and one repository `origin` that points to your forked copy of the PX4 repository.

- 这可以通过以下命令进行检查:

  ```sh
  git remote -v
  ```

- Make the changes that you want to add to the current main.

- 使用代表您的功能的有意义的名称创建一个新分支

  ```sh
  git checkout -b <your feature branch name>
  ```

  you can use the command `git branch` to make sure you're on the right branch.

- 通过添加相应的文件添加您希望成为提交的一部分的更改

  ```sh
  git add <file name>
  ```

  If you prefer having a GUI to add your files see [Gitk](https://git-scm.com/book/en/v2/Git-in-Other-Environments-Graphical-Interfaces) or [`git add -p`](http://nuclearsquid.com/writings/git-add/).

- 提交添加的文件, 并顺便记录一条有意义的消息, 解释您的更改

  ```sh
  git commit -m "<your commit message>"
  ```

  For a good commit message, please refer to the [Source Code Management](../contribute/code.md#commits-and-commit-messages) section.

- Some time might have passed and the [upstream main](https://github.com/PX4/PX4-Autopilot.git) has changed.
  PX4 prefers a linear commit history and uses [git rebase](https://git-scm.com/book/en/v2/Git-Branching-Rebasing).
  To include the newest changes from upstream in your local branch, switch to your main branch

  ```sh
  git checkout main
  ```

  Then pull the newest commits from upstream main

  ```sh
  git pull upstream main
  ```

  Now your local main is up to date.
  Switch back to your feature branch and rebase on your updated main

  ```sh
  git checkout <your feature branch name>
  git rebase main
  ```

- 现在, 您可以将本地提交推送到分支版本库

  ```sh
  git push origin <your feature branch name>
  ```

- You can verify that the push was successful by going to your forked repository in your browser: `https://github.com/<your git name>/PX4-Autopilot.git`

  There you should see the message that a new branch has been pushed to your forked repository.

- 现在是时候创建一个拉取请求 (PR) 了。
  On the right hand side of the "new branch message" (see one step before), you should see a green button saying "Compare & Create Pull Request".
  然后, 它应该列出你的更改，你必须添加一个有意义的标题 (在提交 PR 的情况下, 它通常是提交消息) 和消息 (<span style="color:orange">解释你做了这些更改的原因 </span>，
  Check [other pull requests](https://github.com/PX4/PX4-Autopilot/pulls) for comparison)

- You're done!
  You're done! Responsible members of PX4 will now have a look at your contribution and decide if they want to integrate it. Check if they have questions on your changes every once in a while.
  Check if they have questions on your changes every once in a while.

## Changing Source Trees

We recommend using PX4 `make` commands to switch between source code branches.
This saves you having to remember the commands to update submodules and clean up build artifacts (build files that are not removed will result in "untracked files" errors after the switch).

To switch between branches:

1. Clean up the current branch, de-initializing submodule and removing all build artifacts:

  ```sh
  make clean
  make distclean
  ```

2. Switch to a new branch or tag (here we first fetch the fictional branch "PR_test_branch" from the `upstream` remote):

  ```sh
  git fetch upstream PR_test_branch
  git checkout PR_test_branch
  ```

3. Get the submodules for the new branch:

  ```sh
  make submodulesclean
  ```

<!-- FYI: Cleaning commands in https://github.com/PX4/PX4-Autopilot/blob/main/Makefile#L494 -->

## 更新子模块

Specific PX4 point releases are made as tags of the [release branches](#get-a-release-branch), and are named using the format `v<release>`.
These are listed on Github here (or you can query all tags using `git tag -l`).

To get the source code for a _specific older release_ (tag):

1. Clone the PX4-Autopilot repo and navigate into _PX4-Autopilot_ directory:

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git
  cd PX4-Autopilot
  ```

  :::note

  You can reuse an existing repo rather than cloning a new one.
  In this case clean the build environment (see [changing source trees](#changing-source-trees)):

  ```sh
  make clean
  make distclean
  ```


:::

2. Checkout code for particular tag (e.g. for tag v1.13.0-beta2)

  ```sh
  git checkout v1.13.0-beta2
  ```

3. Update submodules:

  ```sh
  make submodulesclean
  ```

## Get a Release Branch

Releases branches are branched of `main`, and used to backport necessary changes from main into a release.
The branches are named using the format `release/<release_number>` (e.g. `release/v1.13`).
The are [listed here](https://github.com/PX4/PX4-Autopilot/branches/all?query=release).

To get a release branch:

- Clone the PX4-Autopilot repo and navigate into _PX4-Autopilot_ directory:

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git
  cd PX4-Autopilot
  ```

  :::note

  You can reuse an existing repo rather than cloning a new one.
  In this case clean the build environment (see [changing source trees](#changing-source-trees)):

  ```sh
  make clean
  make distclean
  ```


:::

- Fetch the desired release branch.
  For example, assuming you want the source for PX4 v1.14:

  ```sh
  git fetch origin release/1.14
  ```

- Checkout the code for the branch

  ```sh
  git checkout release/1.14
  ```

- Update submodules:

  ```sh
  make submodulesclean
  ```

## 更新子模块

有几种方法可以更新子模块。
Either you clone the repository or you go in the submodule directory and follow the same procedure as in [Contributing code to PX4](#contributing_code).

## 为子模块更新执行 PR

This is required after you have done a PR for a submodule X repository and the bug-fix / feature-add is in the current main of submodule X. Since the Firmware still points to a commit before your update, a submodule pull request is required such that the submodule used by the Firmware points to the newest commit.

```sh
cd Firmware
```

- 创建一个分支，描述子模块更新的 bug 修复/功能：

  ```sh
  git checkout -b pr-some-fix
  ```

- 进入子模块的子目录

  ```sh
  cd <path to submodule>
  ```

- PX4 submodule might not necessarily point to the newest commit. Therefore, first checkout master and pull the newest upstream code. Therefore, first checkout main and pull the newest upstream code.

  ```sh
  git checkout main
  git pull upstream main
  ```

- 回到 Firmware 目录，如往常一样添加、提交和上推更改。

  ```sh
  cd -
  git add <path to submodule>
  git commit -m "Update submodule to include ..."
  git push upstream pr-some-fix
  ```

## 查看拉取请求

You can test someone's pull request (changes are not yet merged) even if the branch to merge only exists on the fork from that person. Do the following 执行以下指令：:

```sh
git fetch upstream  pull/<PR ID>/head:<branch name>
```

`PR ID` is the number right next to the PR's title (without the #) and the `<branch name>` can also be found right below the `PR ID`, e.g. `<the other persons git name>:<branch name>`. 之后, 您可以看到新创建的分支在本地

```sh
git branch
```

然后切换到那个分支

```sh
git checkout <branch name>
```

## 常见错误

### 强制推送到分叉存储库

做完第一个 PR 后, 来自 PX4 社区的人将回顾你的更改。 在大多数情况下, 这意味着您必须根据评审来修复本地分支。 After changing the files locally, the feature branch needs to be rebased again with the most recent upstream/main. 但是, 在重新建立基础后, 不再可能将特征分支直接推送到分叉存储库, 而是需要使用强制推送:

```sh
git push --force-with-lease origin <your feature branch name>
```

### 重新建立合并冲突

If a conflict occurs during a `git rebase`, please refer to [this guide](https://docs.github.com/en/get-started/using-git/resolving-merge-conflicts-after-a-git-rebase).

### 拉取合并冲突

If a conflict occurs during a `git pull`, please refer to [this guide](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/#competing-line-change-merge-conflicts).

### Build error due to git tags out of date

The build error `Error: PX4 version too low, expected at least vx.x.x` occurs if git tags are out of date.

This can be solved by fetching the upstream repository tags:

```sh
git add -p](http://nuclearsquid.com/writings/git-add/). * Commit the added files with a meaningful message explaining your changes
```
