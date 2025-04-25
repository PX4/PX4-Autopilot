# Contributing to Documentation

Contributions to the PX4 User Guide are very welcome; from simple fixes to spelling and grammar, through to the creation of whole new sections.

This topic explains how to make and test changes.
Towards the end there is a basic style guide.

:::tip Note
You will need a (free) [GitHub](https://github.com/) account to contribute to the guides.
:::

## Quick Changes in Github

Simple changes to _existing content_ can be made by clicking the **Edit on GitHub** link that appears at the bottom of every page (this opens the page on Github for editing).

![Vitepress: Edit Page button](../../assets/vuepress/vuepress_edit_page_on_github_link.png)

To edit an existing English page:

1. Open the page.
1. Click the **Edit on GitHub** link below the page content.
1. Make the desired change.
1. Below the Github page editor you'll be prompted to create a separate branch and then guided to submit a _pull request_.

The documentation team will review the request and either merge it or work with you to update it.

Note that you can only make changes to the English version directly in the source.
[Translations are handled in Crowdin](../contribute/translation.md).

## Changes using Git

More substantial changes, including adding new pages or adding/modifying images, aren't as easy to make (or properly test) on Github.

For these kinds of changes we suggest using the same approach as for _code_:

1. Use the _git_ toolchain to get the PX4 source code onto your local computer.
1. Modify the documentation as needed (add, change, delete).
1. _Test_ that it builds properly using Vitepress.
1. Create a branch for your changes and create a pull request (PR) to pull it back into the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git) repo.

The following explain how to get the source code, build locally (to test), and modify the code.

### Get Documentation Source Code

Documentation sources are in the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot/) repo, alongside all the other PX4 source code.
The sources are markdown files located the [/docs](https://github.com/PX4/PX4-Autopilot/tree/main/docs) subdirectory.
The English source files are in the [/docs/en/](https://github.com/PX4/PX4-Autopilot/tree/main/docs/en) subdirectory and can be edited directly.
[Translation](../contribute/translation.md) sources are in language specific subdirectories, such as `ko` for korean and `zh` for Chinese: these are edited via the Crowdin tool, and should not be edited directly.

::: tip
If you already have a clone of the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot/) you can ignore this section.
:::

To get the library(s) sources onto your local computer you will need to use the git toolchain.
The instructions below explain how to get git and use it on your local computer.

1. Download git for your computer from [https://git-scm.com/downloads](https://git-scm.com/downloads)
1. [Sign up](https://github.com/join) for Github if you haven't already
1. Create a copy (Fork) of the [PX4-Autopilot repo](https://github.com/PX4/PX4-Autopilot) on Github ([instructions here](https://docs.github.com/en/get-started/quickstart/fork-a-repo)).
1. Clone (copy) your forked repository to your local computer:

   ```sh
   cd ~/wherever/
   git clone https://github.com/<your git name>/PX4-Autopilot.git
   ```

   For example, to clone PX4 source fork for a user with Github account "john_citizen":

   ```sh
   git clone https://github.com/john_citizen/PX4-Autopilot.git
   ```

1. Navigate to your local repository:

   ```sh
   cd ~/wherever/PX4-Autopilot
   ```

1. Add a _remote_ called "upstream" to point to the "official" PX4 version of the library:

   ```sh
   git remote add upstream https://github.com/PX4/PX4-Autopilot.git
   ```

   :::tip
   A "remote" is a handle to a particular repository.
   The remote named _origin_ is created by default when you clone the repository, and points to _your fork_ of the guide.
   Above you create a new remote _upstream_ that points to the PX4 project version of the documents.
   :::

### Make/Push Documentation Changes

Within the repository you created above:

1. Bring your copy of the repository `main` branch up to date:

   ```sh
   git checkout main
   git fetch upstream main
   git pull upstream main
   ```

2. Create a new branch for your changes:

   ```sh
   git checkout -b <your_feature_branch_name>
   ```

   This creates a local branch on your computer named `your_feature_branch_name`.

3. Make changes to the documentation as needed (general guidance on this in following sections)
4. Once you are satisfied with your changes, you can add them to your local branch using a "commit":

   ```sh
   git add <file name>
   git commit -m "<your commit message>"
   ```

   For a good commit message, please refer to the [Source Code Management](../contribute/code.md#commits-and-commit-messages) section.

5. Push your local branch (including commits added to it) to your forked repository on Github.

   ```sh
   git push origin your_feature_branch_name
   ```

6. Go to your forked repository on Github in a web browser, e.g.: `https://github.com/<your git name>/PX4-Autopilot.git`.
   There you should see the message that a new branch has been pushed to your forked repository.
7. Create a pull request (PR):

   - On the right hand side of the "new branch message" (see one step before), you should see a green button saying "Compare & Create Pull Request".
     Press it.
   - A pull request template will be created.
     It will list your commits and you can (must) add a meaningful title (in case of a one commit PR, it's usually the commit message) and message (<span style="color:orange">explain what you did for what reason</span>.
     Check [other pull requests](https://github.com/PX4/PX4-Autopilot/pulls) for comparison).
   - Add the "Documentation" label.

8. You're done!

   Maintainers for the PX4 User Guide will now have a look at your contribution and decide if they want to integrate it.
   Check if they have questions on your changes every once in a while.

### Building the Library Locally

Build the library locally to test that any changes you have made have rendered properly:

1. Install the [Vitepress prerequisites](https://vitepress.dev/guide/getting-started#prerequisites):

   - [Nodejs 18+](https://nodejs.org/en)
   - [Yarn classic](https://classic.yarnpkg.com/en/docs/install)

1. Navigate to your local repository and the `/docs` subdirectory:

   ```sh
   cd ~/wherever/PX4-Autopilot/docs
   ```

1. Install dependencies (including Vitepress):

   ```sh
   yarn install
   ```

1. Preview and serve the library:

   ```sh
   yarn start
   ```

   - Once the development/preview server has built the library (less than a minute for the first time) it will show you the URL you can preview the site on.
     This will be something like: `http://localhost:5173/px4_user_guide/`.
   - Stop serving using **CTRL+C** in the terminal prompt.

1. Open previewed pages in your local editor:

   First specify a local text editor file using the `EDITOR` environment variable, before calling `yarn start` to preview the library.
   For example, on Windows command line you can enable VSCode as your default editor by entering:

   ```sh
   set EDITOR=code
   ```

   The **Open in your editor** link at the bottom of each page will then open the current page in the editor (this replaces the _Open in GitHub_ link).

1. You can build the library as it would be done for deployment:

   ```sh
   # Ubuntu
   yarn docs:build

   # Windows
   yarn docs:buildwin
   ```

:::tip
Use `yarn start` to preview changes _as you make them_ (documents are updated and served very quickly).
Before submitting a PR you should also build it using `yarn docs:build`, as this can highlight issues that are not visible when using `yarn start`.
:::

### Source Code Structure

The guide uses the [Vitepress](https://vitepress.dev/) toolchain.

In overview:

- Pages are written in separate files using markdown.
  - The syntax is almost the same as that used by the Github wiki.
  - Vitepress also supports some [markdown extensions](https://vitepress.dev/guide/markdown#markdown-extensions).
    We try and avoid using these, except for [tips, warning, etc.](https://vitepress.dev/guide/markdown#default-title).
    This might be revisited - there are some interesting options provided!
- This is a [multilingual](https://vitepress.dev/guide/i18n) book:
  - Pages for each language are stored in the folder named for the associated language code (e.g. "en" for English, "zh" for Chinese, "ko" for Korean).
  - Only edit the ENGLISH (`/en`) version of files.
    We use [Crowdin](../contribute/translation.md) to manage the translations.
- All pages must be in an appropriately named sub-folder of `/en` (e.g. this page is in folder `en/contribute/`).
  - This makes linking easier because other pages and images are always as the same relative levels
- The _structure_ of the book is defined in `SUMMARY.md`.

  - If you add a new page to the guide you must also add an entry to this file!

    :::tip
    This is not "standard vitepress" way to define the sidebar (the summary file is imported by [.vitepress/get_sidebar.js](https://github.com/PX4/PX4-user_guide/blob/main/.vitepress/get_sidebar.js)).
    :::

- Images must be stored in a sub folder of `/assets`.
  This is two folders down from content folders, so if you add an image you will reference it like:

  ```plain
  ![Image Description](../../assets/path_to_file/filename.jpg)
  ```

- A file named **package.json** defines any dependencies of the build.
- A web hook is used to track whenever files are merged into the master branch on this repository, causing the book to rebuild.

### Adding New Pages

When you add a new page you must also add it to `en/SUMMARY.md`!

## Style Guide

1. Files/file names

   - Put new markdown files in an appropriate sub-folder of `/en/`, such as `/en/contribute/`.
     Do not further nest folders.
   - Put new image files in an appropriate nested sub-folder of `/assets/`.
     Deeper nesting is allowed/encouraged.
   - Use descriptive names for folders and files.
     In particular, image filenames should describe what they contain (don't name as "image1.png")
   - Use lower case filenames and separate words using underscores (`_`).

2. Images

   - Use the smallest size and lowest resolution that makes the image still useful (this reduces download cost for users with poor bandwidth).
   - New images should be created in a sub-folder of `/assets/` (so they can be shared between translations).
   - SVG files are preferred for diagrams.
     PNG files are preferred over JPG for screenshots.

3. Content:

   - Use "style" (**bold**, _emphasis_, etc.) consistently and sparingly (as little as possible).
     - **Bold** for button presses and menu definitions.
     - _Emphasis_ for tool names such as _QGroundControl_ or _prettier_.
     - `code` for file paths, and code, parameter names that aren't linked, using tools in a command line, such as `prettier`.
   - Headings and page titles should use "First Letter Capitalisation".
   - The page title should be a first level heading (`#`).
     All other headings should be h2 (`##`) or lower.
   - Don't add any style to headings.
   - Don't translate the text indicating the name of an `info`, `tip` or `warning` declaration (e.g. `::: tip`) as this precise text is required to render the aside properly.
   - Break lines on sentences by preference.
     Don't break lines based on some arbitrary line length.
   - Format using _prettier_ (_VSCode_ is a has extensions can be used for this).

4. Videos:

   - Youtube videos can be added using the format `<lite-youtube videoid="<youtube-video-id>" title="your title"/>` (supported via the [https://www.npmjs.com/package/lite-youtube-embed](https://www.npmjs.com/package/lite-youtube-embed) custom element, which has other parameters you can pass).
     - Use instructional videos sparingly as they date badly, and are hard to maintain.
     - Cool videos of airframes in flight are always welcome.

## Where Do I Add Changes?

Add new files in folders that cover similar topics.
Then reference them in the sidebar (`/en/SUMMARY.md`) in line with the existing structure!

## Translations

For information about translation see: [Translation](../contribute/translation.md).

## Licence

All PX4/Dronecode documentation is free to use and modify under terms of the permissive [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) license.
