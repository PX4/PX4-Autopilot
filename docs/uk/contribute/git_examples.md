# Приклад GIT

<a id="contributing_code"></a>

## Внесок коду у PX4

Додавання функції до PX4 слідує за визначеним робочим процесом. Щоб поділитися своїми внесками в PX4, ви можете слідувати цьому прикладу.

- [Sign up](https://github.com/join) for github if you haven't already

- Fork the PX4-Autopilot repo (see [here](https://docs.github.com/en/get-started/quickstart/fork-a-repo))

- Клонуйте ваш форкнутий репозиторій на локальний комп'ютер

  ```sh
  cd ~/wherever/
  git clone https://github.com/<your git name>/PX4-Autopilot.git
  ```

- Зайдіть в новий каталог, ініціалізуйте та оновіть підмодулі, і додайте оригінальні upstream PX4-Autopilot

  ```sh
  cd PX4-Autopilot
  git submodule update --init --recursive
  git remote add upstream https://github.com/PX4/PX4-Autopilot.git
  ```

- You should have now two remote repositories: One repository is called `upstream` that points to PX4/PX4-Autopilot, and one repository `origin` that points to your forked copy of the PX4 repository.

- Це може бути відмічено наступною командою:

  ```sh
  git remote -v
  ```

- Внесіть зміни, які ви хочете додати до основного.

- Створіть нову гілку з значущим ім'ям, яке репрезентує вашу функцію

  ```sh
  git checkout -b <your feature branch name>
  ```

  you can use the command `git branch` to make sure you're on the right branch.

- Додайте зміни, які ви хочете бути частиною коміту, додавши відповідні файли

  ```sh
  git add <file name>
  ```

  If you prefer having a GUI to add your files see [Gitk](https://git-scm.com/book/en/v2/Git-in-Other-Environments-Graphical-Interfaces) or [`git add -p`](http://nuclearsquid.com/writings/git-add/).

-

  ```sh
  git commit -m "<your commit message>"
  ```

  For a good commit message, please refer to the [Source Code Management](../contribute/code.md#commits-and-commit-messages) section.

- Some time might have passed and the [upstream main](https://github.com/PX4/PX4-Autopilot.git) has changed.
  PX4 prefers a linear commit history and uses [git rebase](https://git-scm.com/book/en/v2/Git-Branching-Rebasing).
  Щоб включити найновіші зміни з початкової версії до локальної гілки, перейдіть до головної гілки

  ```sh
  git checkout main
  ```

  Потім зтягнути найновіші коміти з upstream main

  ```sh
  git pull upstream main
  ```

  Тепер ваш локальний головний  репозиторій оновлений.
  Поверніться до своєї функціональної гілки та перебазуйте свою оновлену основну

  ```sh
  git checkout <your feature branch name>
  git rebase main
  ```

- Тепер ви можете відправляти свої локальні коміти у свій форкований репозиторій

  ```sh
  git push origin <your feature branch name>
  ```

- You can verify that the push was successful by going to your forked repository in your browser: `https://github.com/<your git name>/PX4-Autopilot.git`

  Там ви маєте побачити повідомлення, що нова гілка була відправлена у вашу репозиторію-форк.

- Тепер настав час створити запит на злиття (PR).
  On the right hand side of the "new branch message" (see one step before), you should see a green button saying "Compare & Create Pull Request".
  Потім у ньому має бути перелік ваших змін, і ви можете (обов’язково) додати змістовний заголовок (у випадку PR з одним комітом, зазвичай це повідомлення коміту) і повідомлення (<span style="color:orange">пояснити, що ви зробили, з якої причини</span>.
  Check [other pull requests](https://github.com/PX4/PX4-Autopilot/pulls) for comparison)

- Готово!
  Відповідальні учасники PX4 тепер побачать ваш внесок і вирішать, чи хочуть вони інтегрувати його.
  Періодично перевіряйте, чи є у них питання по вашим змінам.

##

We recommend using PX4 `make` commands to switch between source code branches.
Це позбавить вас від необхідності запам’ятовувати команди для оновлення підмодулів і очищення артефактів збірки (файли збірки, які не видалено, призведуть до помилок «невідстежуваних файлів» після перемикання).

Переключитися між гілками:

1. Очистити поточну гілку, деініціалізувати підмодуль та видалити всі артефакти збірки:

  ```sh
  make clean
  make distclean
  ```

2. Switch to a new branch or tag (here we first fetch the fictional branch "PR_test_branch" from the `upstream` remote):

  ```sh
  git fetch upstream PR_test_branch
  git checkout PR_test_branch
  ```

3. Отримати підмодулі для нової гілки:

  ```sh
  make submodulesclean
  ```

<!-- FYI: Cleaning commands in https://github.com/PX4/PX4-Autopilot/blob/main/Makefile#L494 -->

## Отримання конкретного  релізу

Specific PX4 point releases are made as tags of the [release branches](#get-a-release-branch), and are named using the format `v<release>`.
These are listed on Github here (or you can query all tags using `git tag -l`).

To get the source code for a _specific older release_ (tag):

1. Clone the PX4-Autopilot repo and navigate into _PX4-Autopilot_ directory:

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git
  cd PX4-Autopilot
  ```

  :::info

  Ви можете повторно використовувати існуючий репозиторій, а не клонувати новий.
  In this case clean the build environment (see [changing source trees](#changing-source-trees)):

  ```sh
  make clean
  make distclean
  ```


:::

2. Код оформлення замовлення для конкретного тегу (наприклад, для мітки v1.13.0-beta2)

  ```sh
  git checkout v1.13.0-beta2
  ```

3. Оновити підмодулі:

  ```sh
  make submodulesclean
  ```

## Щоб отримати гілку релізу

Releases branches are branched of `main`, and used to backport necessary changes from main into a release.
The branches are named using the format `release/<release_number>` (e.g. `release/v1.13`).
The are [listed here](https://github.com/PX4/PX4-Autopilot/branches/all?query=release).

Щоб отримати гілку релізу:

- Clone the PX4-Autopilot repo and navigate into _PX4-Autopilot_ directory:

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git
  cd PX4-Autopilot
  ```

  :::info

  Ви можете повторно використовувати існуючий репозиторій, а не клонувати новий.
  In this case clean the build environment (see [changing source trees](#changing-source-trees)):

  ```sh
  make clean
  make distclean
  ```


:::

- Отримати бажану гілку релізу.
  Наприклад, припустимо, що ви хочете джерело для PX4 v1.14:

  ```sh
  git fetch origin release/1.14
  ```

- Перевірте код гілки

  ```sh
  git checkout release/1.14
  ```

- Оновити підмодулі:

  ```sh
  make submodulesclean
  ```

## Оновити підмодулі

Існує кілька способів оновлення підмодуля.
Either you clone the repository or you go in the submodule directory and follow the same procedure as in [Contributing code to PX4](#contributing_code).

## Робити PR для оновлення підмодуля

Це необхідно після того, як ви зробили PR для репозиторію підмодуля X і виправлення помилки/додавання функції є в поточному основному основному підмодулю X. Оскільки прошивка все ще вказує на твердження до вашого оновлення, підмодуль запит на злиття обов'язковий таким чином, щоб підмодуль використовувався точками прошивки до найновішого коміту.

```sh
cd Firmware
```

- Створити нову гілку, яка описує виправлення / функцію оновлення підмодулів:

  ```sh
  git checkout -b pr-some-fix
  ```

- Перейти до підкаталогу підмодулів

  ```sh
  cd <path to submodule>
  ```

- Підмодуль PX4 не обов'язково вказує на найновіший коміт. Тому для того, для того, щоб оформити замовлення основного і витягнути найновіший код першоджерела.

  ```sh
  git checkout main
  git pull upstream main
  ```

- Поверніться до каталогу прошивки та, як зазвичай, додайте, зафіксуйте та надішліть зміни.

  ```sh
  cd -
  git add <path to submodule>
  git commit -m "Update submodule to include ..."
  git push upstream pr-some-fix
  ```

## Checkout pull requests

Ви можете перевірити чийсь запит на злиття (зміни ще не злиті), навіть якщо гілка зливання існує лише на форку з цієї людини. Зробіть наступне:

```sh
git fetch upstream  pull/<PR ID>/head:<branch name>
```

`PR ID` is the number right next to the PR's title (without the #) and the `<branch name>` can also be found right below the `PR ID`, e.g. `<the other persons git name>:<branch name>`. Після цього ви зможете побачити новостворену гілку локально за допомогою

```sh
git branch
```

Потім перейдіть на цю гілку

```sh
git checkout <branch name>
```

## Common pitfalls

### Force push to forked repository

Після завершення першого PR, люди зі спільноти PX4 переглянуть ваші зміни. У більшості випадків це означає, що ви повинні виправити свою місцеву гілку відповідно до розгляду. Після локальної зміни файлів гілку функцій потрібно знову базувати з останньою версією upstream/main. Однак після перебазування більше неможливо надіслати гілку функцій у ваш розгалужений репозиторій напряму, замість цього вам потрібно використати force push:

```sh
git push --force-with-lease origin <your feature branch name>
```

### Rebase merge conflicts

If a conflict occurs during a `git rebase`, please refer to [this guide](https://docs.github.com/en/get-started/using-git/resolving-merge-conflicts-after-a-git-rebase).

### Pull merge conflicts

If a conflict occurs during a `git pull`, please refer to [this guide](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/#competing-line-change-merge-conflicts).

### Помилка збірки через застарілі git теги

The build error `Error: PX4 version too low, expected at least vx.x.x` occurs if git tags are out of date.

Це можна вирішити шляхом отримання тегів репозиторію upstream :

```sh
git fetch upstream --tags
```
