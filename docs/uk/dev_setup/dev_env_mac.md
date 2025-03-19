# Середовище розробки MacOS

Наступні інструкції для встановлення середовища розробки PX4 для macOS.
Це середовище може бути використане для збірки PX4 для:

- Pixhawk та іншого апаратного забезпечення на основі NuttX
- [Gazebo Classic Simulation](../sim_gazebo_classic/index.md)

:::tip
This setup is supported by the PX4 dev team.
To build other targets you will need to use a [different OS](../dev_setup/dev_env.md#supported-targets) (or an [unsupported development environment](../advanced/community_supported_dev_env.md)).
:::

## Відеоінструкція

<lite-youtube videoid="tMbMGiMs1cQ" title="Setting up your PX4 development environment on macOS"/>

## Базове налаштування

"Базове" налаштування macOS встановлює інструменти, необхідні для збірки прошивки та включає загальні інструменти, які будуть потрібні для встановлення/використання симуляторів.

### Налаштування середовища

:::details
Apple Silicon Macbook users!
If you have an Apple M1, M2 etc. Macbook, make sure to run the terminal as x86 by setting up an x86 terminal:

1. Locate the Terminal application within the Utilities folder (**Finder > Go menu > Utilities**)
2. Select _Terminal.app_ and right-click on it, then choose **Duplicate**.
3. Rename the duplicated Terminal app, e.g. to _x86 Terminal_
4. Now select the renamed _x86 Terminal_ app and right-click and choose \*_Get Info_
5. Check the box for **Open using Rosetta**, then close the window
6. Run the _x86 Terminal_ as usual, which will fully support the current PX4 toolchain

:::

Спочатку налаштуйте середовище

1. Enable more open files by appending the following line to the `~/.zshenv` file (creating it if necessary):

  ```sh
  echo ulimit -S -n 2048 >> ~/.zshenv
  ```

  ::: info
  If you don't do this, the build toolchain may report the error: `"LD: too many open files"`

:::

2. Enforce Python 3 by appending the following lines to `~/.zshenv`

  ```sh
  # Point pip3 to MacOS system python 3 pip
  alias pip3=/usr/bin/pip3
  ```

### Загальні інструменти

Для налаштування середовища з можливістю збірки для обладнання Pixhawk/NuttX (і встановлення загальних інструментів для використання симуляторів):

1. Install Homebrew by following these [installation instructions](https://brew.sh).

2. Виконайте ці команди в командній оболонці для встановлення загальних інструментів:

  ```sh
  brew tap PX4/px4
  brew install px4-dev
  ```

3. Встановіть необхідні пакети Python:

  ```sh
  # install required packages using pip3
  python3 -m pip install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema
  # if this fails with a permissions error, your Python install is in a system path - use this command instead:
  sudo -H python3 -m pip install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema
  ```

## Симуляція Gazebo Classic

To setup the environment for [Gazebo Classic](../sim_gazebo_classic/index.md) simulation:

1. Виконайте наступні команди в командній оболонці:

  ```sh
  brew unlink tbb
  sed -i.bak '/disable! date:/s/^/  /; /disable! date:/s/./#/3' $(brew --prefix)/Library/Taps/homebrew/homebrew-core/Formula/tbb@2020.rb
  brew install tbb@2020
  brew link tbb@2020
  ```

  ::: info
  September 2021: The commands above are a workaround to this bug: [PX4-Autopilot#17644](https://github.com/PX4/PX4-Autopilot/issues/17644).
  Вони можуть бути видалені після того, як вона буде виправлена (разом з цією нотаткою).

:::

2. Для встановлення симуляції SITL з Gazebo Classic:

  ```sh
  brew install --cask temurin
  brew install --cask xquartz
  brew install px4-sim-gazebo
  ```

3. Run the macOS setup script: `PX4-Autopilot/Tools/setup/macos.sh`
  The easiest way to do this is to clone the PX4 source, and then run the script from the directory, as shown:

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  cd PX4-Autopilot/Tools/setup
  sh macos.sh
  ```

## Наступні кроки

Після того, як ви закінчите налаштування інструментів командного рядка:

- Install [VSCode](../dev_setup/vscode.md) (if you prefer using an IDE to the command line).

- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)

  :::tip
  The _daily build_ includes development tools that are hidden in release builds.
  Вона також може надати доступ до нових функцій PX4, які ще не підтримуються в релізних збірках.

:::

- Continue to the [build instructions](../dev_setup/building_px4.md).
