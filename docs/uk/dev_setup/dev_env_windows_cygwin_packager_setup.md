# Середовище розробки Windows Cygwin (Інструкції з обслуговування)

:::warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env.md).
Це може працювати або не працювати з поточними версіями PX4.

Дивіться [Встановлення інструментарію](../dev_setup/dev_env.md) для інформації про середовища та інструменти, що підтримуються основною командою розробників.
:::

This topic explains how to construct and extend the development environment used for the no-longer-supported [Cygwin-based Windows Development Environment](../dev_setup/dev_env_windows_cygwin.md).

## Додаткова інформація

### Можливості / Проблеми

Відомо що наступні можливості працюють (версія 2.0):

- Building and running SITL with jMAVSim with significantly better performance than a VM (it generates a native windows binary **px4.exe**).
- Збірка та завантаження NuttX збірок (напр.: px4_fmu-v2 та px4_fmu-v4)
- Style check with _astyle_ (supports the command: `make format`)
- Автодоповнення в командному рядку
- Неінвазивний встановлювач! Встановлювач НЕ впливає на систему і глобальний шлях (лише змінює вибрану директорію установки, наприклад \*\*C:\PX4\*\* і використовує тимчасовий локальний шлях).
- Встановлювач підтримує оновлення до нової версії зі збереженням ваших особистих змін у теці інструментарію.

Відсутні:

- Симуляція: Gazebo та ROS не підтримуються.
- Підтримуються лише збірки NuttX і JMAVSim/SITL.
- [Known problems](https://github.com/orgs/PX4/projects/6) (Also use to report issues).

### Встановлення за допомогою скриптів оболонки

Ви також можете встановити середовище за допомогою скриптів в Github проєкті.

1. Make sure you have [Git for Windows](https://git-scm.com/download/win) installed.

2. Клонуйте репозиторій https://github.com/PX4/windows-toolchain туди куди ви хочете встановити інструментарій. Default location and naming is achieved by opening the `Git Bash` and executing:

   ```sh
   cd /c/
   git clone https://github.com/PX4/windows-toolchain PX4
   ```

3. If you want to install all components navigate to the freshly cloned folder and double click on the script `install-all-components.bat` located in the folder `toolchain`. If you only need certain components and want to safe Internet traffic and or disk space you can navigate to the different component folders like e.g. `toolchain\cygwin64` and click on the **install-XXX.bat** scripts to only fetch something specific.

4. Continue with [Getting Started](../dev_setup/dev_env_windows_cygwin.md#getting-started).

### Ручне встановлення (для розробників інструментарію)

Цей розділ описує як налаштувати інструментарій Cygwin вручну самостійно, із вказанням на відповідні скрипти з репозитарію установки за допомогою скриптів.
Результат повинен бути таким самим як при використанні скриптів, так і встановлювачі MSI.

:::info
The toolchain gets maintained and hence these instructions might not cover every detail of all the future changes.
:::

1. Create the _folders_: \*\*C:\PX4\*\*, \*\*C:\PX4\toolchain\*\* and \*\*C:\PX4\home\*\*

2. Download the _Cygwin installer_ file [setup-x86_64.exe](https://cygwin.com/setup-x86_64.exe) from the [official Cygwin website](https://cygwin.com/install.html)

3. Запустіть завантажений файл встановлювача

4. У майстрі оберіть встановлення в теку: \*\*C:\PX4\toolchain\cygwin64\*\*

5. Оберіть для встановлення стандартні основні пакети Cygwin і найновішу версію додаткових пакетів:

   - **Category:Packagename**
   - Devel:cmake (3.3.2 не дає попереджень про застарілість, 3.6.2 працює, але попереджає про це)
   - Devel:gcc-g++
   - Devel:gdb
   - Devel:git
   - Devel:make
   - Devel:ninja
   - Devel:patch
   - Editors:xxd
   - Editors:nano (якщо ви не професіонал із vim)
   - Python:python2
   - Python:python2-pip
   - Python:python2-numpy
   - Python:python2-jinja2
   - Python:python2-pyyaml
   - Python:python2-cerberus
   - Archive:unzip
   - Utils:astyle
   - Shells:bash-completion
   - Web:wget

   ::: info
   Do not select as many packages as possible which are not on this list, there are some which conflict and break the builds.

:::

   ::: info
   That's what [cygwin64/install-cygwin-px4.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-px4.bat) does.

:::

6. Write up or copy the **batch scripts** [`run-console.bat`](https://github.com/MaEtUgR/PX4Toolchain/blob/master/run-console.bat) and [`setup-environment.bat`](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat).

   Причиною запуску всіх інструментів розробки через підготовлений пакетні скрипти є те, що вони налаштовують початкову програму використовувати локальне, портативне середовище Cygwin всередині директорії інструментарію.
   This is done by always first calling the script [**setup-environment.bat**](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat) and the desired application like the console after that.

   The script [setup-environment.bat](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat) locally sets environmental variables for the workspace root directory `PX4_DIR`, all binary locations `PATH`, and the home directory of the unix environment `HOME`.

7. Add necessary **python packages** to your setup by opening the Cygwin toolchain console (double clicking **run-console.bat**) and executing

   ```sh
   pip2 install toml
   pip2 install pyserial
   pip2 install pyulog
   ```

   ::: info
   That's what [cygwin64/install-cygwin-python-packages.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/cygwin64/install-cygwin-python-packages.bat) does.

:::

8. Download the [**ARM GCC compiler**](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) as zip archive of the binaries for Windows and unpack the content to the folder `C:\PX4\toolchain\gcc-arm`.

   ::: info
   This is what the toolchain does in: [gcc-arm/install-gcc-arm.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/gcc-arm/install-gcc-arm.bat).

:::

9. Встановіть JDK:

   - Download Java 14 from [Oracle](https://www.oracle.com/java/technologies/javase-jdk14-downloads.html) or [AdoptOpenJDK](https://adoptopenjdk.net/).
   - Оскільки, на жаль, не існує портативного архіву, який містить безпосередньо бінарні файли вам потрібно встановити Java.
   - Find the binaries and move/copy them to **C:\PX4\toolchain\jdk**.
   - Ви можете видалити Java із вашої системи Windows, нам були потрібні лише бінарні файли для набору інструментів.

   ::: info
   This is what the toolchain does in: [jdk/install-jdk.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/jdk/install-jdk.bat).

:::

10. Download [**Apache Ant**](https://ant.apache.org/bindownload.cgi) as zip archive of the binaries for Windows and unpack the content to the folder `C:\PX4\toolchain\apache-ant`.

   :::tip
   Make sure you don't have an additional folder layer from the folder which is inside the downloaded archive.

:::

   ::: info
   This is what the toolchain does in: [apache-ant/install-apache-ant.bat](https://github.com/MaEtUgR/PX4Toolchain/blob/master/toolchain/apache-ant/install-apache-ant.bat).

:::

11. Download, build and add _genromfs_ to the path:

   - Clone the source code to the folder **C:\PX4\toolchain\genromfs\genromfs-src** with

      ```sh
      cd /c/toolchain/genromfs
      git clone https://github.com/chexum/genromfs.git genromfs-src
      ```

   - Скомпілюйте це:

      ```sh
      cd genromfs-src
      make all
      ```

   - Copy the resulting binary **genromfs.exe** one folder level out to: **C:\PX4\toolchain\genromfs**

12. Make sure all the binary folders of all the installed components are correctly listed in the `PATH` variable configured by [**setup-environment.bat**](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat).
