# Windows Cygwin Development Environment (Maintenance Instructions)

:::warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

This topic explains how to construct and extend the development environment used for the no-longer-supported [Cygwin-based Windows Development Environment](../dev_setup/dev_env_windows_cygwin.md).

## 附加信息

### Features / Issues

The following features are known to work (version 2.0):

- Building and running SITL with jMAVSim with significantly better performance than a VM (it generates a native windows binary **px4.exe**).
- Building and uploading NuttX builds (e.g.: px4_fmu-v2 and px4_fmu-v4)
- Style check with _astyle_ (supports the command: `make format`)
- Command line auto completion
- Non-invasive installer! The installer does NOT affect your system and global path (it only modifies the selected installation directory e.g. \*\*C:\PX4\*\* and uses a temporary local path).
- The installer supports updating to a new version keeping your personal changes inside the toolchain folder

Omissions:

- Simulation: Gazebo and ROS are not supported.
- Only NuttX and JMAVSim/SITL builds are supported.
- [Known problems](https://github.com/orgs/PX4/projects/6) (Also use to report issues).

### Shell Script Installation

You can also install the environment using shell scripts in the Github project.

1. Make sure you have [Git for Windows](https://git-scm.com/download/win) installed.

2. Clone the repository https://github.com/PX4/windows-toolchain to the location you want to install the toolchain. Default location and naming is achieved by opening the `Git Bash` and executing:

   ```sh
   cd /c/
   git clone https://github.com/PX4/windows-toolchain PX4
   ```

3. If you want to install all components navigate to the freshly cloned folder and double click on the script `install-all-components.bat` located in the folder `toolchain`. If you only need certain components and want to safe Internet traffic and or disk space you can navigate to the different component folders like e.g. `toolchain\cygwin64` and click on the **install-XXX.bat** scripts to only fetch something specific.

4. Continue with [Getting Started](../dev_setup/dev_env_windows_cygwin.md#getting-started).

### Manual Installation (for Toolchain Developers)

This section describes how to setup the Cygwin toolchain manually yourself while pointing to the corresponding scripts from the script based installation repo.
The result should be the same as using the scripts or MSI installer.

:::info
The toolchain gets maintained and hence these instructions might not cover every detail of all the future changes.
:::

1. Create the _folders_: \*\*C:\PX4\*\*, \*\*C:\PX4\toolchain\*\* and \*\*C:\PX4\home\*\*

2. Download the _Cygwin installer_ file [setup-x86_64.exe](https://cygwin.com/setup-x86_64.exe) from the [official Cygwin website](https://cygwin.com/install.html)

3. Run the downloaded setup file

4. In the wizard choose to install into the folder: \*\*C:\PX4\toolchain\cygwin64\*\*

5. Select to install the default Cygwin base and the newest available version of the following additional packages:

   - **Category:Packagename**
   - Devel:cmake (3.3.2 gives no deprecated warnings, 3.6.2 works but has the warnings)
   - Devel:gcc-g++
   - Devel:gdb
   - Devel:git
   - Devel:make
   - Devel:ninja
   - Devel:patch
   - Editors:xxd
   - Editors:nano (unless you're the vim pro)
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

   The reason to start all the development tools through the prepared batch script is they preconfigure the starting program to use the local, portable Cygwin environment inside the toolchain's folder.
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

9. Install the JDK:

   - Download Java 14 from [Oracle](https://www.oracle.com/java/technologies/javase-jdk14-downloads.html) or [AdoptOpenJDK](https://adoptopenjdk.net/).
   - Because sadly there is no portable archive containing the binaries directly you have to install it.
   - Find the binaries and move/copy them to **C:\PX4\toolchain\jdk**.
   - You can uninstall the Kit from your Windows system again, we only needed the binaries for the toolchain.

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

   - Compile it with:

      ```sh
      cd genromfs-src
      make all
      ```

   - Copy the resulting binary **genromfs.exe** one folder level out to: **C:\PX4\toolchain\genromfs**

12. Make sure all the binary folders of all the installed components are correctly listed in the `PATH` variable configured by [**setup-environment.bat**](https://github.com/PX4/windows-toolchain/blob/master/toolchain/scripts/setup-environment.bat).
