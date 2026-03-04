# Qt Creator IDE

:::warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env.md).
It may or may not work with current versions of PX4.

Qt Creator has been replaced by [VSCode](../dev_setup/vscode.md) as the officially supported (and recommended) IDE for PX4 development.
See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[Qt Creator](https://www.qt.io/download-open-source) is a popular cross-platform open-source IDE that can be used to compile and debug PX4.

## Qt Creator 기능

Qt Creator는 클릭 가능한 기호, 전체 코드베이스의 자동 완성, 펌웨어 빌드 및 플래싱을 제공합니다.

![Screenshot of Qt Creator](../../assets/toolchain/qtcreator.png)

아래 비디오는 사용 방법을 보여줍니다.

<lite-youtube videoid="Bkk8zttWxEI" title="(Qt Creator) PX4 Flight Stack Build Experience"/>

## IDE 설정

### 리눅스용 Qt Creator

Before starting Qt Creator, the [project file](https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/Generator-Specific-Information#codeblocks-generator) needs to be created:

```sh
cd ~/src/PX4-Autopilot
mkdir ../Firmware-build
cd ../Firmware-build
cmake ../PX4-Autopilot -G "CodeBlocks - Unix Makefiles"
```

Then load the CMakeLists.txt in the root PX4-Autopilot folder via **File > Open File or Project** (Select the CMakeLists.txt file).

After loading, the **play** button can be configured to run the project by selecting 'custom executable' in the run target configuration and entering 'make' as executable and 'upload' as argument.

### Windows용 Qt Creator

:::info
Windows has not been tested for PX4 development with Qt Creator.
:::

### Mac OS용 Qt Creator

Before starting Qt Creator, the [project file](https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/Generator-Specific-Information#codeblocks-generator) needs to be created:

```sh
cd ~/src/PX4-Autopilot
mkdir -p build/creator
cd build/creator
cmake ../.. -G "CodeBlocks - Unix Makefiles"
```

끝났습니다! Start _Qt Creator_ and then set up the project to build.

<!-- note, video here was removed/made private, and in any case out of date. Just hoping people can work it out -->
