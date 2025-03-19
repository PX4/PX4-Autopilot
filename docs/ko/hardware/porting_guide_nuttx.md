# NuttX 보드 포팅 가이드

NuttX의 PX4를 포팅하려면, 하드웨어가 NuttX를 지원하여야 합니다.
The NuttX project maintains an excellent [porting guide](https://cwiki.apache.org/confluence/display/NUTTX/Porting+Guide) for porting NuttX to a new computing platform.

The following guide assumes you are using an already supported hardware target or have ported NuttX (including the [PX4 base layer](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/nuttx/src/px4)) already.

The configuration files for all boards, including linker scripts and other required settings, are located under [/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards/) in a vendor- and board-specific directory (i.e. **boards/_VENDOR_/_MODEL_/**).

The following example uses FMUv5 as it is a recent [reference configuration](../hardware/reference_design.md) for NuttX based flight controllers:

- Running `make px4_fmu-v5_default` from the **PX4-Autopilot** directory will build the FMUv5 config
- The base FMUv5 configuration files are located in: [/boards/px4/fmu-v5](https://github.com/PX4/PX4-Autopilot/tree/main/boards/px4/fmu-v5).
  - Board specific header (NuttX pins and clock configuration): [/boards/px4/fmu-v5/nuttx-config/include/board.h](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/nuttx-config/include/board.h).
  - Board specific header (PX4 configuration): [/boards/px4/fmu-v5/src/board_config.h](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/src/board_config.h).
  - NuttX OS config (created with NuttX menuconfig): [/boards/px4/fmu-v5/nuttx-config/nsh/defconfig](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/nuttx-config/nsh/defconfig).
  - Build configuration: [boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board).

## NuttX 메뉴 구성 설정

To modify the NuttX OS configuration, you can use [menuconfig](https://bitbucket.org/patacongo/nuttx/src/master/) using the PX4 shortcuts:

```sh
make px4_fmu-v5_default menuconfig
make px4_fmu-v5_default qconfig
```

For fresh installs of PX4 onto Ubuntu using [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/ubuntu.sh) <!-- NEED px4_version --> you will also need to install _kconfig_ tools from [NuttX tools](https://bitbucket.org/nuttx/tools/src/master/).

:::info
The following steps are not required if using the [px4-dev-nuttx](https://hub.docker.com/r/px4io/px4-dev-nuttx/) docker container or have installed to macOS using our normal instructions (as these include`kconfig-mconf`).
:::

임의의 디렉터리에서 다음 명령어를 실행합니다.

```sh
git clone https://bitbucket.org/nuttx/tools.git
cd tools/kconfig-frontends
sudo apt install gperf
./configure --enable-mconf --disable-nconf --disable-gconf --enable-qconf --prefix=/usr
make
sudo make install
```

The `--prefix=/usr` determines the specific installation location (which must be in the `PATH` environment variable).
The `--enable-mconf` and `--enable-qconf` options will enable the `menuconfig` and `qconfig` options respectively.

To run `qconfig` you may need to install additional Qt dependencies.

### 부트로더

먼저 하드웨어 대상에 적합한 부트로더가 필요합니다.

- STM32H7: 부트로더는 NuttX를 기반으로 하며, PX4 펌웨어에 포함되어 있습니다.
  See [here](https://github.com/PX4/PX4-Autopilot/tree/main/boards/holybro/durandal-v1/nuttx-config/bootloader) for an example.
- 다른 대상의 경우 https://github.com/PX4/Bootloader가 사용됩니다. See [here](https://github.com/PX4/Bootloader/pull/155/files) for an example how to add a new target.
  Then checkout the [building and flashing instructions](../software_update/stm32_bootloader.md).

### 펌웨어 포팅 단계

1. Make sure you have a working [development setup](../dev_setup/dev_env.md) and installed the NuttX \`menuconfig\`\` tool (see above).

2. 소스 코드를 다운로드하고 기존 대상을 빌드할 수 있는 지 확인합니다.

  ```sh
  git clone --recursive https://github.com/PX4/PX4-Autopilot.git
  cd PX4-Autopilot
  make px4_fmu-v5
  ```

3. 동일한(또는 유사한) CPU 유형을 사용하는 기존 대상을 복사합니다.
  예: STM32F7의 경우

  ```sh
  mkdir boards/manufacturer
  cp -r boards/px4/fmu-v5 boards/manufacturer/my-target-v1
  ```

  Change **manufacturer** to the manufacturer name and **my-target-v1** to your board name.

Next you need to go through all files under **boards/manufacturer/my-target-v1** and update them according to your board.

1. **firmware.prototype**: update the board ID and name
2. **default.px4board**: update the **VENDOR** and **MODEL** to match the directory names (**my-target-v1**).
  직렬 포트를 설정합니다.
3. Configure NuttX (**defconfig**) via `make manufacturer_my-target-v1 menuconfig`: Adjust the CPU and chip, configure the peripherals (UART's, SPI, I2C, ADC).
4. **nuttx-config/include/board.h**: Configure the NuttX pins.
  여러 가지 핀 옵션이 있는 주변 장치에서는 NuttX는 핀 정보을 알아야 합니다.
  They are defined in the chip-specific pinmap header file, for example [stm32f74xx75xx_pinmap.h](https://github.com/PX4/NuttX/blob/px4_firmware_nuttx-8.2/arch/arm/src/stm32f7/hardware/stm32f74xx75xx_pinmap.h).
5. **src**: go through all files under **src** and update them as needed, in particular **board_config.h**.
6. **init/rc.board_sensors**: start the sensors that are attached to the board.
