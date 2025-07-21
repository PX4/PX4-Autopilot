# 펌웨어 설치 및 업데이트

_QGroundControl_ **desktop** versions can be used to install PX4 firmware onto [Pixhawk-series](../getting_started/flight_controller_selection.md) flight-controller boards.

:::warning
**Before you start installing Firmware** all USB connections to the vehicle must be _disconnected_ (both direct or through a telemetry radio).
The vehicle must _not be_ powered by a battery.
:::

## PX4 안정 버전 설치

Generally you should use the most recent _released_ version of PX4, in order to benefit from bug fixes and get the latest and greatest features.

:::tip
This is the version that is installed by default.
:::

PX4 설치

1. Start _QGroundControl_ and connect the vehicle.

2. Select **"Q" icon > Vehicle Setup > Firmware** (sidebar) to open _Firmware Setup_.

   ![Firmware disconnected](../../assets/qgc/setup/firmware/firmware_disconnected.png)

3. USB를 통해 비행 콘트롤러를 컴퓨터에 직접 연결합니다.

   ::: info
   Connect directly to a powered USB port on your machine (do not connect through a USB hub).

:::

4. Select the **PX4 Pro Stable Release vX.x.x** option to install the latest stable version of PX4 _for your flight controller_ (autodetected).

   ![Install PX4 default](../../assets/qgc/setup/firmware/firmware_connected_default_px4.png)

5. Click the **OK** button to start the update.

   펌웨어가 업그레이드(펌웨어 다운로드, 이전 펌웨어 삭제 등)를 진행합니다.
   각 단계 화면을 표출하고, 전체 진행률이 표시줄에 출력됩니다.

   ![Firmware upgrade complete](../../assets/qgc/setup/firmware/firmware_upgrade_complete.png)

   펌웨어의 업로드가 완료되면, 장치가 재부팅되고 다시 연결됩니다.

   :::tip
   If _QGroundControl_ installs the FMUv2 target (see console during installation) and you have a newer board, you may need to [update the bootloader](#bootloader) in order to access all the memory on your flight controller.

:::

Next you will need to specify the [vehicle airframe](../config/airframe.md) (and then sensors, radio, etc.)

<a id="custom"></a>

## Installing PX4 Main, Beta or Custom Firmware

다른 버전의 PX4 설치

1. Connect the vehicle as above, and select **PX4 Pro Stable Release vX.x.x**.
   ![Install PX4 version](../../assets/qgc/setup/firmware/qgc_choose_firmware.png)
2. Check **Advanced settings** and select the version from the dropdown list:
   - **Standard Version (stable):** The default version (i.e. no need to use advanced settings to install this!)
   - **Beta Testing (beta):** A beta/candidate release.
      신규 버전 출시 이전에 테스트 할 경우에만 사용할 수 있습니다.
   - **Developer Build (master):** The latest build of PX4/PX4-Autopilot _main_ branch.
   - **Custom Firmware file...:** A custom firmware file (e.g. [that you have built locally](../dev_setup/building_px4.md)).
      사용자 정의 펌웨어 파일을 선택한 경우 다음 단계에서 파일 시스템에서 사용자 정의 펌웨어를 선택하여야 합니다.

그러면 펌웨어 업데이트가 이전과 같이 계속됩니다.

<a id="bootloader"></a>

## 부트로더 업데이트

Pixhawk hardware usually comes with an appropriate bootloader version pre-installed.

A case where you may need to update is newer Pixhawk boards that install FMUv2 firmware.
If _QGroundControl_ installs the FMUv2 target (see console during installation), and you have a newer board, you may need to update the bootloader in order to access all the memory on your flight controller.

![FMUv2 update](../../assets/qgc/setup/firmware/bootloader_update.jpg)

You can update it by following the instructions in [Bootloader update > FMUv2 Bootloader Update](../advanced_config/bootloader_update.md#fmuv2-bootloader-update).

## 추가 정보

- [QGroundControl User Guide > Firmware](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html).
- [PX4 Setup Video](https://youtu.be/91VGmdSlbo4) (Youtube)
