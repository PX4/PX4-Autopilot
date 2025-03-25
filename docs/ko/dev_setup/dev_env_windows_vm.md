# Windows 가상 머신 호스팅 도구 모음

:::warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

Windows 개발자는 Linux를 게스트 운영 체제로 사용하는 가상 머신(VM)에서 PX4 툴체인을 실행할 수 있습니다.
가상 머신을 설정한 후, 가상 머신내의 PX4 설치 및 설정은 일반 Linux 환경에서의 설정과 동일합니다.

가상 머신을 사용하는 것은 펌웨어 구축 환경을 설정과 테스트가 매우 편리하지만, 사용자는 다음 사항에 유의하여야 합니다.

1. 펌웨어 빌드는 Linux에서 빌드하는 것보다 조금 느립니다.
2. The JMAVSim simulation, frame rate be much slower than on native Linux.
   경우에 따라서, 가상 머신 리소스 부족과 관련된 문제로 차량이 충돌할 수 있습니다.
3. Gazebo와 ROS는 설치할 수 있지만, 사용할 수 없을 정도로 느립니다.

:::tip
Allocate as many CPU cores and memory resources to the VM as possible.
:::

시스템에서 PX4 실행을 위한 가상 머신을 설정하는 방법에는 여러 가지가 있습니다.
이 가이드는 VMWare 설정 방법을 설명합니다.
There is also an incomplete section for VirtualBox at the end (we'd welcome expansion of this section from a community member).

## VMWare Setup

VMWare performance is acceptable for basic usage (building Firmware) but not for running ROS or Gazebo Classic.

1. Download [VMWare Player Freeware](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html)

2. 윈도우 시스템에 설치합니다.

3. Download the desired version of [Ubuntu Desktop ISO Image](https://www.ubuntu.com/download/desktop).
   (see [Linux Instructions Page](../dev_setup/dev_env_linux.md) for recommended Ubuntu version).

4. Open _VMWare Player_.

5. Enable 3D acceleration in the VM's settings: **VM > Settings > Hardware > Display > Accelerate 3D graphics**

   ::: info
   This option is required to properly run 3D simulation environments like jMAVSim and Gazebo Classic.
   가상 환경에 Linux를 설치하기 전에 이 작업을 수행하는 것이 좋습니다.

:::

6. 새 가상 머신을 생성하는 메뉴를 선택합니다.

7. 가상 머신 생성 마법사에서 다운로드한 Ubuntu ISO 이미지를 설치 매체로 선택하면, 사용하려는 운영 체제가 자동으로 감지됩니다.

8. 마법사에서 실행 중인 가상 머신에 할당할 리소스를 선택합니다.
   가상 머신에 최대한 많은 메모리와 CPU 코어를 할당하십시오.

9. 마법사가 종료시 새 가상 머신을 실행하고, 설정 지침에 따라 Ubuntu를 설치합니다.
   모든 설정은 호스트 운영 체제에서 사용하기 위한 것이므로, 네트워크 공격의 위험을 증가시키지 않는 화면 보호기 및 로컬 워크스테이션 보안 기능을 비활성화할 수 있습니다.

10. Once the new VM is booted up make sure you install _VMWare tools drivers and tools extension_ inside your guest system.
   이렇게 하면 다음과 같은 VM 사용의 성능과 유용성들이 향상됩니다.

   - 크게 향상된 그래픽 성능
   - Proper support for hardware device usage like USB port allocation (important for target upload), proper mouse wheel scrolling, sound support
   - 창 크기에 따른 게스트 디스플레이 해상도 조정
   - 호스트 시스템 클립보드 공유
   - 호스트 시스템 파일 공유

11. Continue with [PX4 environment setup for Linux](../dev_setup/dev_env_linux.md)

## VirtualBox 7 Setup

The setup for VirtualBox is similar to VMWare.
Community members, we'd welcome a step-by-step guide here!

### USB passthrough for QGroundControl / Firmware Flashing

:::tip
This section has been tested for VirtualBox 7 running Ubuntu 20.04 LTS on a Windows 10 host machine.
:::

One limitation of virtual machines is that you can't automatically connect to a flight controller attached to the host computer USB port in order to [build and upload PX4 firmware from a terminal](../dev_setup/building_px4.md#uploading-firmware-flashing-the-board).
You also can't connect to the flight controller from QGroundControl in the virtual machine.

To allow this, you need to configure USB passthrough settings:

1. Ensure that the user has been added to the dialout group in the VM using the terminal command:

   ```sh
   sudo usermod -a -G dialout $USER
   ```

   Then restart Ubuntu in the virtual machine.

2. Enable serial port(s) in VM: **VirtualBox > Settings > Serial Ports 1/2/3/etc...**

3. Enable USB controller in VM: **VirtualBox > Settings > USB**

4. Add USB filters for the bootloader in VM: **VirtualBox > Settings > USB > Add new USB filter**.

   - Open the menu and plug in the USB cable connected to your autopilot.
      Select the `...Bootloader` device when it appears in the UI.

      ::: info
      The bootloader device only appears for a few seconds after connecting USB.
      If it disappears before you can select it, disconnect and then reconnect USB.

:::

   - Select the `...Autopilot` device when it appears (this happens after the bootloader completes).

5. Select the device in the VM instance's dropdown menu **VirtualBox > Devices > your_device**

If successful, your device will show up with `lsusb` and QGroundControl will connect to the device automatically.
You should also be able to build and upload firmware using a command like:

```sh
make px4_fmu-v5_default upload
```

### Telemetry over WiFi for QGroundControl

If using _QGroundControl_ within a virtual machine you should set the VM networking settings to "Bridged Adapter" mode.
This gives the guest OS direct access to networking hardware on the host.
If you use the Network Address Translation (NAT), which is set by default for VirtualBox 7 running Ubuntu 20.04 LTS, this will block the outbound UDP packets that QGroundControl uses to communicate with the vehicle.
