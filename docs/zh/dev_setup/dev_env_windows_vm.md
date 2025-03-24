# Windows 虚拟机托管的工具链

:::warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

Windows developers can run the PX4 toolchain in a virtual machine (VM) with Linux as the guest operating system.
After setting up the virtual machine, the installation and setup of PX4 within the VM is exactly the same as on a native Linux computer.

While using a VM is a very easy way to set up and test an environment for building firmware, users should be aware:

1. 固件的编译速度比原生 Linux 要更慢一些。
2. The JMAVSim simulation, frame rate be much slower than on native Linux.
   虚拟机运行资源不足可能导致特定情况下无人机坠毁。
3. 可以安装 Gazebo 和 ROS，但运行速度非常慢。

:::tip
Allocate as many CPU cores and memory resources to the VM as possible.
:::

There are multiple ways to setup a VM which is capable of executing the PX4 environment on your system.
This guide walks you through a VMWare setup.
There is also an incomplete section for VirtualBox at the end (we'd welcome expansion of this section from a community member).

## VMWare Setup

VMWare performance is acceptable for basic usage (building Firmware) but not for running ROS or Gazebo Classic.

1. Download [VMWare Player Freeware](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html)

2. 将其安装在 Windows 系统上。

3. Download the desired version of [Ubuntu Desktop ISO Image](https://www.ubuntu.com/download/desktop).
   (see [Linux Instructions Page](../dev_setup/dev_env_linux.md) for recommended Ubuntu version).

4. Open _VMWare Player_.

5. Enable 3D acceleration in the VM's settings: **VM > Settings > Hardware > Display > Accelerate 3D graphics**

   ::: info
   This option is required to properly run 3D simulation environments like jMAVSim and Gazebo Classic.
   We recommend this is done before installing Linux in the virtual environment.

:::

6. Select the option to create a new virtual machine.

7. In the VM creation wizard choose the downloaded Ubuntu ISO image as your installation medium and will automatically detect the operating system you want to use.

8. Also in the wizard, select the resources you want to allocate to your virtual machine while it is running.
   Allocate as much memory and as many CPU cores as you can without rendering your host Windows system unusable.

9. Run your new VM at the end of the wizard and let it install Ubuntu following the setup instructions.
   Remember all settings are only for within your host operating system usage and hence you can disable any screen saver and local workstation security features which do not increase risk of a network attack.

10. Once the new VM is booted up make sure you install _VMWare tools drivers and tools extension_ inside your guest system.
   This will enhance performance and usability of your VM usage:

   - Significantly enhanced graphics performance
   - Proper support for hardware device usage like USB port allocation (important for target upload), proper mouse wheel scrolling, sound support
   - Guest display resolution adaption to the window size
   - Clipboard sharing to host system
   - File sharing to host system

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
