# Raspberry Pi 2/3/4 Navio2 Autopilot

<LinkedBadge type="warning" text="Experimental" url="../flight_controller/autopilot_experimental.html"/>

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://emlid.com/) for hardware support or compliance issues.
:::

This is the developer "quickstart" for Raspberry Pi 2/3/4 Navio2 autopilots.
It allows you to build PX4 and transfer to the RPi, or build natively.

![Ra Pi Image](../../assets/hardware/hardware-rpi2.jpg)

## 操作系统映像

Use the preconfigured [Emlid Raspberry Pi OS image for Navio 2](https://docs.emlid.com/navio2/configuring-raspberry-pi).
The default image will have most of the setup procedures shown below already done.

:::warning
Make sure not to upgrade the system (more specifically the kernel).
By upgrading, a new kernel can get installed which lacks the necessary HW support (you can check with `ls /sys/class/pwm`, the directory should not be empty).
:::

## Setting up Access

The Raspberry Pi OS image has SSH setup already.
Username is "pi" and password is "raspberry".
We assume that the username and password remain at their defaults for the purpose of this guide.

To setup the Pi to join your local wifi, follow [this guide](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md), or connect it via an ethernet cable.

To connect to your Pi via SSH, use the default username (`pi`) and hostname (`navio`).
Alternatively (if this doesn't work), you can find the IP address of your RPi and specify it.

```sh
ssh pi@navio.local
```

或

```sh
ssh pi@<IP-ADDRESS>
```

## Expand the Filesystem

Expand the file system to take advantage of the entire SD card by running:

```sh
sudo raspi-config --expand-rootfs
```

## Disable Navio RGB Overlay

The existing Navio RGB overlay claims GPIOs used by PX4 for RGB Led.
Edit `/boot/config.txt` by commenting the line enabling the `navio-rgb` overlay.

```
#dtoverlay=navio-rgb
```

## Testing file transfer

We use SCP to transfer files from the development computer to the target board over a network (WiFi or Ethernet).

To test your setup, try pushing a file from the development PC to the Pi over the network now.
Make sure the Pi has network access, and you can SSH into it.

```sh
echo "Hello" > hello.txt
scp hello.txt pi@navio.local:/home/pi/
rm hello.txt
```

This should copy over a "hello.txt" file into the home folder of your Pi.
Validate that the file was indeed copied, and you can proceed to the next step.

## PX4 Development Environment

These instructions explain how to install a PX4 development environment for building RPi on Ubuntu 18.04.

:::warning
PX4 binaries for Navio 2 can only be run on Ubuntu 18.04.

You can build PX4 using the GCC toolchain on Ubuntu 20.04, but the generated binary files are too new to run on actual Pi (as of September 2023).
For more information see [PilotPi with Raspberry Pi OS Developer Quick Start > Alternative build method using docker](../flight_controller/raspberry_pi_pilotpi_rpios.md#alternative-build-method-using-docker).
:::

### Install the Common Dependencies

To get the common dependencies for Raspberry Pi:

1. Download [ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/ubuntu.sh) <!-- NEED px4_version --> and [requirements.txt](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**): <!-- NEED px4_version -->

   ```sh
   wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh
   wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt
   ```

2. Run **ubuntu.sh** in a terminal to get just the common dependencies:

   ```sh
   bash ubuntu.sh --no-nuttx --no-sim-tools
   ```

3. Then setup a cross-compiler (either GCC or clang) as described in the following sections.

### GCC (armhf)

Ubuntu software repository provides a set of pre-compiled toolchains. Note that Ubuntu Focal comes up with `gcc-9-arm-linux-gnueabihf` as its default installation which is not fully supported, so we must manually install `gcc-8-arm-linux-gnueabihf` and set it as the default toolchain. This guide also applies to earlier Ubuntu releases (Bionic).
The following instruction assumes you haven't installed any version of arm-linux-gnueabihf, and will set up the default executable with `update-alternatives`.
Install them with the terminal command:

```sh
sudo apt-get install -y gcc-8-arm-linux-gnueabihf g++-8-arm-linux-gnueabihf
```

Set them as default:

```sh
sudo update-alternatives --install /usr/bin/arm-linux-gnueabihf-gcc arm-linux-gnueabihf-gcc /usr/bin/arm-linux-gnueabihf-gcc-8 100 --slave /usr/bin/arm-linux-gnueabihf-g++ arm-linux-gnueabihf-g++ /usr/bin/arm-linux-gnueabihf-g++-8
sudo update-alternatives --config arm-linux-gnueabihf-gcc
```

### GCC (aarch64)

If you want to build PX4 for ARM64 devices, this section is required.

```sh
sudo apt-get install -y gcc-8-aarch64-linux-gnu g++-8-aarch64-linux-gnu
sudo update-alternatives --install /usr/bin/aarch64-linux-gnu-gcc aarch64-linux-gnu-gcc /usr/bin/aarch64-linux-gnu-gcc-8 100 --slave /usr/bin/aarch64-linux-gnu-g++ aarch64-linux-gnu-g++ /usr/bin/aarch64-linux-gnu-g++-8
sudo update-alternatives --config aarch64-linux-gnu-gcc
```

### Clang (optional)

First install GCC (needed to use clang).

We recommend you to get clang from the Ubuntu software repository, as shown below:

```sh
sudo apt-get install clang
```

Example below for building PX4 firmware out of tree, using _CMake_.

```sh
cd <PATH-TO-PX4-SRC>
mkdir build/px4_raspberrypi_default_clang
cd build/px4_raspberrypi_default_clang
cmake \
-G"Unix Makefiles" \
-DCONFIG=px4_raspberrypi_default \
-UCMAKE_C_COMPILER \
-DCMAKE_C_COMPILER=clang \
-UCMAKE_CXX_COMPILER \
-DCMAKE_CXX_COMPILER=clang++ \
../..
make
```

## 构建代码

Specify the IP (or hostname) of your Pi using:

```sh
export AUTOPILOT_HOST=navio.local
```

或

```sh
export AUTOPILOT_HOST=192.168.X.X
```

:::info
The value of the environment variable should be set before the build, or `make upload` will fail to find your Pi.
:::

Build the executable file on your development machine:

```sh
cd PX4-Autopilot
make emlid_navio2
```

The "px4" executable file is in the directory **build/emlid_navio2_default/**.
Make sure you can connect to your Pi over SSH, see [instructions how to access your Pi](#setting-up-access) following the instructions for armhf under Raspberry Pi.

Then upload it with:

```sh
cd PX4-Autopilot
make emlid_navio2 upload
```

Then, connect over ssh and run it on the Pi (as root):

```sh
cd ~/px4
sudo ./bin/px4 -s px4.config
```

A successful build followed by executing PX4 will give you something like this:

```sh

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.


pxh>
```

## Autostart

To autostart px4, add the following to the file **/etc/rc.local** (adjust it accordingly if you use native build), right before the `exit 0` line:

```sh
cd /home/pi && ./bin/px4 -d -s px4.config > px4.log
```
