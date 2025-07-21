# PilotPi 使用 Ubuntu Server 操作系统

:::warning
Ubuntu Server on RPi 4B consumes a lot of current and generates a lot of heat.
Design for better heat dissipation and high power consumption when using this hardware.
:::

## 开发者快速指南

### 操作系统映像

请从官方 <a href="https://cdimage.ubuntu.com/releases/">cdimage</a> 页面获取最新更新的操作系统。

#### armhf

- [Ubuntu Server 18.04.5 for RPi2](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-armhf+raspi2.img.xz)
- [Ubuntu Server 18.04.5 for RPi3](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-armhf+raspi3.img.xz)
- [Ubuntu Server 18.04.5 for RPi4](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-armhf+raspi4.img.xz)
- [Ubuntu Server 20.04.1 for RPi 2/3/4](https://cdimage.ubuntu.com/releases/20.04.1/release/ubuntu-20.04.2-preinstalled-server-arm64+raspi.img.xz)

#### arm64

- [Ubuntu Server 18.04.5 for RPi3](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz)
- [Ubuntu Server 18.04.5 for RPi4](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz)
- [Ubuntu Server 20.04.1 for RPi 3/4](https://cdimage.ubuntu.com/releases/20.04.1/release/ubuntu-20.04.2-preinstalled-server-arm64+raspi.img.xz)

#### 最新操作系统

Please refer to official [cdimage](https://cdimage.ubuntu.com/releases/) page for any new updates.

### 首次启动

When setting up RPi's WiFi for the first time we recommended using a wired Ethernet connection between your home router and RPi, and a monitor and keyboard.

#### 启动前

现在将 SD 卡插入您的 Pi 并首次开机。
Please follow the official instruction [here](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet).

Now plug the SD card onto your Pi and boot for the first time.
Make sure you have shell access to the RPi - either SSH connection over wired Ethernet, or direct accessing with keyboard and monitor.

#### WiFi 区域

First install required package:

```sh
sudo apt-get install crda
```

Edit the file `/etc/default/crda` to change the correct WiFi region. [Reference List](https://www.arubanetworks.com/techdocs/InstantWenger_Mobile/Advanced/Content/Instant%20User%20Guide%20-%20volumes/Country_Codes_List.htm)

```sh
sudo nano /etc/default/crda
```

让我们先设置主机名。

#### 主机名和 mDNS

Let's set up hostname at first.

```sh
sudo nano /etc/hostname
```

Change the hostname to whatever you like.
Then install the package required by mDNS:

```sh
sudo apt-get update
sudo apt-get install avahi-daemon
```

在上述操作后通过无线网络重新连回树莓派。

```sh
sudo reboot
```

您也可能想要设置 <a href="https://www.raspberrypi.org/documentation/remote-access/ssh/passwordless.md">无密码认证</a>。

```sh
ssh ubuntu@pi_hostname.local
```

#### 无密码认证(可选)

You may want to setup [passwordless auth](https://www.raspberrypi.org/documentation/remote-access/ssh/passwordless.md) as well.

### 配置操作系统

#### config.txt

The corresponding file in Ubuntu is `/boot/firmware/usercfg.txt`.

```sh
sudo nano /boot/firmware/usercfg.txt
```

将文件内容替换为：

```sh
# enable sc16is752 overlay
dtoverlay=sc16is752-spi1
# enable I2C-1 and set the frequency to 400KHz
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
# enable spidev0.0
dtparam=spi=on
# enable RC input
enable_uart=1
# enable I2C-0
dtparam=i2c_vc=on
# switch Bluetooth to miniuart
dtoverlay=miniuart-bt
```

#### cmdline.txt

在Ubuntu Server 18.04 或更早版本，<code>nobtcmd.txt</code> 和 <code>btcmd.txt</code> 都需要修改。

```sh
sudo nano /boot/firmware/cmdline.txt
```

On Ubuntu Server 18.04 or earlier, `nobtcmd.txt` and `btcmd.txt` should both be modified.

```sh
sudo nano /boot/firmware/nobtcmd.txt
```

Find `console=/dev/ttyAMA0,115200` and remove that part to disable the login shell on serial interface.

Append `isolcpus=2` after the last word.
我们将在稍后手动在该核心运行PX4。

```sh
net.ifnames=0 dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc isolcpus=2
```

The above line tells the Linux kernel do not schedule any process on CPU core 2.
我们将在稍后手动在该核心运行 PX4。

检查串口：

检查串口：

```sh
ls /dev/tty*
```

There should be `/dev/ttyAMA0`, `/dev/ttySC0` and `/dev/ttySC1`.

检查 I2C：

```sh
ls /dev/i2c*
```

There should be `/dev/i2c-0` and `/dev/i2c-1`

应该有 <code>/dev/spidev0.0</code>。

```sh
ls /dev/spidev*
```

There should be `/dev/spidev0.0`.

#### rc.local

In this section we will configure the auto-start script in **rc.local**.
Note that we need to create this file, as it is not present on a fresh Ubuntu OS.

```sh
sudo nano /etc/rc.local
```

Append the content below to the file:

```sh
#!/bin/sh

echo "25" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio25/direction
if [ $(cat /sys/class/gpio/gpio25/value) -eq 1 ] ; then
        echo "Launching PX4"
        cd /home/ubuntu/px4 ; nohup taskset -c 2 ./bin/px4 -d -s pilotpi_mc.config 2 &> 1 > /home/ubuntu/px4/px4.log &
fi
echo "25" > /sys/class/gpio/unexport

exit 0
```

保存并退出。
Then set the correct permissions:

```sh
sudo chmod +x /etc/rc.local
```

:::info
Don't forget to turn off the switch when it is not needed!
:::

#### CSI 相机

:::warning
Enable CSI camera will stop anything works on I2C-0.
:::

```sh
sudo nano /boot/firmware/usercfg.txt
```

或

```sh
start_x=1
```

### 构建代码

To get the _very latest_ version onto your computer, enter the following command into a terminal:

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

:::info
This is all you need to do just to build the latest code.
:::

#### 配置上传

然后上传：

```sh
export AUTOPILOT_HOST=192.168.X.X
```

或

```sh
export AUTOPILOT_HOST=pi_hostname.local
```

在 PX4-Autopilot 文件夹下执行：

```sh
export AUTOPILOT_USER=ubuntu
```

#### 为 armhf 目标交叉编译

PX4 已配置使用多旋翼模型启动。

```sh
cd Firmware
make scumaker_pilotpi_default
```

Then upload it with:

```sh
make scumaker_pilotpi_default upload
```

#### 备选armhf构建方法 (使用 docker)

If you are compiling for the first time with docker, please refer to the [official docs](../test_and_ci/docker.md#prerequisites).

如果您是首次使用 Docker 进行编译，请参考<a href="https://dev.px4.io/master/en/test_and_ci/docker.html#prerequisites">官方说明</a>。

```sh
./Tools/docker_run.sh "export AUTOPILOT_HOST=192.168.X.X; export AUTOPILOT_USER=ubuntu; export NO_NINJA_BUILD=1; make scumaker_pilotpi_default upload"
```

:::info
mDNS is not supported within docker. 您也可以自己创建一个。
:::

:::info
If your IDE doesn't support ninja build, `NO_NINJA_BUILD=1` option will help.
You can compile without uploading too. Just remove `upload` target.
:::

It is also possible to just compile the code with command:

```sh
./Tools/docker_run.sh "make scumaker_pilotpi_default"
```

#### 为arm64交叉编译

:::info
This step requires `aarch64-linux-gnu` tool-chain to be installed.
:::

PX4 已配置使用多旋翼模型启动。

```sh
cd PX4-Autopilot
make scumaker_pilotpi_arm64
```

Then upload it with:

```sh
make scumaker_pilotpi_arm64 upload
```

#### 备选 arm64 构建方法 (使用 docker)

If you are compiling for the first time with docker, please refer to the [official docs](../test_and_ci/docker.md#prerequisites).

Execute the command in `PX4-Autopilot` folder:

```sh
./Tools/docker_run.sh "export AUTOPILOT_HOST=192.168.X.X; export AUTOPILOT_USER=ubuntu; export NO_NINJA_BUILD=1; make scumaker_pilotpi_arm64 upload"
```

:::info
mDNS is not supported within docker. 您也可以自己创建一个。
:::

:::info
If your IDE doesn't support ninja build, `NO_NINJA_BUILD=1` option will help.
You can compile without uploading too - just remove the `upload` target.
:::

It is also possible to just compile the code with command:

```sh
./Tools/docker_run.sh "make scumaker_pilotpi_arm64"
```

#### 手动运行 PX4

Connect over SSH and run it with:

```sh
cd px4
sudo taskset -c 2 ./bin/px4 -s pilotpi_mc.config
```

在执行下一步之前，先清除现有构建目录：

If you encountered the similar problem executing `bin/px4` on your Pi as following:

```
bin/px4: /lib/xxxx/xxxx: version `GLIBC_2.29' not found (required by bin/px4)
```

如果您是首次使用 Docker 进行编译，请参考<a href="https://dev.px4.io/master/en/test_and_ci/docker.html#prerequisites">官方说明</a>。

在 PX4-Autopilot 文件夹下执行：

```sh
rm -rf build/scumaker_pilotpi_*
```

Then go back to the corresponding chapter above.

### 后期配置

Please refer to the instructions [here](raspberry_pi_pilotpi_rpios.md)
