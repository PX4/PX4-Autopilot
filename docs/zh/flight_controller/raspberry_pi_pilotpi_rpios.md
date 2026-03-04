# 在 Raspberry Pi OS 上使用 PilotPi

## 开发者快速指南

### 操作系统映像

The latest official [Raspberry Pi OS Lite](https://downloads.raspberrypi.org/raspios_lite_armhf_latest) image is always recommended.

默认你已经通过ssh连接到了树莓派。

### 设置快速访问(可选)

#### 主机名和 mDNS

mDNS helps you connect to your RPi with hostname instead of IP address.

```sh
sudo raspi-config
```

Navigate to **Network Options > Hostname**.
设置并退出。
You may want to setup [passwordless auth](https://www.raspberrypi.org/documentation/remote-access/ssh/passwordless.md) as well.

### 配置操作系统

#### config.txt

```sh
sudo nano /boot/config.txt
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

```sh
sudo raspi-config
```

**Interfacing Options > Serial > login shell = No > hardware = Yes**.
启用 UART 但禁用登陆shell。

```sh
sudo nano /boot/cmdline.txt
```

Append `isolcpus=2` after the last word.
整个文件将是：

```sh
console=tty1 root=PARTUUID=xxxxxxxx-xx rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait isolcpus=2
```

这告诉 Linux 内核不要在 CPU 核心2 上调度任何进程。
我们将在稍后手动在该核心运行 PX4。

Reboot and SSH onto your RPi.

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

检查SPI：

```sh
ls /dev/spidev*
```

There should be `/dev/spidev0.0`.

#### rc.local

In this section we will configure the auto-start script in **rc.local**.

```sh
sudo nano /etc/rc.local
```

Append below content to the file above `exit 0`:

```sh
echo "25" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio25/direction
if [ $(cat /sys/class/gpio/gpio25/value) -eq 1 ] ; then
        echo "Launching PX4"
        cd /home/pi/px4 ; nohup taskset -c 2 ./bin/px4 -d -s pilotpi_mc.config 2 &> 1 > /home/pi/px4/px4.log &
fi
echo "25" > /sys/class/gpio/unexport
```

保存并退出。

:::info
Don't forget to turn off the switch when it is not needed.
:::

#### CSI 相机

:::info
Enable CSI camera will stop anything works on I2C-0.
:::

```sh
sudo raspi-config
```

**Interfacing Options > Camera**

### 构建代码

To get the _very latest_ version onto your computer, enter the following command into a terminal:

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

:::info
This is all you need to do just to build the latest code.
:::

#### 为 Raspberry Pi OS 交叉编译

然后上传：

```sh
export AUTOPILOT_HOST=192.168.X.X
```

或

```sh
export AUTOPILOT_HOST=pi_hostname.local
```

PX4 已配置使用多旋翼模型启动。

```sh
cd PX4-Autopilot
make scumaker_pilotpi_default
```

Then upload it with:

```sh
make scumaker_pilotpi_default upload
```

这时应当使用基于 Docker 的编译。

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
rm -rf build/scumaker_pilotpi_default
```

### 备选构建方法 (使用 docker)

只是为了编译代码，则可以执行：

If you are compiling for the first time with docker, please refer to the [official docs](../test_and_ci/docker.md#prerequisites).

混控器在 <code>pilotpi_xx.conf</code> 文件中启用：

```sh
./Tools/docker_run.sh "export AUTOPILOT_HOST=192.168.X.X; export NO_NINJA_BUILD=1; make scumaker_pilotpi_default upload"
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

### 后期配置

示例输出

#### Actuator Configuration

First set the [CA_AIRFRAME](../advanced_config/parameter_reference.md#CA_AIRFRAME) parameter for your vehicle.

You will then be able to assign outputs using the normal [Actuator Configuration](../config/actuators.md) configuration screen (an output tab will appear for the RPi PWM output driver).

#### External Compass

In the startup script(`*.config`), you will find

```sh
# external GPS & compass
gps start -d /dev/ttySC0 -i uart -p ubx -s
#hmc5883 start -X
#ist8310 start -X
```

Uncomment the correct one for your case.
Not sure which compass comes up with your GPS module? Execute the following commands and see the output:

```sh
sudo apt-get update
sudo apt-get install i2c-tools
i2cdetect -y 0
```

Sample output:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- 0e --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1e --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

`1e` indicates a HMC5883 based compass is mounted on external I2C bus. Similarly, IST8310 has a value of `0e`.

:::info
Generally you only have one of them.
Other devices will also be displayed here if they are connected to external I2C bus.(`/dev/i2c-0`)
:::
