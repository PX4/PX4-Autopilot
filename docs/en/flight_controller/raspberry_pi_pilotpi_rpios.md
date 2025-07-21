# PilotPi with Raspberry Pi OS

## Developer Quick Start

### OS Image

The latest official [Raspberry Pi OS Lite](https://downloads.raspberrypi.org/raspios_lite_armhf_latest) image is always recommended.

To install you must already have a working SSH connection to RPi.

### Setting up Access (Optional)

#### Hostname and mDNS

mDNS helps you connect to your RPi with hostname instead of IP address.

```sh
sudo raspi-config
```

Navigate to **Network Options > Hostname**.
Set and exit.
You may want to setup [passwordless auth](https://www.raspberrypi.org/documentation/remote-access/ssh/passwordless.md) as well.

### Setting up OS

#### config.txt

```sh
sudo nano /boot/config.txt
```

Replace the file with:

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
Enable UART but without a login shell on it.

```sh
sudo nano /boot/cmdline.txt
```

Append `isolcpus=2` after the last word.
The whole file would be:

```sh
console=tty1 root=PARTUUID=xxxxxxxx-xx rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait isolcpus=2
```

This tells the Linux kernel not to schedule any process on CPU core 2.
We will manually run PX4 onto that core later.

Reboot and SSH onto your RPi.

Check UART interface:

```sh
ls /dev/tty*
```

There should be `/dev/ttyAMA0`, `/dev/ttySC0` and `/dev/ttySC1`.

Check I2C interface:

```sh
ls /dev/i2c*
```

There should be `/dev/i2c-0` and `/dev/i2c-1`

Check SPI interface

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

Save and exit.

::: info
Don't forget to turn off the switch when it is not needed.
:::

#### CSI camera

::: info
Enable CSI camera will stop anything works on I2C-0.
:::

```sh
sudo raspi-config
```

**Interfacing Options > Camera**

### Building the code

To get the _very latest_ version onto your computer, enter the following command into a terminal:

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

::: info
This is all you need to do just to build the latest code.
:::

#### Cross build for Raspberry Pi OS

Set the IP (or hostname) of your RPi using:

```sh
export AUTOPILOT_HOST=192.168.X.X
```

or

```sh
export AUTOPILOT_HOST=pi_hostname.local
```

Build the executable file:

```sh
cd PX4-Autopilot
make scumaker_pilotpi_default
```

Then upload it with:

```sh
make scumaker_pilotpi_default upload
```

Connect over ssh and run it with:

```sh
cd px4
sudo taskset -c 2 ./bin/px4 -s pilotpi_mc.config
```

Now PX4 is started with multi-rotor configuration.

If you encountered the similar problem executing `bin/px4` on your Pi as following:

```
bin/px4: /lib/xxxx/xxxx: version `GLIBC_2.29' not found (required by bin/px4)
```

Then you should compile with docker instead.

Before proceeding to next step, clear the existing building at first:

```sh
rm -rf build/scumaker_pilotpi_default
```

### Alternative build method (using docker)

The following method can provide the same tool-sets deployed in CI.

If you are compiling for the first time with docker, please refer to the [official docs](../test_and_ci/docker.md#prerequisites).

Execute the command in PX4-Autopilot folder:

```sh
./Tools/docker_run.sh "export AUTOPILOT_HOST=192.168.X.X; export NO_NINJA_BUILD=1; make scumaker_pilotpi_default upload"
```

::: info
mDNS is not supported within docker. You must specify the correct IP address every time when uploading.
:::

::: info
If your IDE doesn't support ninja build, `NO_NINJA_BUILD=1` option will help.
You can compile without uploading too. Just remove `upload` target.
:::

It is also possible to just compile the code with command:

```sh
./Tools/docker_run.sh "make scumaker_pilotpi_default"
```

### Post-configuration

You need to check these extra items to get your vehicle work properly.

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

::: info
Generally you only have one of them.
Other devices will also be displayed here if they are connected to external I2C bus.(`/dev/i2c-0`)
:::
