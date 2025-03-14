# PilotPi з Raspberry Pi OS

## Швидкий старт для розробника

### OS Image

The latest official [Raspberry Pi OS Lite](https://downloads.raspberrypi.org/raspios_lite_armhf_latest) image is always recommended.

Для встановлення вам потрібно мати працююче SSH-з'єднання з RPi.

### Налаштування доступу (необов'язково)

#### Hostname та mDNS

mDNS helps you connect to your RPi with hostname instead of IP address.

```sh
sudo raspi-config
```

Navigate to **Network Options > Hostname**.
Встановіть та вийдіть.
You may want to setup [passwordless auth](https://www.raspberrypi.org/documentation/remote-access/ssh/passwordless.md) as well.

### Налаштування OS

#### config.txt

```sh
sudo nano /boot/config.txt
```

Замініть на:

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
Увімкніть UART, але без логіну в shell.

```sh
sudo nano /boot/cmdline.txt
```

Append `isolcpus=2` after the last word.
Весь файл буде:

```sh
console=tty1 root=PARTUUID=xxxxxxxx-xx rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait isolcpus=2
```

Це вказує ядру Linux не планувати жодного процесу на ядрі CPU 2.
Ми пізніше вручну запустимо PX4 на цьому ядрі.

Reboot and SSH onto your RPi.

Перевірте інтерфейс UART:

```sh
ls /dev/tty*
```

There should be `/dev/ttyAMA0`, `/dev/ttySC0` and `/dev/ttySC1`.

Перевірте інтерфейс I2C:

```sh
ls /dev/i2c*
```

There should be `/dev/i2c-0` and `/dev/i2c-1`

Перевірте інтерфейс SPI

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

Збережіть та вийдіть.

:::info
Don't forget to turn off the switch when it is not needed.
:::

#### CSI камера

:::info
Enable CSI camera will stop anything works on I2C-0.
:::

```sh
sudo raspi-config
```

**Interfacing Options > Camera**

### Збірка коду

To get the _very latest_ version onto your computer, enter the following command into a terminal:

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

:::info
This is all you need to do just to build the latest code.
:::

#### Кросс збірка для операційної системи Raspberry Pi

Встановіть IP-адресу (або ім'я хоста) вашого RPi за допомогою:

```sh
export AUTOPILOT_HOST=192.168.X.X
```

або

```sh
export AUTOPILOT_HOST=pi_hostname.local
```

Зберіть виконуваний файл:

```sh
cd PX4-Autopilot
make scumaker_pilotpi_default
```

Потім завантажте його за допомогою:

```sh
make scumaker_pilotpi_default upload
```

Під'єднайтесь через ssh та запустіть за допомогою:

```sh
cd px4
sudo taskset -c 2 ./bin/px4 -s pilotpi_mc.config
```

Тепер PX4 запустився з конфігурацією мультиротора.

If you encountered the similar problem executing `bin/px4` on your Pi as following:

```
bin/px4: /lib/xxxx/xxxx: version `GLIBC_2.29' not found (required by bin/px4)
```

Тоді ви повинні скомпілювати за допомогою Docker замість цього.

Перш ніж перейти до наступного кроку, спочатку видаліть наявну збірку:

```sh
rm -rf build/scumaker_pilotpi_default
```

### Альтернативний метод збірки (з використанням Docker)

Наступний метод може надати ті ж набори інструментів, що використовуються в CI.

If you are compiling for the first time with docker, please refer to the [official docs](../test_and_ci/docker.md#prerequisites).

Виконайте команду в директорії PX4-Autopilot:

```sh
./Tools/docker_run.sh "export AUTOPILOT_HOST=192.168.X.X; export NO_NINJA_BUILD=1; make scumaker_pilotpi_default upload"
```

:::info
mDNS is not supported within docker. Вам потрібно вказувати правильну IP-адресу кожного разу під час завантаження.
:::

:::info
If your IDE doesn't support ninja build, `NO_NINJA_BUILD=1` option will help.
Ви можете компілювати без завантаження також. Just remove `upload` target.
:::

Також можливо просто скомпілювати код за допомогою команди:

```sh
./Tools/docker_run.sh "make scumaker_pilotpi_default"
```

### Післяконфігурація

Вам потрібно перевірити ці додаткові елементи, щоб ваш апарат працював належним чином.

#### Конфігурація приводу

First set the [CA_AIRFRAME](../advanced_config/parameter_reference.md#CA_AIRFRAME) parameter for your vehicle.

You will then be able to assign outputs using the normal [Actuator Configuration](../config/actuators.md) configuration screen (an output tab will appear for the RPi PWM output driver).

#### Зовнішній компас

In the startup script(`*.config`), you will find

```sh
# external GPS & compass
gps start -d /dev/ttySC0 -i uart -p ubx -s
#hmc5883 start -X
#ist8310 start -X
```

Розкоментуйте правильний варіант для вашого випадку.
Не впевнені, який компас використовується з вашим модулем GPS? Виконайте наступні команди та перегляньте вивід:

```sh
sudo apt-get update
sudo apt-get install i2c-tools
i2cdetect -y 0
```

Приклад виводу:

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
