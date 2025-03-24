# PilotPi з Ubuntu Server

:::warning
Ubuntu Server on RPi 4B consumes a lot of current and generates a lot of heat.
Розробляйте дизайн для кращого відведення тепла та високого енергоспоживання при використанні цього обладнання.
:::

## Швидкий старт для розробника

### OS Image

Підтримуються і armhf і arm64.

#### armhf

- [Ubuntu Server 18.04.5 for RPi2](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-armhf+raspi2.img.xz)
- [Ubuntu Server 18.04.5 for RPi3](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-armhf+raspi3.img.xz)
- [Ubuntu Server 18.04.5 for RPi4](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-armhf+raspi4.img.xz)
- [Ubuntu Server 20.04.1 for RPi 2/3/4](https://cdimage.ubuntu.com/releases/20.04.1/release/ubuntu-20.04.2-preinstalled-server-arm64+raspi.img.xz)

#### arm64

- [Ubuntu Server 18.04.5 for RPi3](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz)
- [Ubuntu Server 18.04.5 for RPi4](https://cdimage.ubuntu.com/releases/18.04.5/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz)
- [Ubuntu Server 20.04.1 for RPi 3/4](https://cdimage.ubuntu.com/releases/20.04.1/release/ubuntu-20.04.2-preinstalled-server-arm64+raspi.img.xz)

#### Найновіша ОС

Please refer to official [cdimage](https://cdimage.ubuntu.com/releases/) page for any new updates.

### Перший запуск

When setting up RPi's WiFi for the first time we recommended using a wired Ethernet connection between your home router and RPi, and a monitor and keyboard.

#### Перед завантаженням

Встановіть SD-карту на ваш комп'ютер та змініть налаштування мережі.
Please follow the official instruction [here](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet).

Тепер під'єднайте SD-карту до вашого Pi та завантажтеся вперше.
Переконайтеся, що у вас є доступ до RPi через командний рядок - або через з'єднання SSH по кабелю Ethernet, або безпосередній доступ за допомогою клавіатури та монітора.

#### WiFi регіон

Спочатку встановіть необхідний пакет:

```sh
sudo apt-get install crda
```

Edit the file `/etc/default/crda` to change the correct WiFi region. [Reference List](https://www.arubanetworks.com/techdocs/InstantWenger_Mobile/Advanced/Content/Instant%20User%20Guide%20-%20volumes/Country_Codes_List.htm)

```sh
sudo nano /etc/default/crda
```

Потім ваш Pi зможе приєднатися до вашої мережі WiFi після перезавантаження.

#### Hostname та mDNS

Давайте спочатку налаштуємо ім'я хоста.

```sh
sudo nano /etc/hostname
```

Змініть значення hostname на те, що вам подобається.
Потім встановіть пакет, необхідний для mDNS:

```sh
sudo apt-get update
sudo apt-get install avahi-daemon
```

Виконайте перезавантаження.

```sh
sudo reboot
```

Відновіть доступність через підключення WiFi після вищезазначеної операції.

```sh
ssh ubuntu@pi_hostname.local
```

#### Автентифікація без пароля (необов'язково)

You may want to setup [passwordless auth](https://www.raspberrypi.org/documentation/remote-access/ssh/passwordless.md) as well.

### Налаштування OS

#### config.txt

The corresponding file in Ubuntu is `/boot/firmware/usercfg.txt`.

```sh
sudo nano /boot/firmware/usercfg.txt
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

В Ubuntu Server 20.04:

```sh
sudo nano /boot/firmware/cmdline.txt
```

On Ubuntu Server 18.04 or earlier, `nobtcmd.txt` and `btcmd.txt` should both be modified.

```sh
sudo nano /boot/firmware/nobtcmd.txt
```

Find `console=/dev/ttyAMA0,115200` and remove that part to disable the login shell on serial interface.

Append `isolcpus=2` after the last word.
Весь файл буде виглядати так:

```sh
net.ifnames=0 dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 elevator=deadline rootwait fixrtc isolcpus=2
```

Вищезазначений рядок вказує ядру Linux не планувати жодного процесу на ядрі CPU 2.
Ми пізніше вручну запустимо PX4 на цьому ядрі.

Перезавантажте та увійдіть за допомогою SSH на ваш Pi.

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

Перевірте інтерфейс SPI:

```sh
ls /dev/spidev*
```

There should be `/dev/spidev0.0`.

#### rc.local

In this section we will configure the auto-start script in **rc.local**.
Зверніть увагу, що нам потрібно створити цей файл, оскільки він відсутній у свіжій операційній системі Ubuntu.

```sh
sudo nano /etc/rc.local
```

Додати вміст нижче до файлу:

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

Збережіть та вийдіть.
Потім встановіть правильні права:

```sh
sudo chmod +x /etc/rc.local
```

:::info
Don't forget to turn off the switch when it is not needed!
:::

#### CSI камера

:::warning
Enable CSI camera will stop anything works on I2C-0.
:::

```sh
sudo nano /boot/firmware/usercfg.txt
```

Додайте наступний рядок в кінці файлу:

```sh
start_x=1
```

### Збірка коду

To get the _very latest_ version onto your computer, enter the following command into a terminal:

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

:::info
This is all you need to do just to build the latest code.
:::

#### Встановити точку призначення завантаження RPi

Встановіть IP-адресу (або ім'я хоста) вашого RPi за допомогою:

```sh
export AUTOPILOT_HOST=192.168.X.X
```

або

```sh
export AUTOPILOT_HOST=pi_hostname.local
```

Додатково, нам потрібно встановити ім'я користувача:

```sh
export AUTOPILOT_USER=ubuntu
```

#### Збірка для архітектури armhf

Зберіть виконуваний файл:

```sh
cd Firmware
make scumaker_pilotpi_default
```

Потім завантажте його за допомогою:

```sh
make scumaker_pilotpi_default upload
```

#### Альтернативний метод збірки для armhf (з використанням Docker)

If you are compiling for the first time with docker, please refer to the [official docs](../test_and_ci/docker.md#prerequisites).

Виконайте команду в директорії firmware:

```sh
./Tools/docker_run.sh "export AUTOPILOT_HOST=192.168.X.X; export AUTOPILOT_USER=ubuntu; export NO_NINJA_BUILD=1; make scumaker_pilotpi_default upload"
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

#### Збірка для архітектури arm64

:::info
This step requires `aarch64-linux-gnu` tool-chain to be installed.
:::

Зберіть виконуваний файл:

```sh
cd PX4-Autopilot
make scumaker_pilotpi_arm64
```

Потім завантажте його за допомогою:

```sh
make scumaker_pilotpi_arm64 upload
```

#### Альтернативний метод збірки для arm64 (з використанням Docker)

If you are compiling for the first time with docker, please refer to the [official docs](../test_and_ci/docker.md#prerequisites).

Execute the command in `PX4-Autopilot` folder:

```sh
./Tools/docker_run.sh "export AUTOPILOT_HOST=192.168.X.X; export AUTOPILOT_USER=ubuntu; export NO_NINJA_BUILD=1; make scumaker_pilotpi_arm64 upload"
```

:::info
mDNS is not supported within docker. Вам потрібно вказувати правильну IP-адресу кожного разу під час завантаження.
:::

:::info
If your IDE doesn't support ninja build, `NO_NINJA_BUILD=1` option will help.
You can compile without uploading too - just remove the `upload` target.
:::

Також можливо просто скомпілювати код за допомогою команди:

```sh
./Tools/docker_run.sh "make scumaker_pilotpi_arm64"
```

#### Ручний запуск PX4

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
rm -rf build/scumaker_pilotpi_*
```

Потім поверніться до відповідного розділу вище.

### Післяконфігурація

Please refer to the instructions [here](raspberry_pi_pilotpi_rpios.md)
