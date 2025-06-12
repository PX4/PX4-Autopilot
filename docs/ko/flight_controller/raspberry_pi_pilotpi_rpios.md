# 라즈베리 파이 OS 기반 파일럿파이

## 개발자 가이드

### OS 이미지

The latest official [Raspberry Pi OS Lite](https://downloads.raspberrypi.org/raspios_lite_armhf_latest) image is always recommended.

설치를 위히 라즈베리파이에 SSH 연결이 가능하여야 합니다.

### 접근 설정 (선택 사항)

#### 호스트명과 mDNS

mDNS helps you connect to your RPi with hostname instead of IP address.

```sh
sudo raspi-config
```

Navigate to **Network Options > Hostname**.
설정하고 종료합니다.
You may want to setup [passwordless auth](https://www.raspberrypi.org/documentation/remote-access/ssh/passwordless.md) as well.

### 운영체제 설정

#### config.txt

```sh
sudo nano /boot/config.txt
```

파일을 다음의 내용으로 변경합니다.

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
로그인 셸없이 UART를 활성화합니다.

```sh
sudo nano /boot/cmdline.txt
```

Append `isolcpus=2` after the last word.
전체 파일은 다음과 같습니다.

```sh
console=tty1 root=PARTUUID=xxxxxxxx-xx rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait isolcpus=2
```

이것은 리눅스 커널이 CPU 코어 2에서 프로세스를 예약하지 않도록 지시합니다.
나중에 해당 코어에서 PX4를 수동으로 실행합니다.

Reboot and SSH onto your RPi.

UART 인터페이스를 확인합니다.

```sh
ls /dev/tty*
```

There should be `/dev/ttyAMA0`, `/dev/ttySC0` and `/dev/ttySC1`.

I2C 인터페이스를 확인합니다.

```sh
ls /dev/i2c*
```

There should be `/dev/i2c-0` and `/dev/i2c-1`

SPI 인터페이스를 확인합니다.

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

저장후 종료합니다.

:::info
Don't forget to turn off the switch when it is not needed.
:::

#### CSI 카메라

:::info
Enable CSI camera will stop anything works on I2C-0.
:::

```sh
sudo raspi-config
```

**Interfacing Options > Camera**

### 코드 빌드

To get the _very latest_ version onto your computer, enter the following command into a terminal:

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

:::info
This is all you need to do just to build the latest code.
:::

#### Raspberry Pi OS용 크로스 빌드

다음을 사용하여 라즈베리파이의 IP(또는 호스트 이름)를 설정합니다.

```sh
export AUTOPILOT_HOST=192.168.X.X
```

또는

```sh
export AUTOPILOT_HOST=pi_hostname.local
```

실행 파일을 빌드하십시오.

```sh
cd PX4-Autopilot
make scumaker_pilotpi_default
```

다음 명령으로 업로드하십시오.

```sh
make scumaker_pilotpi_default upload
```

ssh에서 다음을 명령어를 실행하십시오.

```sh
cd px4
sudo taskset -c 2 ./bin/px4 -s pilotpi_mc.config
```

이제 PX4는 다중로터 설정으로 시작합니다.

If you encountered the similar problem executing `bin/px4` on your Pi as following:

```
bin/px4: /lib/xxxx/xxxx: version `GLIBC_2.29' not found (required by bin/px4)
```

docker로 컴파일하여야 합니다.

다음 단계로 진행하기 전에 먼저 기존 빌드를 삭제합니다.

```sh
rm -rf build/scumaker_pilotpi_default
```

### 대체 빌드 방법 (도커 사용)

다음 방법은 CI에 배포된 동일한 도구 세트를 제공할 수 있습니다.

If you are compiling for the first time with docker, please refer to the [official docs](../test_and_ci/docker.md#prerequisites).

PX4-Autopilot 폴더에서 다음 명령을 실행합니다.

```sh
./Tools/docker_run.sh "export AUTOPILOT_HOST=192.168.X.X; export NO_NINJA_BUILD=1; make scumaker_pilotpi_default upload"
```

:::info
mDNS is not supported within docker. 업로드시에 올바른 IP 주소를 설정하여야합니다.
:::

:::info
If your IDE doesn't support ninja build, `NO_NINJA_BUILD=1` option will help.
업로드하지 않고도 컴파일할 수 있습니다. Just remove `upload` target.
:::

다음 명령으로 코드를 컴파일합니다.

```sh
./Tools/docker_run.sh "make scumaker_pilotpi_default"
```

### 사후 설정

기체가 제대로 작동하려면 이러한 추가 항목을 확인하여야 합니다.

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

사용자의 환경에 맞추어 주석을 적절하게 제거하십시오.
GPS 모듈과 함께 제공되는 나침반이 확실하지 않습니까? 다음 명령을 실행하고 출력을 확인합니다.

```sh
sudo apt-get update
sudo apt-get install i2c-tools
i2cdetect -y 0
```

샘플 출력:

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
