# Black Magic Probe (and Dronecode Probe)

The [Black Magic Probe](https://black-magic.org) is an easy to use, mostly plug-and-play, JTAG/SWD debugger for embedded microcontrollers.
Since the Black Magic Probe is a generic debug probe, you will need an adapter to connect to Pixhawk flight controllers, which can be purchased here:

- [Drone Code Debug Adapter](https://1bitsquared.com/products/drone-code-debug-adapter) (1 BIT SQUARED).

## Dronecode Probe

The [Dronecode Probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) is a specialization of the Black Magic Probe for debugging PX4 autopilots.

The probe's USB interface exposes two separate virtual serial port interfaces: one for connecting to the [System Console](system_console.md) (UART) and the other for an embedded GDB server (SWD interface).

The probe provides a DCD-M connector cable for attaching to the [Pixhawk Debug Mini](swd_debug.md#pixhawk-debug-mini).

:::info
The _6-pos DF13_ connector that comes with the probe cannot be used for SWD debugging (it is for using the System Console).
:::

## Using the Probe

:::info
To debug STM32F7 or later (FMUv5 and newer) the Dronecode probe / Blackmagic probe likely requires a firmware update.
You can find how to update the [blackmagic probe here](https://github.com/blacksphere/blackmagic/wiki/Upgrading-Firmware).
:::

GDB와 함께 Dronecode 프로브를 사용하려면, 현재 자동조종장치에서 플래싱된 정확한 ELF 파일로 GDB를 시작하십시오.

```sh
arm-none-eabi-gdb build/px4_fmu-v5_default/px4_fmu-v5_default.elf
```

그런 다음, Dronecode 프로브 인터페이스를 선택하여야 합니다. Linux에서는 다음과 같습니다.

```sh
target ext /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_f9414d5_7DB85DAC-if00
```

그런 다음 대상을 스캔합니다.

```sh
monitor swdp_scan
```

다음과 같은 내용이 표시되어야 합니다.

```sh
Target voltage: 3.3V
Available Targets:
No. Att Driver
 1      STM32F76x M7
```

일부 자동조종장치는 0.0V를 표시하지만, 후속 단계는 그럼에도 불구하고 작동합니다.

이제 해당 대상에 연결할 수 있습니다.

```sh
attach 1
```

이제 연결되어야 합니다.
