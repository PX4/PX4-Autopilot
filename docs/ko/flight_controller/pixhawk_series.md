# Pixhawk 시리즈

[Pixhawk<sup>&reg;</sup>](https://pixhawk.org/) is an independent open-hardware project providing readily-available, low-cost, and high-end, _autopilot hardware designs_ to the academic, hobby and industrial communities.

Pixhawk is the reference hardware platform for PX4, and runs PX4 on the [NuttX](https://nuttx.apache.org/) OS.

제조업체는화물 운반에서 1인칭 시점(FPV) 레이서에 이르는 애플리케이션에 최적화 된 폼 팩터를 사용하여 개방형 디자인을 기반으로 다양한 보드를 제조하였습니다.

:::tip
For computationally intensive tasks (e.g. computer vision) you will need a separate companion computer (e.g. [Raspberry Pi 2/3 Navio2](../flight_controller/raspberry_pi_navio2.md)) or a platform with an integrated companion solution.
:::

## 주요 장점

Key benefits of using a _Pixhawk series_ controller include:

- 소프트웨어 지원 - PX4 기준 하드웨어로서 가장 잘 관리되는 보드입니다.
- 장착 가능한 하드웨어 주변 장치의 유연성.
- 고품질
- 폼 팩터 측면에서 정밀하게 수정할 수 있습니다.
- 널리 사용되어 잘 테스트되고 안정적입니다.
- Automated update of latest firmware via _QGroundControl_ (end-user friendly).

## 지원 보드

The PX4 Project uses [Pixhawk Standard Autopilots](../flight_controller/autopilot_pixhawk_standard.md) as reference hardware.
Pixhawk 표준(상표 사용 포함)과 호환는 컨트롤러입니다.

:::info
The PX4 maintenance and test teams maintain and support these standard boards.
:::

Pixhawk-like boards that are not fully compliant with the specification may be [manufacturer-supported](../flight_controller/autopilot_manufacturer_supported.md), [experimental/discontinued](../flight_controller/autopilot_experimental.md), or unsupported.

이 섹션의 나머지 부분에서는 Pixhawk 시리즈에 대해 조금 더 설명하지만 반드시 읽어야하는 것은 아닙니다.

## 배경

The [Pixhawk project](https://pixhawk.org/) creates open hardware designs in the form of schematics, which define a set of components (CPU, sensors, etc.) and their connections/pin mappings.

Manufacturers are encouraged to take the [open designs](https://github.com/pixhawk/Hardware) and create products that are best suited to a particular market or use case (the physical layout/form factor not part of the open specification). 동일한 디자인의 보드는 바이너리 수준에서 호환이 가능합니다.

:::info
While a physical connector standard is not mandated, newer products generally follow the [Pixhawk Connector Standard](https://pixhawk.org/pixhawk-connector-standard/).
:::

The project also creates reference autopilot boards based on the open designs, and shares them under the same [licence](#licensing-and-trademarks).

<a id="fmu_versions"></a>

### FMU 버전

Pixhawk 프로젝트는 다양한 개방형 회로도를 디자인하였습니다.
같은 디자인의 보드들은 동일한 펌웨어가 바이너리 수준에서 호환되어야 합니다.

각 디자인의 이름은 FMUvX(예 : FMUv1, FMUv2, FMUv3, FMUv4 등)입니다.
FMU 번호가 높을 수록 보드가 최신 버전이며, 반드시 성능 향상을 의미하지는 않습니다. 버전이 거의 동일할 수 있으며 커넥터 배선만 다를 수 있습니다.

PX4 _users_ generally do not need to know very much about FMU versions:

- _QGroundControl_ automatically downloads the correct firmware for a connected autopilot (based on its FMU version "under the hood").
- 일반적으로 FMU 버전이 아닌 물리적 제약과 폼 팩터에 의해서 컨트롤러를 선택합니다.

:::info
The exception is that if you're using FMUv2 firmware it is [limited to 1MB of flash](../flight_controller/silicon_errata.md#fmuv2-pixhawk-silicon-errata).
이 제한된 공간에 PX4를 맞추기 위하여, 다수의 모듈들이 비활성화되어 있습니다.
You may find that some [parameters are missing](../advanced_config/parameters.md#missing) and that some hardware does not work "out of the box".
:::

PX4 _developers_ need to know the FMU version of their board, as this is required to build custom hardware.

주요 차이점은 아래와 같습니다.

- **FMUv2:** Single board with STM32427VI processor ([Pixhawk 1 (Discontinued)](../flight_controller/pixhawk.md), [pix32](../flight_controller/holybro_pix32.md), [Pixfalcon](../flight_controller/pixfalcon.md), [Drotek DroPix](../flight_controller/dropix.md))
- **FMUv3:** Identical to FMUv2, but usable flash doubled to 2MB ([Hex Cube Black](../flight_controller/pixhawk-2.md),[CUAV Pixhack v3](../flight_controller/pixhack_v3.md),[mRo Pixhawk](../flight_controller/mro_pixhawk.md), [Pixhawk Mini (Discontinued)](../flight_controller/pixhawk_mini.md))
- **FMUv4:** Increased RAM. 더 빨라진 CPU. 더 많은 직렬 포트. No IO processor ([Pixracer](../flight_controller/pixracer.md))
- **FMUv4-PRO:** Slightly increased RAM. 더 많은 직렬 포트. IO processor ([Pixhawk 3 Pro](../flight_controller/pixhawk3_pro.md))
- **FMUv5:** New processor (F7).
  훨씬 더 빨라짐.
  더 많은 RAM.
  More CAN buses.
  Much more configurable.
  ([Pixhawk 4](../flight_controller/pixhawk4.md),[CUAV v5](../flight_controller/cuav_v5.md),[CUAV V5+](../flight_controller/cuav_v5_plus.md),[CUAV V5 nano](../flight_controller/cuav_v5_nano.md))
- **FMUv5X:** New processor (F7).
  Much faster, Modular design.
  More reliable.
  More Redundancy.
  더 많은 RAM.
  More CAN buses.
  Much more configurable & customizable.
  ([Pixhawk 5X](../flight_controller/pixhawk5x.md), Skynode)
- **FMUv6C:**
  ([Holybro Pixhawk 6C Mini](../flight_controller/pixhawk6c_mini.md), [Holybro Pixhawk 6C](../flight_controller/pixhawk6c.md))
- **FMUv6X:**
  ([CUAV Pixhawk V6X](../flight_controller/cuav_pixhawk_v6x.md),[Holybro Pixhawk 6X](../flight_controller/pixhawk6x.md))
- **FMUv6X-RT:** Faster MCU core (1GHz) (vs 480Mhz on 6X).
  More RAM (2Mb).
  More flash (64Mb) (2Mb on v6X/v5X).
  ([Holybro Pixhawk 6X-RT](../flight_controller/pixhawk6x-rt.md))

<a id="licensing-and-trademarks"></a>

### 라이선스와 상표

Pixhawk project schematics and reference designs are licensed under [CC BY-SA 3](https://creativecommons.org/licenses/by-sa/3.0/legalcode).

The license allows you to use, sell, share, modify and build on the files in almost any way you like - provided that you give credit/attribution, and that you share any changes that you make under the same open source license (see the [human readable version of the license](https://creativecommons.org/licenses/by-sa/3.0/) for a concise summary of the rights and obligations).

:::info
Boards that are _derived directly_ from Pixhawk project schematic files (or reference boards) must be open sourced.
독점 제품으로 상업적 라이센스를 받을 수 없습니다.
:::

Manufacturers can create (compatible) _fully independent products_ by first generating fresh schematic files that have the same pin mapping/components as the FMU designs.
독립적으로 제작된 회로도를 기반으로하는 제품은 원본 작품으로 간주되며, 필요에 따라 라이센스를 받을 수 있습니다.

제품 이름/브랜드도 상표가 될 수 있습니다.
상표명은 소유자의 허가없이 사용할 수 없습니다.

:::tip
_Pixhawk_ is a trademark, and cannot be used in product names without permission.
:::

## 추가 정보

### LED

All _Pixhawk-series_ flight controllers support:

- A user facing RGB _UI LED_ to indicate the current _readiness to fly_ status of the vehicle. This is typically a superbright I2C peripheral, which may or may not be mounted on the board (i.e. FMUv4 does not have one on board and typically uses an LED mounted on the GPS).
- Three _Status LED_s that provide lower level power status, bootloader mode and activity, and error information.

To interpret the LEDs see: [LED Meanings](../getting_started/led_meanings.md).
