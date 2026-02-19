# [NarinFC-H7 VOLOLAND Inc.](https://vololand.com/pages/product/computer "NarinFC-H7")

## 소개

NarinFC-H7은 [VOLOLAND Inc.](https://vololand.com "VOLOLAND Inc.") 에서 생산하는 비행 컨트롤러입니다.

NarinFC-H7은 VOLOLAND CO., LTD.에서 자체 개발한 첨단 자동 조종 시스템 제품군입니다.

이 제품은 고성능 STM32H7 프로세서를 사용하고 산업용 등급 센서를 통합합니다.

기존 자동 조종 장치에 비해 성능이 향상되었고 신뢰성도 높아졌습니다.

![NarinFC-H7](../../assets/flight_controller/narinfc-h7/NarinFC_Header.jpg "NarinFC")

## 특징/사양

-  **프로세스**
    - STM32H743

-  **센서**
    - Accelerometer/Gyroscope: ADIS16470
    - Accelerometer/Gyroscope: ICM-20649
	- Accelerometer/Gyroscope: BMI088
	- Magnetometer: RM3100
	- Barometer: MS5611*2

-  **인터페이스**
    - 14 PWM servo outputs
	- Support multiple RC inputs (SBus / CPPM / DSM)
    - Analog/PWM RSSI input
	- 2 GPS ports (GPS and UART4 ports)
	- 4 ⅹ I2C buses
	- 2 ⅹ CAN bus ports
	- 2 ⅹ Power ports
	- 2 ⅹ ADC ports
	- 1 ⅹ USB port

-  **전원**
    - Power 4.3V ~ 5.4V
    - USB Input 4.75V ~ 5.25V

-  **크기**
    - 93.4mm x 46.4mm x 34.1mm
    - 106g

## 구매처

[VOLOLAND Inc.](https://vololand.com "VOLOLAND Inc.")

## 외형 치수

![Outline Dimensions](../../assets/flight_controller/narinfc-h7/2.Outline_Dimensions.png "Outline Dimensions")

## 배선도

![Wire Diagram](../../assets/flight_controller/narinfc-h7/3.Wire_Diagram.png "Wire Diagram")

## UART 매핑 (Port Diagram & Pin outs)
 각 시리얼 포트에 해당하는 UART와 기본 프로토콜은 아래와 같습니다.

  - SERIAL0 = USB (MAVLink2 default)
  - SERIAL1 = USART2,Telemetry1 (MAVlink2 default,DMA-enabled) 
  - SERIAL2 = USART6,Telemetry2 (MAVlink2 default,DMA-enabled) 
  - SERIAL3 = USART1,GPS1 (GPS default, DMA-enabled)
  - SERIAL4 = UART4,GPS2 (GPS2 default)
  - SERIAL5 = UART8 (not available except on custom carrier boards)(USER default,DMA-enabled)
  - SERIAL6 = UART7,DEBUG (USER)
  - SERIAL7 = USB2 (MAVLink2 default)

 직렬 프로토콜은 개인 선호도에 따라 조정할 수 있습니다.
  
![핀맵](../../assets/flight_controller/narinfc-h7/4.Port_Diagram_Pin_outs_Diagram-A.png "Port Diagram-A")

#### 1. TELEM1, TELEM2 Port

![TELEM Pinout](../../assets/flight_controller/narinfc-h7/4.1.TELEM1,TELEM2_Port_JST_GH_6P_Connector.png "TELEM Pinout")

  - JST GH 6P connector
  - TELEMETRY Port

#### 2. CAN1, CAN2 Port

![CAN Port](../../assets/flight_controller/narinfc-h7/4.2.CAN1,CAN2_Port_JST_HG_4P_Connector.png "CAN Port")

  - JST GH 4P connector
  - Communication Protocol: UAVCAN v0 (default), UAVCAN v1 (limited support)
  - Power Supply: Typically provides 5V or 12V output
  - Pin Configuration: Usually includes CAN_H, CAN_L, VCC, and GND

#### 3. I2C, I2C2, I2C3, I2C4 Port

![I2C Port](../../assets/flight_controller/narinfc-h7/4.3.I2C1,I2C2,I2C3,I2C4_Port_JST_GH_4P_Connector.png "I2C Port")

  - JST GH 4P connector

#### 4. UART4 Port

![UART Port](../../assets/flight_controller/narinfc-h7/4.4.UART4_Port_JST_GH_6P_Connector.png "UART Port")
  
  - JST GH 6P connector


#### 5. RSSI Port
  - RSSI input

![SPI Port](../../assets/flight_controller/narinfc-h7/13.RSSI.png "RSSI input")


#### 6. GPS & Safety Port

![GPS & Safety Port](../../assets/flight_controller/narinfc-h7/4.5.GPS_Safety_Port_JST_GH_10P_Connector.png "GPS & Safety Port")

  - JST GH 10P connector
  - GPS NODMA

![Port Diagram & Pin outs](../../assets/flight_controller/narinfc-h7/4.Port_Diagram_Pin_outs_Diagram-B.png "Port Diagram-B")

#### 7. PWM & RC_IN

NarinFC-H7은 최대 14개의 PWM 출력을 지원합니다. 출력은 그룹으로 묶이며, 그룹 내의 모든 출력은 동일한 프로토콜을 사용해야 합니다.

![PWM Out](../../assets/flight_controller/narinfc-h7/4.6.PWM_Out_M1-M14.png "PWM Out")
  
  - 2.54mm pitch DuPont connector
  - RC_IN : Remote control receiver

#### 8. Power Input

![Power Input](../../assets/flight_controller/narinfc-h7/4.7.Power_Input.png "Power Input")

  - 2mm pitch DuPont connector


#### 9. ADC Port
  - ADC input

![SPI Port](../../assets/flight_controller/narinfc-h7/12.ADC.png "ADC input")

#### 10. DEBUG/UART7 Port
UART7(SERIAL6) is labeled DEBUG RX/TX below

![DEBUG Port](../../assets/flight_controller/narinfc-h7/4.8.DEBUG_Port_JST_HG_6P_Connector.png "DEBUG Port")
  
  - JST GH 6P connector
  - DEBUG NODMA

#### 11. USB Port
  - USB C Type

#### 12. SPI Port

![SPI Port](../../assets/flight_controller/narinfc-h7/4.10.SPI_Port_JST_GH_7P_Connector.png "SPI Port")

  - JST GH 7P connector
  - SPI Port

#### 13. SD CARD
  - SD CARD

## PWM Output

NarinFC-H7은 최대 14개의 PWM 출력을 지원합니다. M13 및 M14 출력을 제외한 모든 출력은 DShot을 지원합니다. 출력 1~8은 양방향 DShot을 지원합니다.

The 14 PWM outputs are in 4 groups:
  - Outputs 1, 2, 3 and 4 in group1
  - Outputs 5, 6, 7 and 8 in group2
  - Outputs 9, 10, 11 and 12 in group3
  - Outputs 13 and 14 in group4

같은 그룹 내의 모든 출력은 동일한 출력 속도와 프로토콜을 사용해야 합니다.

## GPIOs

14개의 출력은 GPIO(릴레이, 버튼, RPM 등)로 사용할 수 있습니다. 사용하려면 출력값을 SERVOx_FUNCTION-1로 설정해야 합니다. 자세한 내용은 GPIO 페이지를 참조하십시오.

PX4의 PIN 매개변수에 사용되는 GPIO의 번호 매기기는 다음과 같습니다.

- PWM1(M1) 50
- PWM2(M2) 51
- PWM3(M3) 52
- PWM4(M4) 53
- PWM5(M5) 54
- PWM6(M6) 55
- PWM7(M7) 56
- PWM8(M8) 57
- PWM9(M9) 58
- PWM10(M10) 59
- PWM11(M11) 60
- PWM12(M12) 61
- PWM13(M13) 62
- PWM14(M14) 63

## Analog inputs

NarinFC-H7은 6V 내성 아날로그 입력 하나와 3.3V 내성 아날로그 입력 하나, 총 2개의 아날로그 입력 단자를 갖추고 있습니다.

- ADC Pin16 -> BATT_VOLTAGE_SENS
- ADC Pin17 -> BATT_CURRENT_SENS
- ADC Pin14 -> BATT2_VOLTAGE_SENS
- ADC Pin2  -> BATT2_VOLTAGE_SENS
- ADC Pin4  -> SPARE1_ADC1(6.6V)
- ADC Pin18 -> SPARE2_ADC1(3.3V)
- ADC Pin6  -> RSSI_IN_ADC1(3.3V)
- ADC Pin8  -> VDD_5V_SENS
- ADC Pin11 -> SCALED_V3V3

## Battery Monitor

보드에는 6핀 커넥터에 연결된 두 개의 전용 전원 모니터링 포트가 있습니다. 올바른 배터리 설정 매개변수는 연결된 전원 어댑터의 종류에 따라 달라집니다. 기본적으로 CAN 배터리 모니터링 기능이 활성화되어 있습니다.

## RC Input

기본적으로 타이머 입력으로 매핑되는 RCIN 핀은 PX4에서 지원하는 모든 단방향 수신 프로토콜에 사용할 수 있습니다. CRSF/ELRS 및 SRXL2와 같은 양방향 프로토콜은 전체 UART 연결이 필요합니다. FPort를 RCIN에 연결하면 원격 측정 정보 없이 RC 신호만 제공됩니다.

Fport, CRSF 및 SRXL2 수신기에서 사용 가능한 CRSF 및 내장 텔레메트리를 사용하려면 수신기 연결에 SERIAL6(UART7)과 같은 전체 UART를 사용해야 합니다. 아래는 Serial6을 사용한 설정 예시입니다.  

- :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>` should be set to "23".
- FPort would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "15".
- CRSF would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "0".
- SRXL2 would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "4" and connects only the TX pin.

모든 UART를 RC 시스템 연결에 사용할 수 있으며, PPM을 제외한 모든 프로토콜과 호환됩니다. common-rc-systems자세한 내용은 :ref:를 참조하십시오. 이 커넥터 위치와 연결된 전원 레일은 USB 또는 PMU를 통해 전원이 공급됩니다.

## Loading Firmware

이 보드에는 PX4 펌웨어가 사전 설치되어 있으며, 대부분의 지상 제어 스테이션을 사용하여 다른 차량/버전의 PX4 펌웨어를 로드할 수 있습니다.

이 보드에는 PX4 부트로더가 사전 설치되어 있어 지상국을 사용하여 펌웨어 파일을 로드할 수 있습니다.

<br>

# [VOLOLAND Inc.](https://vololand.com "VOLOLAND Inc.")

