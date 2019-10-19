/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * ModalAI FC-v1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/

// NA on ModalAI FC-v1

/* PX4FMU GPIOs ***********************************************************************************/

#undef TRACE_PINS

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V or used as TRACE0-2 */

#if !defined(TRACE_PINS)
#  define GPIO_nLED_RED        /* PB0 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_nLED_GREEN      /* PB1 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_nLED_BLUE       /* PA7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN7)

#  define GPIO_nLED_2_RED      /* PI0 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN0)
#  define GPIO_nLED_2_GREEN    /* PH11 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN11)
#  define GPIO_nLED_2_BLUE     /* PA2 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN2)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE
#endif

/* SPI
 *
 * SPI1 is sensors1
 *  ICM-20602
 *    CS        PI9
 *    DRDY      PF2
 *
 * SPI2 is sensors2
 *  ICM-42688
 *    CS        PH5
 *    DRDY      PH12
 *
 * SPI3 is not used
 *
 * SPI4 is not used
 *
 * SPI5 is FRAM
 *  FM25V02A
 *    CS        PG7
 *
 * SPI6 is sensors3
 *  BMI088
 *    CS1       PI10
 *    CS2       PA15
 *    DRDY1     PI6
 *    DRDY2     PI7
 */

#define PX4_SPI_BUS_SENSORS1   1
#define PX4_SPI_BUS_SENSORS2   2
// SPI 3 not used
// SPI 4 not used
#define PX4_SPI_BUS_MEMORY     5
#define PX4_SPI_BUS_SENSORS3   6

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

/* SPI 1 CS */

#define GPIO_SPI1_nCS1_ICM20602     /* PI9 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN9)

/*  Define the SPI1 Data Ready interrupts */

#define GPIO_SPI1_DRDY1_ICM20602    /* PF2  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTF|GPIO_PIN2)

/*  SPI1 off */

#define GPIO_SPI1_SCK_OFF   _PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF  _PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(GPIO_SPI1_MOSI)

#define GPIO_DRDY_OFF_SPI1_DRDY1_ICM20602    _PIN_OFF(GPIO_SPI1_DRDY1_ICM20602)

/* SPI 2 CS */

#define GPIO_SPI2_nCS1_ICM_42688       /* PH5   */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN5)

/*  Define the SPI2 Data Ready interrupts */

#define GPIO_SPI2_DRDY1_ICM_42688      /* PH12  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTH|GPIO_PIN12)

/*  SPI2 off */

#define GPIO_SPI2_SCK_OFF   _PIN_OFF(GPIO_SPI2_SCK)
#define GPIO_SPI2_MISO_OFF  _PIN_OFF(GPIO_SPI2_MISO)
#define GPIO_SPI2_MOSI_OFF  _PIN_OFF(GPIO_SPI2_MOSI)

#define GPIO_DRDY_OFF_SPI2_DRDY1_ICM_42688    _PIN_OFF(GPIO_SPI2_DRDY1_ICM_42688)

/* SPI 5 CS */

#define GPIO_SPI5_nCS1_FRAM          /* PG7  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN7)

/* SPI 6 CS */

#define GPIO_SPI6_nCS1_BMI088       /* PI10  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN10)
#define GPIO_SPI6_nCS2_BMI088       /* PA15  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)

/*  Define the SPI6 Data Ready interrupts */

#define GPIO_SPI6_DRDY1_BMI088_INT1_ACCEL /* PI6  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN6)
#define GPIO_SPI6_DRDY2_BMI088_INT3_GYRO  /* PI7  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN7)

/*  Define the SPI6 Data Ready interrupts */

#define GPIO_DRDY_OFF_SPI6_DRDY1_BMI088    _PIN_OFF(GPIO_SPI6_DRDY1_BMI088_INT1_ACCEL)#define GPIO_DRDY_OFF_SPI6_DRDY2_BMI088    _PIN_OFF(GPIO_SPI6_DRDY2_BMI088_INT3_GYRO)

/*  SPI6 off */

#define GPIO_SPI6_SCK_OFF   _PIN_OFF(GPIO_SPI6_SCK)
#define GPIO_SPI6_MISO_OFF  _PIN_OFF(GPIO_SPI6_MISO)
#define GPIO_SPI6_MOSI_OFF  _PIN_OFF(GPIO_SPI6_MOSI)

#define GPIO_DRDY_OFF_SPI6_DRDY1    _PIN_OFF(GPIO_DRDY_OFF_SPI6_DRDY1_BMI088)
#define GPIO_DRDY_OFF_SPI6_DRDY2    _PIN_OFF(GPIO_DRDY_OFF_SPI6_DRDY2_BMI088)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))
#define PX4_SPI_BUS_RAMTRON  PX4_SPI_BUS_MEMORY

#define PX4_SPIDEV_ICM_20602        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS1,0)
#define PX4_SENSORS1_BUS_CS_GPIO    {GPIO_SPI1_nCS1_ICM20602}

#define PX4_SPIDEV_ICM_42688        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS2,0)
#define PX4_SENSORS2_BUS_CS_GPIO    {GPIO_SPI2_nCS1_ICM_42688}

#define PX4_SPIDEV_MEMORY           PX4_MK_SPI_SEL(PX4_SPI_BUS_MEMORY,0)
#define PX4_MEMORY_BUS_CS_GPIO      {GPIO_SPI5_nCS1_FRAM}

#define PX4_SPIDEV_BMI088_GYR       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS3,0)
#define PX4_SPIDEV_BMI088_ACC       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS3,1)
#define PX4_SENSORS3_BUS_CS_GPIO    {GPIO_SPI6_nCS2_BMI088, GPIO_SPI6_nCS1_BMI088}

/* I2C busses */

#define PX4_I2C_BUS_EXPANSION       1
#define PX4_I2C_BUS_EXPANSION1      2
#define PX4_I2C_BUS_EXPANSION2      3
#define PX4_I2C_BUS_ONBOARD         4
#define PX4_I2C_BUS_LED             PX4_I2C_BUS_EXPANSION

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_BMP388        0x76
#define PX4_I2C_OBDEV_A71CH         0x49

#define BOARD_NUMBER_I2C_BUSES      4
#define BOARD_I2C_BUS_CLOCK_INIT    {100000, 100000, 100000, 100000}

#define GPIO_I2C4_DRDY1_BMP388      /* PG5  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN5)
#define GPIO_A71CH_nRST             /* PH3  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN3)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n)                GPIO_ADC1_IN##n
#define ADC3_CH(n)                  (n)
#define ADC3_GPIO(n)                GPIO_ADC3_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */

#define PX4_ADC_GPIO  \
	/* PC0 */  ADC1_GPIO(10), \
	/* PC1 */  ADC1_GPIO(11), \
	/* PF4 */  ADC3_GPIO(14), \
	/* PF5 */  ADC3_GPIO(15)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_SCALED_V5_CHANNEL                   /* PC0 */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL      /* PC1 */  ADC1_CH(11)
#define ADC_HW_VER_SENSE_CHANNEL                /* PF4 */  ADC3_CH(14)
#define ADC_HW_REV_SENSE_CHANNEL                /* PF5 */  ADC3_CH(15)

#define ADC_CHANNELS \
	((1 << ADC_SCALED_V5_CHANNEL) | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL) | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define GPIO_HW_VER_REV_DRIVE  /* PA4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define GPIO_HW_REV_SENSE      /* PF5 */  ADC3_GPIO(15)
#define GPIO_HW_VER_SENSE      /* PF4 */  ADC3_GPIO(14)
#define HW_INFO_INIT           {'V','1','x', 'x',0}
#define HW_INFO_INIT_VER       2 /* Offset in above string of the VER */
#define HW_INFO_INIT_REV       3 /* Offset in above string of the REV */

/* PWM
 *
 * 8  PWM outputs are configured.
 *
 * Pins:
 *
 * FMU_CH1 : PE14 : TIM1_CH4
 * FMU_CH2 : PA10 : TIM1_CH3
 * FMU_CH3 : PE11 : TIM1_CH2
 * FMU_CH4 : PA8  : TIM1_CH1
 * FMU_CH5 : PD13 : TIM4_CH2
 * FMU_CH6 : PD14 : TIM4_CH3
 * FMU_CH7 : PD12 : TIM4_CH1
 * FMU_CH8 : PH15 : TIM4_CH4
 *
 */
#define GPIO_TIM4_CH4OUT      /* PH15  T4C4   FMU8 */ GPIO_TIM4_CH4OUT_2
#define GPIO_TIM4_CH1OUT      /* PD12  T4C1   FMU7 */ GPIO_TIM4_CH1OUT_2
#define GPIO_TIM4_CH3OUT      /* PD14  T4C3   FMU6 */ GPIO_TIM4_CH3OUT_2
#define GPIO_TIM4_CH2OUT      /* PD13  T4C2   FMU5 */ GPIO_TIM4_CH2OUT_2
#define GPIO_TIM1_CH1OUT      /* PA8   T1C1   FMU4 */ GPIO_TIM1_CH1OUT_1
#define GPIO_TIM1_CH2OUT      /* PE11  T1C2   FMU3 */ GPIO_TIM1_CH2OUT_2
#define GPIO_TIM1_CH3OUT      /* PA10  T1C3   FMU2 */ GPIO_TIM1_CH3OUT_1
#define GPIO_TIM1_CH4OUT      /* PE14  T1C4   FMU1 */ GPIO_TIM1_CH4OUT_2

#define DIRECT_PWM_OUTPUT_CHANNELS  8

#define GPIO_TIM4_CH4IN       /* PD15  T4C4   FMU8 */ GPIO_TIM4_CH4IN_2
#define GPIO_TIM4_CH1IN       /* PD12  T4C1   FMU7 */ GPIO_TIM4_CH1IN_2
#define GPIO_TIM4_CH3IN       /* PD14  T4C3   FMU6 */ GPIO_TIM4_CH3IN_2
#define GPIO_TIM4_CH2IN       /* PD13  T4C2   FMU5 */ GPIO_TIM4_CH2IN_2
#define GPIO_TIM1_CH1IN       /* PA8   T1C1   FMU4 */ GPIO_TIM1_CH1IN_1
#define GPIO_TIM1_CH2IN       /* PE11  T1C2   FMU3 */ GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN       /* PA10  T1C3   FMU2 */ GPIO_TIM1_CH3IN_1
#define GPIO_TIM1_CH4IN       /* PE14  T1C4   FMU1 */ GPIO_TIM1_CH4IN_2

#define DIRECT_INPUT_TIMER_CHANNELS  8

/* User GPIOs
 *
 */

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))

#define GPIO_GPIO7_INPUT        /* PD15  T4C4   FMU8 */ _MK_GPIO_INPUT(GPIO_TIM4_CH4IN)
#define GPIO_GPIO6_INPUT        /* PD12  T4C1   FMU7 */ _MK_GPIO_INPUT(GPIO_TIM4_CH1IN)
#define GPIO_GPIO5_INPUT        /* PD14  T4C3   FMU6 */ _MK_GPIO_INPUT(GPIO_TIM4_CH3IN)
#define GPIO_GPIO4_INPUT        /* PD13  T4C2   FMU5 */ _MK_GPIO_INPUT(GPIO_TIM4_CH2IN)
#define GPIO_GPIO3_INPUT        /* PA8   T1C1   FMU4 */ _MK_GPIO_INPUT(GPIO_TIM1_CH1IN)
#define GPIO_GPIO2_INPUT        /* PE11  T1C2   FMU3 */ _MK_GPIO_INPUT(GPIO_TIM1_CH2IN)
#define GPIO_GPIO1_INPUT        /* PA10  T1C3   FMU2 */ _MK_GPIO_INPUT(GPIO_TIM1_CH3IN)
#define GPIO_GPIO0_INPUT        /* PE14  T1C4   FMU1 */ _MK_GPIO_INPUT(GPIO_TIM1_CH4IN)

#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))

#define GPIO_GPIO7_OUTPUT        /* PD15  T4C4   FMU8 */ _MK_GPIO_OUTPUT(GPIO_TIM4_CH4OUT)
#define GPIO_GPIO6_OUTPUT        /* PD12  T4C1   FMU7 */ _MK_GPIO_OUTPUT(GPIO_TIM4_CH1OUT)
#define GPIO_GPIO5_OUTPUT        /* PD14  T4C3   FMU6 */ _MK_GPIO_OUTPUT(GPIO_TIM4_CH3OUT)
#define GPIO_GPIO4_OUTPUT        /* PD13  T4C2   FMU5 */ _MK_GPIO_OUTPUT(GPIO_TIM4_CH2OUT)
#define GPIO_GPIO3_OUTPUT        /* PA8   T1C1   FMU4 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH1OUT)
#define GPIO_GPIO2_OUTPUT        /* PE11  T1C2   FMU3 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH2OUT)
#define GPIO_GPIO1_OUTPUT        /* PA10  T1C3   FMU2 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH3OUT)
#define GPIO_GPIO0_OUTPUT        /* PE14  T1C4   FMU1 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH4OUT)

/* Power supply control and monitoring GPIOs */

#define BOARD_NUMBER_BRICKS             0
#define BOARD_NUMBER_DIGITAL_BRICKS     0

#define GPIO_CAN1_SILENT                /* PI11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN11)

#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_VDD_3V3_SD_CARD_EN         /* PC13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

/* For primary/backup signaling with VOXL, 2 pins on J4 are exposed */

#define GPIO_VOXL_STATUS_OUT            /* PE4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_VOXL_STATUS_IN             /* PE3 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN3)

/* Define True logic Power Control in arch agnostic form */

#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* T8C1 */  1  /* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PI5 T8C1 */ GPIO_TIM8_CH1IN_2

#define RC_UXART_BASE                      STM32_USART6_BASE
#define RC_SERIAL_PORT                     "/dev/ttyS5"
#define BOARD_HAS_SINGLE_WIRE              1 /* HW is capable of Single Wire */
#define BOARD_HAS_SINGLE_WIRE_ON_TX        0 /* HW default is wired as Single Wire On RX pin */
#define BOARD_HAS_RX_TX_SWAP               1 /* HW Can swap TX and RX */
#define RC_SERIAL_PORT_IS_SWAPED           1 /* Board wired with RC's TX is on cpu RX */

/* Safety Switch: Enable the FMU to control it as there is no px4io in ModalAI FC-v1 */
#define GPIO_SAFETY_SWITCH_IN              /* PF3 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTF|GPIO_PIN3)
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* Power switch controls ******************************************************/

#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * ModalAI FC-v1 has a separate RC_IN
 *
 * GPIO PPM_IN on PI5 T8CH1
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PC7
 *   Inversion is possible in the UART and can drive  GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN5)

#define SPEKTRUM_RX_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(SPEKTRUM_RX_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(SPEKTRUM_RX_AS_OUT, (_one_true))

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

/* ModalAI FC-v1 never powers off the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#if !defined(BOARD_HAS_LTC44XX_VALIDS) || BOARD_HAS_LTC44XX_VALIDS == 0
#  define BOARD_ADC_BRICK1_VALID  (1)
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 1
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 2
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 3
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 4
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#  define BOARD_ADC_BRICK4_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK4_VALID))
#else
#  error Unsupported BOARD_HAS_LTC44XX_VALIDS value
#endif

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

/* The list of GPIO that will be initialized */

#define PX4_GPIO_PWM_INIT_LIST { \
		GPIO_GPIO7_INPUT, \
		GPIO_GPIO6_INPUT, \
		GPIO_GPIO5_INPUT, \
		GPIO_GPIO4_INPUT, \
		GPIO_GPIO3_INPUT, \
		GPIO_GPIO2_INPUT, \
		GPIO_GPIO1_INPUT, \
		GPIO_GPIO0_INPUT, \
	}

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_A71CH_nRST,                  \
		GPIO_VOXL_STATUS_OUT,             \
		GPIO_VOXL_STATUS_IN,              \
		GPIO_SAFETY_SWITCH_IN             \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 5

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);


/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#include <drivers/boards/common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
