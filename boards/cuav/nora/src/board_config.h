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
 * cuav nora internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* PX4IO connection configuration */

#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS5"
#define PX4IO_SERIAL_TX_GPIO           GPIO_UART8_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_UART8_RX
#define PX4IO_SERIAL_BASE              STM32_UART8_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_UART8
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_UART8_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_UART8_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB1LENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB1LENR_UART8EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK1_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */
#define STM32_RCC_APB1ENR  STM32_RCC_APB1LENR
#define RCC_APB1ENR_TIM3EN RCC_APB1LENR_TIM3EN


/* Configuration ************************************************************************************/

#  define BOARD_HAS_LTC44XX_VALIDS      2 // No LTC or N Bricks
#  define BOARD_HAS_USB_VALID           1 // LTC Has No USB valid
#  define BOARD_HAS_NBAT_V              2 // Only one Vbat to ADC
#  define BOARD_HAS_NBAT_I              2 // No Ibat ADC

/* Cuav Nora GPIOs ************************************************************************/

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_RED        /* PD10 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define GPIO_nLED_GREEN      /* PC6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define GPIO_nLED_BLUE       /* PC7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/* SENSORS are on SPI1, 5, 6
 * MEMORY is on bus SPI2
 * MS5611 is on bus SPI4
 */

#define PX4_SPI_BUS_SENSORS   1
#define PX4_SPI_BUS_MEMORY    2
#define PX4_SPI_BUS_BARO      4
#define PX4_SPI_BUS_EXTERNAL1 5
#define PX4_SPI_BUS_EXTERNAL2 6
#define PX4_SPI_BUS_SENSORS3  PX4_SPI_BUS_BARO // for BMI088
#define PX4_SPI_BUS_EXT       PX4_SPI_BUS_EXTERNAL2
/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

/* SPI 1 CS */

#define GPIO_SPI1_CS1_RM3100      /* PF2 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN2)
#define GPIO_SPI1_CS2_ICM20689    /* PG6 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN6)

/*  Define the SPI1 Data Ready interrupts */

#define GPIO_SPI1_DRDY1_RM3100    /* PE4 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN4)
#define GPIO_SPI1_DRDY2_ICM20689 /* PJ0 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTJ|GPIO_PIN0)

/* SPI1 off */

#define GPIO_SPI1_SCK_OFF  _PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF  _PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(GPIO_SPI1_MOSI)

#define GPIO_DRDY_OFF_SPI1_DRDY1_RM3100    _PIN_OFF(GPIO_SPI1_DRDY1_RM3100)
#define GPIO_DRDY_OFF_SPI1_DRDY2_ICM20689 _PIN_OFF(GPIO_SPI1_DRDY2_ICM20689)

/* SPI 2 CS */

#define GPIO_SPI2_CS_FRAM         /* PF5 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN5)

/* SPI 4 CS */

#define GPIO_SPI4_CS1_MS5611      /* PG10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
#define GPIO_SPI4_CS2_BMI088_GYRO /* PF4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN4)
#define GPIO_SPI4_CS3_BMI088_ACC  /* PF3 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN3)

#define GPIO_SPI4_DRDY1_BMI088_GYRO    /* PB14 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SPI4_DRDY2_BMI088_ACC     /* PB15 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN15)
#define GPIO_SPI4_DRDY3_BMI088_GYRO    /* PC13 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_SPI4_DRDY4_BMI088_ACC     /* PI14 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN14)


#define GPIO_DRDY_OFF_SPI4_DRDY1_BMI088_GYRO    _PIN_OFF(GPIO_SPI4_DRDY1_BMI088_GYRO)
#define GPIO_DRDY_OFF_SPI4_DRDY2_MI088_ACC      _PIN_OFF(GPIO_SPI4_DRDY2_BMI088_ACC)
#define GPIO_DRDY_OFF_SPI4_DRDY3_BMI088_GYRO    _PIN_OFF(GPIO_SPI4_DRDY3_BMI088_GYRO)
#define GPIO_DRDY_OFF_SPI4_DRDY4_MI088_ACC      _PIN_OFF(GPIO_SPI4_DRDY4_BMI088_ACC)

/* SPI 5 CS */

#define SPI5_CS1_EXTERNAL1     /* PI4  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN4)
#define SPI5_CS2_EXTERNAL1     /* PI10 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN10)

/*  Define the SPI1 Data Ready and Control signals */

//#define GPIO_SPI5_DRDY7_EXTERNAL1   /* PD15 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)

/* SPI 6 */

#define SPI6_CS1_MS5611_BOARD    /* PI8  */   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN8)
#define SPI6_CS2_ICM20649        /* PI12  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN12)

#define GPIO_SPI6_DRDY1_ICM20649    /* PH5 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTH|GPIO_PIN5)

#define GPIO_DRDY_OFF_SPI6_DRDY1_ICM20649    _PIN_OFF(GPIO_SPI6_DRDY1_ICM20649)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))


//#define GPIO_DRDY_OFF_SPI5_DRDY7_EXTERNAL1   _PIN_OFF(GPIO_SPI5_DRDY7_EXTERNAL1)


/* v BEGIN Legacy SPI defines TODO: fix this with enumeration */
#define PX4_SPI_BUS_RAMTRON  PX4_SPI_BUS_MEMORY
/* ^ END Legacy SPI defines TODO: fix this with enumeration */

#define PX4_SPIDEV_RM               PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
#define PX4_SPIDEV_ICM_20689        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,1)
#define PX4_SENSOR_BUS_CS_GPIO      {GPIO_SPI1_CS1_RM3100, GPIO_SPI1_CS2_ICM20689}

#define PX4_SPIDEV_MEMORY           PX4_MK_SPI_SEL(PX4_SPI_BUS_MEMORY,0)
#define PX4_MEMORY_BUS_CS_GPIO      {GPIO_SPI2_CS_FRAM}

#define PX4_SPIDEV_BARO             PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,0)
#define PX4_SPIDEV_BMI088_GYR       PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,1)
#define PX4_SPIDEV_BMI088_ACC       PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,2)
#define PX4_BARO_BUS_CS_GPIO        {GPIO_SPI4_CS1_MS5611, GPIO_SPI4_CS2_BMI088_GYRO, GPIO_SPI4_CS3_BMI088_ACC}


#define PX4_SPIDEV_EXTERNAL1_1      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL1,0)
#define PX4_SPIDEV_EXTERNAL1_2      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL1,1)
#define PX4_EXTERNAL1_BUS_CS_GPIO   {SPI5_CS1_EXTERNAL1, SPI5_CS2_EXTERNAL1}

#define PX4_SPIDEV_EXT_BARO         PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL2,0)
#define PX4_SPIDEV_ICM_20649        PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL2,1)
#define PX4_SPIDEV_EXT_MPU         1//PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL2,2)
#define PX4_EXTERNAL2_BUS_CS_GPIO   {SPI6_CS1_MS5611_BOARD, SPI6_CS2_ICM20649}

/* I2C busses */

#define PX4_I2C_BUS_EXPANSION       1
#define PX4_I2C_BUS_EXPANSION1      2
#define PX4_I2C_BUS_ONBOARD         3
#define PX4_I2C_BUS_EXPANSION2      4
#define PX4_I2C_BUS_LED             PX4_I2C_BUS_EXPANSION

#define BOARD_NUMBER_I2C_BUSES      4
#define BOARD_I2C_BUS_CLOCK_INIT    {100000, 100000, 100000, 100000}

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */


#define PX4_ADC_GPIO  \
	/* PA0 */  GPIO_ADC1_INP16,   \
	/* PA1 */  GPIO_ADC1_INP17,   \
	/* PA2 */  GPIO_ADC12_INP14,  \
	/* PF11 */ GPIO_ADC1_INP2,  \
	/* PA4 */  GPIO_ADC12_INP18,  \
	/* PF12 */ GPIO_ADC1_INP6,   \
	/* PC5 */  GPIO_ADC12_INP8, \
	/* PC1 */  GPIO_ADC123_INP11, \
	/* PC2 */  GPIO_ADC123_INP12, \
	/* PC3 */  GPIO_ADC12_INP13,  \
	/* PC4 */  GPIO_ADC12_INP4

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY1_VOLTAGE_CHANNEL        /* PA0 */  ADC1_CH(16)
#define ADC_BATTERY1_CURRENT_CHANNEL        /* PA1 */  ADC1_CH(17)
#define ADC_BATTERY2_VOLTAGE_CHANNEL        /* PA2 */  ADC1_CH(14)
#define ADC_BATTERY2_CURRENT_CHANNEL        /* PF11 */ ADC1_CH(2)
#define ADC1_6V6_IN_CHANNEL                 /* PA4 */  ADC1_CH(18)
#define ADC_RSSI_IN_CHANNEL                 /* PF12 */ ADC1_CH(6)
#define ADC_SCALED_V5_CHANNEL               /* PC5 */  ADC1_CH(8)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  /* PC1 */  ADC1_CH(11)
#define ADC_HW_VER_SENSE_CHANNEL            /* PC2 */  ADC1_CH(12)
#define ADC_HW_REV_SENSE_CHANNEL            /* PC3 */  ADC1_CH(13)
#define ADC1_3V3_IN_CHANNEL                 /* PC4 */  ADC1_CH(4)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC1_6V6_IN_CHANNEL)                | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_3V3_IN_CHANNEL))

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV         (18.0f)     /* measured with the provided PM board */
#define BOARD_BATTERY1_A_PER_V       (24.0f)

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define GPIO_HW_REV_DRIVE    /* PJ3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTJ|GPIO_PIN3)
#define GPIO_HW_REV_SENSE    /* PC3   */ GPIO_ADC12_INP13
#define GPIO_HW_VER_DRIVE    /* PJ4   */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTJ|GPIO_PIN4)
#define GPIO_HW_VER_SENSE    /* PC2   */ GPIO_ADC123_INP12
#define HW_INFO_INIT         {'V','D','x', 'x',0}
#define HW_INFO_INIT_VER     0
#define HW_INFO_INIT_REV     0

/* CAN Silence
 *
 * Silent mode control \ ESC Mux select
 */

#define GPIO_CAN1_SILENT_S0  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_CAN2_SILENT_S1  /* PH3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN3)

/* HEATER
 * PWM in future
 */
#define GPIO_HEATER_OUTPUT   /* PA8  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

/* PWM Capture
 *
 * 6  PWM Capture inputs are configured.
 *
 * Pins:
 *
 * FMU_CAP1 : PB11 : TIM2_CH4
 * FMU_CAP2 : PB3  : TIM2_CH2
 * FMU_CAP3 : PA5  : TIM2_CH1
 * FMU_CAP4 : PH9  : TIM12_CH2
 * FMU_CAP5 : PH6  : TIM12_CH1
 * FMU_CAP6 : PD14 : TIM4_CH3
 *
 * FMU_CAP1 : PE9  : TIM1_CH1
 * FMU_CAP2 : PE11 : TIM1_CH2
 * FMU_CAP3 : PA10 : TIM1_CH3
 * FMU_CAP4 : PE14 : TIM1_CH4
 * FMU_CAP5 : PH6  : TIM12_CH1
 * FMU_CAP6 : PH9  : TIM12_CH2
 */

// #define GPIO_TIM1_CH1_IN     /* PE9    FMU_CAP1 */ GPIO_TIM1_CH1IN_2
// #define GPIO_TIM1_CH2_IN     /* PE11   FMU_CAP2 */ GPIO_TIM1_CH2IN_2
// #define GPIO_TIM1_CH3_IN     /* PA10   FMU_CAP3 */ GPIO_TIM1_CH3IN_1
// #define GPIO_TIM1_CH4_IN     /* PE14   FMU_CAP4 */ GPIO_TIM1_CH4IN_2
// #define GPIO_TIM12_CH1_IN    /* PH6    FMU_CAP5 */ GPIO_TIM12_CH1IN_2
// #define GPIO_TIM12_CH2_IN    /* PH9    FMU_CAP6 */ GPIO_TIM12_CH2IN_2

//#define DIRECT_PWM_CAPTURE_CHANNELS  0

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define DIRECT_INPUT_TIMER_CHANNELS  8
#define BOARD_NUM_IO_TIMERS 4

#define BOARD_HAS_LED_PWM              1
#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW 1

#define BOARD_HAS_UI_LED_PWM            1
#define BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW 1

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0, 4, 5, 6, 7};

/* Power supply control and monitoring GPIOs */

#define GPIO_nPOWER_IN_ADC              /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1)
#define GPIO_nPOWER_IN_CAN              /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
#define GPIO_nPOWER_IN_C                /* PG0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN0)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_CAN /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_ADC /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

#define GPIO_nVDD_5V_PERIPH_EN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_nVDD_5V_PERIPH_OC          /* PE15 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN15)
#define GPIO_nVDD_5V_HIPOWER_EN         /* PD11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#define GPIO_nVDD_5V_HIPOWER_OC         /* PG13 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTF|GPIO_PIN13)
#define GPIO_VDD_3V3_SENSORS_EN         /* PE3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)
//#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PE4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_VDD_3V3_SD_CARD_EN         /* PG7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN7)


/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, (on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_nVDD_5V_HIPOWER_EN, (on_true))
#define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))
//#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
//#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))


/* Tone alarm output */

#define TONE_ALARM_TIMER        15  /* timer 15 */
#define TONE_ALARM_CHANNEL      1  /* PE5 TIM15_CH1 */

#define GPIO_BUZZER_1           /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM15_CH1OUT_2

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               3  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

/* RC Serial port */
#define HRT_PPM_CHANNEL         /* T3C1 */  1  /* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PB4 T3C1 */ GPIO_TIM3_CH1IN_2

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS5"
#define RC_SERIAL_SINGLEWIRE

/* Input Capture Channels. */

// #define INPUT_CAP1_TIMER                  1
// #define INPUT_CAP1_CHANNEL     /* T1C1 */ 1
// #define GPIO_INPUT_CAP1        /* PE9 */ GPIO_TIM1_CH1_IN

// #define INPUT_CAP2_TIMER                  1
// #define INPUT_CAP2_CHANNEL     /* T1C2 */ 2
// #define GPIO_INPUT_CAP2        /* PE11 */ GPIO_TIM1_CH2_IN

// #define INPUT_CAP3_TIMER                  1
// #define INPUT_CAP3_CHANNEL     /* T1C3 */ 3
// #define GPIO_INPUT_CAP3        /* PA10 */ GPIO_TIM1_CH3_IN

// #define INPUT_CAP4_TIMER                  1
// #define INPUT_CAP4_CHANNEL     /* T1C4 */ 4
// #define GPIO_INPUT_CAP4        /* PE14 */ GPIO_TIM1_CH4_IN

// #define INPUT_CAP5_TIMER                  12
// #define INPUT_CAP5_CHANNEL     /* T12C1 */ 1
// #define GPIO_INPUT_CAP5        /* PA10 */ GPIO_TIM12_CH1_IN

// #define INPUT_CAP6_TIMER                  12
// #define INPUT_CAP6_CHANNEL     /* T12C2 */ 2
// #define GPIO_INPUT_CAP6       /* PE14 */ GPIO_TIM12_CH2_IN

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */

#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C1 */ 1
#define GPIO_PWM_IN            /* PD12 */ GPIO_TIM4_CH1IN

/* RSSI_IN is grounded via a 10K */

#define GPIO_RSSI_IN                       /* PF12  */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTF|GPIO_PIN12)
#define GPIO_RSSI_IN_INIT                  /* PF12  */ 0 /* Leave as ADC RSSI_IN */

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* PE12 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN12)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* PE12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT
#define GPIO_SAFETY_SWITCH_IN              /* PE10 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN10)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* Power switch controls ******************************************************/

//#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * FMUv5 has a separate RC_IN
 *
 * GPIO PPM_IN on PI5 T8CH1
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PG9 (NOT FMUv5 test HW ONLY)
 *   In version is possible in the UART
 * and can drive  GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

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
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))

/* Board never powers off the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_REV_DRIVE,                \
		GPIO_HW_VER_DRIVE,                \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN2_SILENT_S1,              \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_CAN,                 \
		GPIO_nPOWER_IN_ADC,                 \
		GPIO_nPOWER_IN_C,                 \
		GPIO_nVDD_5V_PERIPH_EN,           \
		GPIO_nVDD_5V_PERIPH_OC,           \
		GPIO_nVDD_5V_HIPOWER_EN,          \
		GPIO_nVDD_5V_HIPOWER_OC,          \
		GPIO_VDD_3V3_SENSORS_EN,          \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_RSSI_IN,                     \
		GPIO_nSAFETY_SWITCH_LED_OUT,      \
		GPIO_SAFETY_SWITCH_IN,            \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

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
 *   Called to configure SPI chip select GPIO pins for the Cuav Nora
 *   board.
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

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
