/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * PX4FMUv3 internal definitions: identical to fmuv2 except for usable flash
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

/* Run time Hardware detection */
#define BOARD_HAS_SIMPLE_HW_VERSIONING 1
#define HW_VER_PB4             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN4)
#define HW_VER_PB12            (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN12)
#define HW_VER_PB4_INIT        (GPIO_SPI1_EXTI_DRDY_PB4)
#define HW_VER_PB12_INIT       (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN12)
#define HW_VER_FMUV2_STATE     0x8 /* PB12:PU:1 PB12:PD:0 PB4:PU:0 PB4PD:0 */
#define HW_VER_FMUV3_STATE     0xE /* PB12:PU:1 PB12:PD:1 PB4:PU:1 PB4PD:0 */
#define HW_VER_FMUV2MINI_STATE 0xA /* PB12:PU:1 PB12:PD:0 PB4:PU:1 PB4PD:0 */
#define HW_VER_TYPE_INIT {'V','2',0, 0}

/*----------------------------------------------------------*/
/*           FMUv3 Cube SPI chip selects and DRDY           */
/*----------------------------------------------------------*/
/* Due to inconsistent use of chip select and dry signal on
 * different board that use this build. We are defining the GPIO
 * inclusive of the SPI port and GPIO to help identify pins the
 * are part of the sensor Net's controlled by different power
 * domains.
 *
 *  --------------- SPI1 -------------------- SPI4 --------------     Incompatibilities    ---------
 *  FMUv3 Cube:                                                       FmuV2            PixhawkMini
 *   Power Domain:  VDD_3V3_SENSORS_EN         NA			      V3V:SPI V5:SPI4   V3V:SPI1 No SPI4
 *   PA5            SPI_INT_SCK
 *   PA6            SPI_INT_MISO
 *   PA7            SPI_INT_MOSI
 *   PB0                                      EXTERN_DRDY          SPI1:GYRO_DRDY          NC
 *   PB1            MAG_DRDY                  nEXTERN_CS           -SPI1:MAG_DRDY          NC
 *   PB4                                          NC               SPI1:ACCEL_DRDY         NC
 *   PC1            SPI_INT_MAG_!CS                                -ADC1_IN11              NC
 *   PC2            nMPU_CS                                        @MPU6000             @MPU9250
 *   PC13                                     nGYRO_EXT_CS         SPI1:nGYRO_CS           NC
 *   PC14                                     nBARO_EXT_CS         GPIO_EXT_1            -20608_DRDY
 *   PC15                                     nACCEL_MAG_EXT_CS    SPI1:nACCEL_MAG_CS     20608_CS
 *   PD7            nBARO_CS
 *   PD15           nMPU_DRDY                                      @MPU6000              @MPU9250
 *   PE2                                      SPI_EXT_SCK                                  NC
 *   PE4                                      MPU_EXT_CS           SPI4:nSPI_EXT_NSS       NC
 *   PE5                                      SPI_EXT_MISO                                 NC
 *   PE6                                      SPI_EXT_MOSI                                 NC
 *
 *
 *   Notes: Prefixed with @ Does not effect board control
 *          Prefixed with + Input used as Output
 *          Prefixed with - Output used as Input
 *          Prefixed with SPIn: Bus changed
 *
 */

/* FMUv3 Cube SPI1 chip selects */
/*      Was a spare ACD IN10 on V2 */
#define GPIO_SPI1_CS_PC1                 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN1)
#define GPIO_SPI4_CS_PB1                 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_SPI4_CS_PC13                GPIO_SPI1_CS_PC13
#define GPIO_SPI4_CS_PC14                GPIO_SPI4_GPIO_PC14
#define GPIO_SPI4_CS_PC15                GPIO_SPI1_CS_PC15

/* FMUv3 Cube chip selects Assignments */
/*                                                        Cube 2.0   V2.1    */
#define GPIO_SPI1_CS_MPU                 GPIO_SPI1_CS_PC2  /* MPU600    MPU9250 */
#define GPIO_SPI1_CS_BARO                GPIO_SPI1_CS_PD7  /* MS5611    MS5611  */
#define GPIO_SPI1_CS_HMC                 GPIO_SPI1_CS_PC1  /* HMC5983   Removed */

/* N.B. bus moves from SPI1 to SPI4 */
#define GPIO_SPI4_GYRO_EXT_CS            GPIO_SPI4_CS_PC13
#define GPIO_SPI4_BARO_EXT_CS            GPIO_SPI4_CS_PC14
#define GPIO_SPI4_ACCEL_MAG_EXT_CS       GPIO_SPI4_CS_PC15

/* No move */
#define GPIO_SPI4_MPU_EXT_CS             GPIO_SPI4_NSS_PE4

/* FMUv3 DRDY Assignments */
#define GPIO_SPI4_EXTI_DRDY_PB0          GPIO_SPI1_EXTI_DRDY_PB0
#define GPIO_SPI4_EXTI_EXTERN_DRDY       GPIO_SPI4_EXTI_DRDY_PB0
#define GPIO_SPI4_EXTERN_CS              GPIO_SPI4_CS_PB1
/* PB1 is an External CS on V3 */

#define PX4_SPIDEV_HMC            5

/*----------------------------------------------------------*/
/*       End FMUv3 Cube SPI chip selects and DRDY           */
/*----------------------------------------------------------*/
/*----------------------------------------------------------*/
/* Due to inconsistent use of chip select and dry signal on
 * different board that use this build. We are defining the GPIO
 * inclusive of the SPI port and GPIO to help identify pins the
 * are part of the sensor Net's controlled by different power
 * domains.
 *
 *  --------------- SPI1 -------------------- SPI4 --------------      Incompatibilities    ---------
 *  FMUv2 Pixhawk Mini                                                FmuV2               FmuV3 Cube
 *   Power Domain:  VDD_3V3_SENSORS_EN        NA                  V3V:SPI V5:SPI4        V3V:SPI1&SPI4
 *   PA5            SPI_INT_SCK
 *   PA6            SPI_INT_MISO
 *   PA7            SPI_INT_MOSI
 *   PB0            NC                                             SPI1:GYRO_DRDY      SPI4:EXTERN_DRDY
 *   PB1            NC                                             -SPI1:MAG_DRDY      +SPI4:nEXTERN_CS
 *   PB4            NC                                             SPI1:ACCEL_DRDY     NC
 *   PC1            Spare ADC ( NC )                                                   +SPI1:SPI_INT_MAG_!CS
 *   PC2            nMPU_CS                                        @MPU6000            @MPU6000|MPU9250
 *   PC13           NC                                             SPI1:nGYRO_CS       SPI4:nGYRO_EXT_CS
 *   PC14           20608_DRDY                                     +GPIO_EXT_1         nBARO_EXT_CS
 *   PC15           20608_CS                                       nACCEL_MAG_CS       SPI4:nACCEL_MAG_EXT_CS
 *   PD7            nBARO_CS
 *   PD15           nMPU_DRDY                                      @MPU6000            @MPU6000|MPU9250
 *   PE2                                      NC                   SPI_EXT_SCK         SPI_EXT_SCK
 *   PE4                                      NC                   SPI4:nSPI_EXT_NSS   SPI4:nMPU_EXT_CS
 *   PE5                                      NC                   SPI_EXT_MISO        SPI_EXT_MISO
 *   PE6                                      NC                   SPI_EXT_MOSI        SPI_EXT_MOSI
 *
 *   Notes: Prefixed with @ Does not effect board control
 *          Prefixed with + Input used as Output
 *          Prefixed with - Output used as Input
 *          Prefixed with SPIn: Bus changed
 *
 */

/*----------------------------------------------------------*/
/*           FMUv2 PixhawkMini SPI chip selects and DRDY    */
/*----------------------------------------------------------*/

/* FMUv2 PixhawkMini SPI1 chip selects */

/* FMUv3 Cube chip selects Assignments */

#define GPIO_SPI1_CS_MPU                 GPIO_SPI1_CS_PC2  /* MPU9250  */
#define GPIO_SPI1_CS_BARO                GPIO_SPI1_CS_PD7  /* MS5611   */
#define GPIO_SPI1_CS_20608               GPIO_SPI1_CS_PC15 /* ICM20608 */

/* FMUv3 DRDY Assignments */

/* Pixhawk mini has reused the PC14 GPIO_SPI_CS_EXT1 signal that was associated
 * with SPI4.
 */
#define GPIO_SPI1_EXTI_20608_DRDY_PC14   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN14)

#define PX4_SPIDEV_ICM_20608         6   /* ICM_20608 on PC15 */

/*----------------------------------------------------------*/
/*           FMUv2 PixhawkMini SPI chip selects and DRDY    */
/*----------------------------------------------------------*/
#include "../px4fmu-v2/board_config.h"
