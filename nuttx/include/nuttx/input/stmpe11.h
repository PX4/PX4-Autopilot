/********************************************************************************************
 * include/nuttx/input/stmpe11.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "STMPE811 S-Touch® advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_STMPE11_H
#define __INCLUDE_NUTTX_INPUT_STMPE11_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/i2c.h>
#include <nuttx/spi.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE11)

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/
/* The STMPE811 interfaces with the host CPU via a I2C or SPI interface. The pin IN_1 allows
 * the selection of interface protocol at reset state.
 */

#if !defined(CONFIG_STMPE11_SPI) && !defined(CONFIG_STMPE11_I2C)
#  error "One of CONFIG_STMPE11_SPI or CONFIG_STMPE11_I2C must be defined"
#endif

#if defined(CONFIG_STMPE11_SPI) && defined(CONFIG_STMPE11_I2C)
#  error "Only one of CONFIG_STMPE11_SPI or CONFIG_STMPE11_I2C can be defined"
#endif

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_STMPE11_NPOLLWAITERS
#  define CONFIG_STMPE11_NPOLLWAITERS 2
#endif

/* Check for some required settings.  This can save the user a lot of time
 * in getting the right configuration.
 */

#ifndef CONFIG_I2C_TRANSFER
#  error "CONFIG_I2C_TRANSFER is required in the I2C configuration"
#endif

#ifdef CONFIG_DISABLE_SIGNALS
#  error "Signals are required.  CONFIG_DISABLE_SIGNALS must not be selected."
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected."
#endif

/* I2C **************************************************************************************/
/* STMPE11 Address:  The STMPE11 may have 7-bit address 0x41 or 0x44, depending upon the
 * state of the ADDR0 pin.
 */

#define STMPE11_I2C_ADDRESS_MASK     (0x78)       /* Bits 3-7: Invariant part of STMPE11 address */
#define STMPE11_I2C_ADDRESS          (0x40)       /* Bits 3-7: Always set at '0100 0xxR' */
#define STMPE11_I2C_A1               (1 << 2)     /* Bit 2: A1 */
#define STMPE11_I2C_A0               (1 << 1)     /* Bit 1: A0 */
#define STMPE11_I2C_READ             (1 << 0)     /* Bit 0=1: Selects read operation */
#define STMPE11_I2C_WRITE            (0)          /* Bit 0=0: Selects write operation */

/* I2C frequency */

#define STMPE11_I2C_MAXFREQUENCY     400000       /* 400KHz */

/* SPI **************************************************************************************/
/* The device always operates in mode 0 */

#define STMPE11_SPI_MODE             SPIDEV_MODE0 /* Mode 0 */

/* I2C frequency */

#define STMPE11_SPI_MAXFREQUENCY     1000000      /* 1MHz */

/* STMPE11 Registers ************************************************************************/
/* Register Addresses */

#define STMPE11_CHIP_ID              0x00  /* Device identification (16-bit) */
#define STMPE11_ID_VER               0x02  /* Revision number: 0x01=sample 0x03=final silicon */
#define STMPE11_SYS_CTRL1            0x03  /* Reset control */
#define STMPE11_SYS_CTRL2            0x04  /* Clock control */
#define STMPE11_SPI_CFG              0x08  /* SPI interface configuration */
#define STMPE11_INT_CTRL             0x09  /* Interrupt control register */
#define STMPE11_INT_EN               0x0a  /* Interrupt enable register */
#define STMPE11_INT_STA              0x0b  /* Interrupt status register */
#define STMPE11_GPIO_EN              0x0c  /* GPIO interrupt enable register */
#define STMPE11_GPIO_INTSTA          0x0d  /* GPIO interrupt status register */
#define STMPE11_ADC_INTEN            0x0e  /* ADC interrupt enable register */
#define STMPE11_ADC_INTSTA           0x0f  /* ADC interrupt status register */
#define STMPE11_GPIO_SETPIN          0x10  /* GPIO set pin register */
#define STMPE11_GPIO_CLRPIN          0x11  /* GPIO clear pin register */
#define STMPE11_GPIO_MPSTA           0x12  /* GPIO monitor pin state register */
#define STMPE11_GPIO_DIR             0x13  /* GPIO direction register */
#define STMPE11_GPIO_ED              0x14  /* GPIO edge detect register */
#define STMPE11_GPIO_RE              0x15  /* GPIO rising edge register */
#define STMPE11_GPIO_FE              0x16  /* GPIO falling edge register */
#define STMPE11_GPIO_AF              0x17  /* Alternate function register */
#define STMPE11_ADC_CTRL1            0x20  /* ADC control */
#define STMPE11_ADC_CTRL2            0x21  /* ADC control */
#define STMPE11_ADC_CAPT             0x22  /* To initiate ADC data acquisition */
#define STMPE11_ADC_DATACH0          0x30  /* ADC channel 0 (16-bit) */
#define STMPE11_ADC_DATACH1          0x32  /* ADC channel 1 (16_bit) */
#define STMPE11_ADC_DATACH2          0x34  /* ADC channel 2 (16-bit) */
#define STMPE11_ADC_DATACH3          0x36  /* ADC channel 3 (16-bit) */
#define STMPE11_ADC_DATACH4          0x38  /* ADC channel 4 (16-bit) */
#define STMPE11_ADC_DATACH5          0x3a  /* ADC channel 5 (16-bit) */
#define STMPE11_ADC_DATACH6          0x3c  /* ADC channel 6 (16-bit) */
#define STMPE11_ADC_DATACH7          0x3e  /* ADC channel 7 (16-bit) */
#define STMPE11_TSC_CTRL             0x40  /* 4-wire touchscreen controller setup */
#define STMPE11_TSWC_CFG             0x41  /* Touchscreen controller configuration */
#define STMPE11_WDW_TRX              0x42  /* Window setup for top right X (16-bit) */
#define STMPE11_WDW_TRY              0x44  /* Window setup for top right Y (16-bit) */
#define STMPE11_WDW_BLX              0x46  /* Window setup for bottom left X (16-bit) */
#define STMPE11_WDW_BLY              0x48  /* Window setup for bottom left Y (16-bit) */
#define STMPE11_FIFO_TH              0x4a  /* FIFO level to generate interrupt */
#define STMPE11_FIFO_STA             0x4b  /* Current status of FIFO */
#define STMPE11_FIFO_SIZE            0x4c  /* Current filled level of FIFO */
#define STMPE11_TSC_DATAX            0x4d  /* Data port for touchscreen (16-bit) */
#define STMPE11_TSC_DATAY            0x4f  /* Data port for touchscreen (16-bit) */
#define STMPE11_TSC_DATAZ            0x51  /* Data port for touchscreen */
#define STMPE11_TSC_DATAXYZ          0x52  /* Data port for touchscreen (32-bit) */
#define STMPE11_TSC_FRACTIONZ        0x56  /* Touchscreen controller FRACTION_Z */
#define STMPE11_TSC_DATA             0x57  /* Data port for touchscreen */
#define STMPE11_TSC_IDRIVE           0x58  /* Touchscreen controller drive I */
#define STMPE11_TSC_SHIELD           0x59  /* Touchscreen controller shield */
#define STMPE11_TEMP_CTRL            0x60  /* Temperature sensor setup */
#define STMPE11_TEMP_DATA            0x61  /* Temperature data access port */
#define STMPE11_TEMP_TH              0x62  /* Threshold for temperature controlled interrupt */

/* Register bit definitions */

/* Device identification (16-bit) */

#define CHIP_ID                      0x0811

/* Revision number: 0x01=sample 0x03=final silicon */

#define ID_VER_SAMPLE                0x01
#define ID_VER_FINAL                 0x03

/* Reset control */

#define SYS_CTRL1_HIBERNATE          (1 << 0)  /* Bit 0: Force the device into hibernation mode */
#define SYS_CTRL1_SOFTRESET          (1 << 1)  /* Bit 1: Reset the STMPE811 */

/* Clock control */

#define SYS_CTRL2_ADC_OFF            (1 << 0)  /* Bit 0: Switch off clock to ADC */
#define SYS_CTRL2_TSC_OFF            (1 << 1)  /* Bit 1: Switch off clock to touchscreen controller */
#define SYS_CTRL2_GPIO_OFF           (1 << 2)  /* Bit 2: Switch off clock to GPIO */
#define SYS_CTRL2_TS_OFF             (1 << 3)  /* Bit 3: Switch off clock to temperature sensor */

/* SPI interface configuration */

#define SPI_CFG_SPI_CLK_MOD0         (1 << 0)  /* Bit 0: Value of SCAD/A0 pin at power-up reset */
#define SPI_CFG_SPI_CLK_MOD1         (1 << 1)  /* Bit 1: Value of SCAD/A0 pin at power-up reset */
#define SPI_CFG_AUTO_INCR            (1 << 2)  /* Bit 2: Enable internally autoincremented addressing */

/* Interrupt control register */

#define INT_CTRL_GLOBAL_INT          (1 << 0)  /* Bit 0: Master interrupt enable */
#define INT_CTRL_INT_TYPE            (1 << 1)  /* Bit 1: Type of interrupt signal.  1=edge */
#define INT_CTRL_INT_POLARITY        (1 << 2)  /* Bit 2: Interrupt pin polarity. 1=active high */

/* Interrupt enable/status register */

#define INT_TOUCH_DET                (1 << 0)  /* Bit 0: Touch is detected */
#define INT_FIFO_TH                  (1 << 1)  /* Bit 1: FIFO is equal or above threshold value */
#define INT_FIFO_OFLOW               (1 << 2)  /* Bit 2: FIFO is overflowed */
#define INT_FIFO_FULL                (1 << 3)  /* Bit 3: FIFO is full */
#define INT_FIFO_EMPTY               (1 << 4)  /* Bit 4: FIFO is empty */
#define INT_TEMP_SENS                (1 << 5)  /* Bit 5: Temperature threshold triggering */
#define INT_ADC                      (1 << 6)  /* Bit 6: Any enabled ADC interrupts */
#define INT_GPIO                     (1 << 7)  /* Bit 7: Any enabled GPIO interrupts */


/* GPIO interrupt enable/status register */

#define GPIO_INT(n)                  (1 << (n))

/* ADC interrupt enable/status register */

#define ADC_INT(n)                   (1 << (n))

/* GPIO set/clear/sta/dir/edge/rising/falling/af registers */

#define GPIO_PIN(n)                  (1 << (n))

/* ADC control */

#define ADC_CTRL1_REF_SEL            (1 << 1)  /* Bit 1: Selects internal/external, 1=external */
#define ADC_CTRL1_MOD_12B            (1 << 3)  /* Bit 3: Selects 10/12-bit ADC operation, 1=12-bit */
#define ADC_CTRL1_SAMPLE_TIME_SHIFT  (4)       /* Bits 4-6: ADC conversion time in number of clock */
#define ADC_CTRL1_SAMPLE_TIME_MASK   (7 << ADC_CTRL1_SAMPLE_TIME_SHIFT)
#  define ADC_CTRL1_SAMPLE_TIME_36   (0 << ADC_CTRL1_SAMPLE_TIME_SHIFT)
#  define ADC_CTRL1_SAMPLE_TIME_44   (1 << ADC_CTRL1_SAMPLE_TIME_SHIFT)
#  define ADC_CTRL1_SAMPLE_TIME_56   (2 << ADC_CTRL1_SAMPLE_TIME_SHIFT)
#  define ADC_CTRL1_SAMPLE_TIME_64   (3 << ADC_CTRL1_SAMPLE_TIME_SHIFT)
#  define ADC_CTRL1_SAMPLE_TIME_80   (4 << ADC_CTRL1_SAMPLE_TIME_SHIFT)
#  define ADC_CTRL1_SAMPLE_TIME_96   (5 << ADC_CTRL1_SAMPLE_TIME_SHIFT)
#  define ADC_CTRL1_SAMPLE_TIME_124  (6 << ADC_CTRL1_SAMPLE_TIME_SHIFT)

/* ADC control */

#define ADC_CTRL2_ADC_FREQ_SHIFT     (0)       /* Bits 0-1: Selects the clock speed of ADC */
#define ADC_CTRL2_ADC_FREQ_MASK      (3 << ADC_CTRL2_ADC_FREQ_SHIFT)
#  define ADC_CTRL2_ADC_FREQ_1p625   (0 << ADC_CTRL2_ADC_FREQ_SHIFT) /* 00: 1.625 MHz typ. */
#  define ADC_CTRL2_ADC_FREQ_3p25    (1 << ADC_CTRL2_ADC_FREQ_SHIFT) /* 01: 3.25 MHz typ. */
#  define ADC_CTRL2_ADC_FREQ_6.5     (2 << ADC_CTRL2_ADC_FREQ_SHIFT) /* 1x: 6.5 MHz typ. */

/* To initiate ADC data acquisition */

#define ADC_CAPT_CH(n)               (1 << (n))

/* 4-wire touchscreen controller setup */

#define TSC_CTRL_EN                  (1 << 0)  /* Bit 0: Enable TSC */
#define TSC_CTRL_OP_MOD_SHIFT        (1)       /* Bits 1-3: TSC operating mode */
#define TSC_CTRL_OP_MOD_MASK         (7 << TSC_CTRL_OP_MOD_SHIFT)
#  define TSC_CTRL_OP_MOD_XYZ        (0 << TSC_CTRL_OP_MOD_SHIFT) /* 000: X, Y, Z acquisition */
#  define TSC_CTRL_OP_MOD_XY         (1 << TSC_CTRL_OP_MOD_SHIFT) /* 001: X, Y only */
#  define TSC_CTRL_OP_MOD_X          (2 << TSC_CTRL_OP_MOD_SHIFT) /* 010: X only */
#  define TSC_CTRL_OP_MOD_Y          (3 << TSC_CTRL_OP_MOD_SHIFT) /* 011: Y only */
#  define TSC_CTRL_OP_MOD_Z          (4 << TSC_CTRL_OP_MOD_SHIFT) /* 100: Z only */
#define TSC_CTRL_TRACK_SHIFT         (4)       /* Bits 4-6: Tracking index */
#define TSC_CTRL_TRACK_MASK          (7 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TRACK_NONE          (0 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TRACK_4             (1 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TRACK_8             (2 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TRACK_16            (3 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TRACK_32            (4 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TRACK_64            (5 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TRACK_92            (6 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TRACK_127           (7 << TSC_CTRL_TRACK_SHIFT)
#define TSC_CTRL_TSC_STA             (1 << 7)  /* Bit 7: TSC status. 1=touch detected */

/* Touchscreen controller configuration */

#define TSWC_CFG_SETTLING_SHIFT      (0)       /* Bits 0-2: Panel driver settling time */
#define TSWC_CFG_SETTLING_MASK       (7 << TSWC_CFG_SETTLING_SHIFT)
#  define TSWC_CFG_SETTLING_10US     (0 << TSWC_CFG_SETTLING_SHIFT)
#  define TSWC_CFG_SETTLING_100US    (1 << TSWC_CFG_SETTLING_SHIFT)
#  define TSWC_CFG_SETTLING_500US    (2 << TSWC_CFG_SETTLING_SHIFT)
#  define TSWC_CFG_SETTLING_1MS      (3 << TSWC_CFG_SETTLING_SHIFT)
#  define TSWC_CFG_SETTLING_5MS      (4 << TSWC_CFG_SETTLING_SHIFT)
#  define TSWC_CFG_SETTLING_10MS     (5 << TSWC_CFG_SETTLING_SHIFT)
#  define TSWC_CFG_SETTLING_50MS     (6 << TSWC_CFG_SETTLING_SHIFT)
#  define TSWC_CFG_SETTLING_100MS    (7 << TSWC_CFG_SETTLING_SHIFT)
#define TSWC_CFG_TOUCH_DELAY_SHIFT   (1)       /* Bits 3-5: Touch detect delay */
#define TSWC_CFG_TOUCH_DELAY_MASK    (7 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#  define TSWC_CFG_TOUCH_DELAY_10US  (0 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#  define TSWC_CFG_TOUCH_DELAY_50US  (1 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#  define TSWC_CFG_TOUCH_DELAY_100US (1 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#  define TSWC_CFG_TOUCH_DELAY_500US (2 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#  define TSWC_CFG_TOUCH_DELAY_1MS   (3 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#  define TSWC_CFG_TOUCH_DELAY_5MS   (4 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#  define TSWC_CFG_TOUCH_DELAY_10MS  (5 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#  define TSWC_CFG_TOUCH_DELAY_50MS  (6 << TSWC_CFG_TOUCH_DELAY_SHIFT)
#define TSWC_CFG_AVE_CTRL_SHIFT      (6)       /* Bits 6-7: Average control */
#define TSWC_CFG_AVE_CTRL_MASK       (3 << TSWC_CFG_AVE_CTRL_SHIFT)
#  define TSWC_CFG_AVE_CTRL_1SAMPLE  (0 << TSWC_CFG_AVE_CTRL_SHIFT)
#  define TSWC_CFG_AVE_CTRL_2SAMPLES (1 << TSWC_CFG_AVE_CTRL_SHIFT)
#  define TSWC_CFG_AVE_CTRL_4SAMPLES (2 << TSWC_CFG_AVE_CTRL_SHIFT)
#  define TSWC_CFG_AVE_CTRL_8SAMPLES (3 << TSWC_CFG_AVE_CTRL_SHIFT)

/* Current status of FIFO */

#define FIFO_STA_FIFO_RESET          (1 << 0)  /* Bit 0:  Resets FIFO. All data in FIFO are cleared */
#define FIFO_STA_FIFO_TH_TRIG        (1 << 4)  /* Bit 0: 1 = FIFO size is at or beyond threshold */
#define FIFO_STA_FIFO_EMPTY          (1 << 5)  /* Bit 0: FIFO is empty */
#define FIFO_STA_FIFO_FULL           (1 << 6)  /* Bit 0: FIFO is full */
#define FIFO_STA_FIFO_OFLOW          (1 << 7)  /* Bit 0: FIFO is overflow */

/* Touchscreen controller FRACTION_Z */

#define TSC_FRACTIONZ_MASK           0x07

/* Touchscreen controller drive I */

#define TSC_IDRIVE                   (1 << 0)  /* Bit 0: MAX current on TSC driving channel */

/* Touchscreen controller shield */

#define TSC_SHIELD_YM                (1 << 0)  /* Bit 0: Ground Y- */
#define TSC_SHIELD_YP                (1 << 1)  /* Bit 1: Ground Y+ */
#define TSC_SHIELD_XM                (1 << 2)  /* Bit 2: Ground X- */
#define TSC_SHIELD_XP                (1 << 3)  /* Bit 3: Ground X+ */

/* Temperature sensor setup */

#define TEMP_CTRL_ENABLE             (1 << 0)  /* Bit 0: Enable */
#define TEMP_CTRL_ACQ                (1 << 1)  /* Bit 1: Acquire */
#define TEMP_CTRL_ACQ_MOD            (1 << 2)  /* Bit 2: 0=once, 1=every 10MS */
#define TEMP_CTRL_THRES_EN           (1 << 3)  /* Bit 3: Threshold enable */
#define TEMP_CTRL_THRES_RANGE        (1 << 4)  /* Bit 4: temperature threshold enable, 0='>=' 1='<' */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* A reference to a structure of this type must be passed to the STMPE11
 * driver.  This structure provides information about the configuration
 * of the STMPE11 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct stmpe11_config_s
{
  /* Device characterization */

  uint8_t  address;    /* 7-bit I2C address (only bits 0-6 used) */
  uint32_t frequency;  /* I2C frequency */

  /* If multiple STMPE11 devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

#ifndef CONFIG_STMPE11_MULTIPLE
  int      irq;        /* IRQ number received by interrupt handler. */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the STMPE11 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * attach  - Attach the STMPE11 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   * pendown - Return the state of the pen down GPIO input
   */

  int  (*attach)(FAR struct stmpe11_config_s *state, xcpt_t isr);
  void (*enable)(FAR struct stmpe11_config_s *state, bool enable);
  void (*clear)(FAR struct stmpe11_config_s *state);
  bool (*pendown)(FAR struct stmpe11_config_s *state);
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Name: stmpe11_register
 *
 * Description:
 *   Configure the STMPE11 to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An I2C or SPI driver instance
 *   config  - Persistant board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ********************************************************************************************/

#ifdef CONFIG_STMPE11_SPI
EXTERN int stmpe11_register(FAR struct spi_dev_s *dev,
                            FAR struct stmpe11_config_s *config,
                            int minor);
#else
EXTERN int stmpe11_register(FAR struct i2c_dev_s *dev,
                            FAR struct stmpe11_config_s *config,
                            int minor);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE11 */
#endif /* __INCLUDE_NUTTX_INPUT_STMPE11_H */
