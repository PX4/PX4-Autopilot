/**
 * Copyright (C) 2018 - 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    bmp3_defs.h
 * @date    01 July 2019
 * @version 1.1.3
 * @brief
 *
 */

/*! @file bmp3_defs.h
 * @brief Sensor driver for BMP3 sensor */

/*!
 * @defgroup BMP3 SENSOR API
 * @brief
 * @{*/
#ifndef BMP3_DEFS_H_
#define BMP3_DEFS_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/********************************************************/
/**\name Compiler switch macros */
/**\name Uncomment the below line to use floating-point compensation */
#ifndef BMP3_DOUBLE_PRECISION_COMPENSATION

/* #define BMP3_DOUBLE_PRECISION_COMPENSATION*/
#endif

/********************************************************/
/**\name Macro definitions */
/**\name I2C addresses */
#define BMP3_I2C_ADDR_PRIM                (0x76)
#define BMP3_I2C_ADDR_SEC                 (0x77)

/**\name BMP3 chip identifier */
#define BMP3_CHIP_ID                      (0x50)

/**\name BMP3 pressure settling time (micro secs)*/
#define BMP3_PRESS_SETTLE_TIME            (392)

/**\name BMP3 temperature settling time (micro secs) */
#define BMP3_TEMP_SETTLE_TIME             (313)

/**\name BMP3 adc conversion time (micro secs) */
#define BMP3_ADC_CONV_TIME                (2000)

/**\name Register Address */
#define BMP3_CHIP_ID_ADDR                 (0x00)
#define BMP3_ERR_REG_ADDR                 (0x02)
#define BMP3_SENS_STATUS_REG_ADDR         (0x03)
#define BMP3_DATA_ADDR                    (0x04)
#define BMP3_EVENT_ADDR                   (0x10)
#define BMP3_INT_STATUS_REG_ADDR          (0x11)
#define BMP3_FIFO_LENGTH_ADDR             (0x12)
#define BMP3_FIFO_DATA_ADDR               (0x14)
#define BMP3_FIFO_WM_ADDR                 (0x15)
#define BMP3_FIFO_CONFIG_1_ADDR           (0x17)
#define BMP3_FIFO_CONFIG_2_ADDR           (0x18)
#define BMP3_INT_CTRL_ADDR                (0x19)
#define BMP3_IF_CONF_ADDR                 (0x1A)
#define BMP3_PWR_CTRL_ADDR                (0x1B)
#define BMP3_OSR_ADDR                     (0X1C)
#define BMP3_CALIB_DATA_ADDR              (0x31)
#define BMP3_CMD_ADDR                     (0x7E)

/**\name Error status macros */
#define BMP3_FATAL_ERR                    (0x01)
#define BMP3_CMD_ERR                      (0x02)
#define BMP3_CONF_ERR                     (0x04)

/**\name Status macros */
#define BMP3_CMD_RDY                      (0x10)
#define BMP3_DRDY_PRESS                   (0x20)
#define BMP3_DRDY_TEMP                    (0x40)

/**\name Power mode macros */
#define BMP3_SLEEP_MODE                   (0x00)
#define BMP3_FORCED_MODE                  (0x01)
#define BMP3_NORMAL_MODE                  (0x03)

/**\name FIFO related macros */
/**\name FIFO enable  */
#define BMP3_ENABLE                       (0x01)
#define BMP3_DISABLE                      (0x00)

/**\name Interrupt pin configuration macros */
/**\name Open drain */
#define BMP3_INT_PIN_OPEN_DRAIN           (0x01)
#define BMP3_INT_PIN_PUSH_PULL            (0x00)

/**\name Level */
#define BMP3_INT_PIN_ACTIVE_HIGH          (0x01)
#define BMP3_INT_PIN_ACTIVE_LOW           (0x00)

/**\name Latch */
#define BMP3_INT_PIN_LATCH                (0x01)
#define BMP3_INT_PIN_NON_LATCH            (0x00)

/**\name Advance settings  */
/**\name I2c watch dog timer period selection */
#define BMP3_I2C_WDT_SHORT_1_25_MS        (0x00)
#define BMP3_I2C_WDT_LONG_40_MS           (0x01)

/**\name FIFO Sub-sampling macros */
#define BMP3_FIFO_NO_SUBSAMPLING          (0x00)
#define BMP3_FIFO_SUBSAMPLING_2X          (0x01)
#define BMP3_FIFO_SUBSAMPLING_4X          (0x02)
#define BMP3_FIFO_SUBSAMPLING_8X          (0x03)
#define BMP3_FIFO_SUBSAMPLING_16X         (0x04)
#define BMP3_FIFO_SUBSAMPLING_32X         (0x05)
#define BMP3_FIFO_SUBSAMPLING_64X         (0x06)
#define BMP3_FIFO_SUBSAMPLING_128X        (0x07)

/**\name Over sampling macros */
#define BMP3_NO_OVERSAMPLING              (0x00)
#define BMP3_OVERSAMPLING_2X              (0x01)
#define BMP3_OVERSAMPLING_4X              (0x02)
#define BMP3_OVERSAMPLING_8X              (0x03)
#define BMP3_OVERSAMPLING_16X             (0x04)
#define BMP3_OVERSAMPLING_32X             (0x05)

/**\name Filter setting macros */
#define BMP3_IIR_FILTER_DISABLE           (0x00)
#define BMP3_IIR_FILTER_COEFF_1           (0x01)
#define BMP3_IIR_FILTER_COEFF_3           (0x02)
#define BMP3_IIR_FILTER_COEFF_7           (0x03)
#define BMP3_IIR_FILTER_COEFF_15          (0x04)
#define BMP3_IIR_FILTER_COEFF_31          (0x05)
#define BMP3_IIR_FILTER_COEFF_63          (0x06)
#define BMP3_IIR_FILTER_COEFF_127         (0x07)

/**\name Odr setting macros */
#define BMP3_ODR_200_HZ                   (0x00)
#define BMP3_ODR_100_HZ                   (0x01)
#define BMP3_ODR_50_HZ                    (0x02)
#define BMP3_ODR_25_HZ                    (0x03)
#define BMP3_ODR_12_5_HZ                  (0x04)
#define BMP3_ODR_6_25_HZ                  (0x05)
#define BMP3_ODR_3_1_HZ                   (0x06)
#define BMP3_ODR_1_5_HZ                   (0x07)
#define BMP3_ODR_0_78_HZ                  (0x08)
#define BMP3_ODR_0_39_HZ                  (0x09)
#define BMP3_ODR_0_2_HZ                   (0x0A)
#define BMP3_ODR_0_1_HZ                   (0x0B)
#define BMP3_ODR_0_05_HZ                  (0x0C)
#define BMP3_ODR_0_02_HZ                  (0x0D)
#define BMP3_ODR_0_01_HZ                  (0x0E)
#define BMP3_ODR_0_006_HZ                 (0x0F)
#define BMP3_ODR_0_003_HZ                 (0x10)
#define BMP3_ODR_0_001_HZ                 (0x11)

/**\name API success code */
#define BMP3_OK                           (0)

/**\name API error codes */
#define BMP3_E_NULL_PTR                   (-1)
#define BMP3_E_DEV_NOT_FOUND              (-2)
#define BMP3_E_INVALID_ODR_OSR_SETTINGS   (-3)
#define BMP3_E_CMD_EXEC_FAILED            (-4)
#define BMP3_E_CONFIGURATION_ERR          (-5)
#define BMP3_E_INVALID_LEN                (-6)
#define BMP3_E_COMM_FAIL                  (-7)
#define BMP3_E_FIFO_WATERMARK_NOT_REACHED (-8)

/**\name API warning codes */
#define BMP3_W_SENSOR_NOT_ENABLED         (1)
#define BMP3_W_INVALID_FIFO_REQ_FRAME_CNT (2)

/**\name Macros to select the which sensor settings are to be set by the user.
 * These values are internal for API implementation. Don't relate this to
 * data sheet. */
#define BMP3_PRESS_EN_SEL                 (1 << 1)
#define BMP3_TEMP_EN_SEL                  (1 << 2)
#define BMP3_DRDY_EN_SEL                  (1 << 3)
#define BMP3_PRESS_OS_SEL                 (1 << 4)
#define BMP3_TEMP_OS_SEL                  (1 << 5)
#define BMP3_IIR_FILTER_SEL               (1 << 6)
#define BMP3_ODR_SEL                      (1 << 7)
#define BMP3_OUTPUT_MODE_SEL              (1 << 8)
#define BMP3_LEVEL_SEL                    (1 << 9)
#define BMP3_LATCH_SEL                    (1 << 10)
#define BMP3_I2C_WDT_EN_SEL               (1 << 11)
#define BMP3_I2C_WDT_SEL_SEL              (1 << 12)
#define BMP3_ALL_SETTINGS                 (0x7FF)

/**\name Macros to select the which FIFO settings are to be set by the user
 * These values are internal for API implementation. Don't relate this to
 * data sheet.*/
#define BMP3_FIFO_MODE_SEL                (1 << 1)
#define BMP3_FIFO_STOP_ON_FULL_EN_SEL     (1 << 2)
#define BMP3_FIFO_TIME_EN_SEL             (1 << 3)
#define BMP3_FIFO_PRESS_EN_SEL            (1 << 4)
#define BMP3_FIFO_TEMP_EN_SEL             (1 << 5)
#define BMP3_FIFO_DOWN_SAMPLING_SEL       (1 << 6)
#define BMP3_FIFO_FILTER_EN_SEL           (1 << 7)
#define BMP3_FIFO_FWTM_EN_SEL             (1 << 8)
#define BMP3_FIFO_FULL_EN_SEL             (1 << 9)
#define BMP3_FIFO_ALL_SETTINGS            (0x3FF)

/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.*/
#define BMP3_PRESS                        (1)
#define BMP3_TEMP                         (1 << 1)
#define BMP3_ALL                          (0x03)

/**\name Macros for bit masking */
#define BMP3_ERR_FATAL_MSK                (0x01)

#define BMP3_ERR_CMD_MSK                  (0x02)
#define BMP3_ERR_CMD_POS                  (0x01)

#define BMP3_ERR_CONF_MSK                 (0x04)
#define BMP3_ERR_CONF_POS                 (0x02)

#define BMP3_STATUS_CMD_RDY_MSK           (0x10)
#define BMP3_STATUS_CMD_RDY_POS           (0x04)

#define BMP3_STATUS_DRDY_PRESS_MSK        (0x20)
#define BMP3_STATUS_DRDY_PRESS_POS        (0x05)

#define BMP3_STATUS_DRDY_TEMP_MSK         (0x40)
#define BMP3_STATUS_DRDY_TEMP_POS         (0x06)

#define BMP3_OP_MODE_MSK                  (0x30)
#define BMP3_OP_MODE_POS                  (0x04)

#define BMP3_PRESS_EN_MSK                 (0x01)

#define BMP3_TEMP_EN_MSK                  (0x02)
#define BMP3_TEMP_EN_POS                  (0x01)

#define BMP3_IIR_FILTER_MSK               (0x0E)
#define BMP3_IIR_FILTER_POS               (0x01)

#define BMP3_ODR_MSK                      (0x1F)

#define BMP3_PRESS_OS_MSK                 (0x07)

#define BMP3_TEMP_OS_MSK                  (0x38)
#define BMP3_TEMP_OS_POS                  (0x03)

#define BMP3_FIFO_MODE_MSK                (0x01)

#define BMP3_FIFO_STOP_ON_FULL_MSK        (0x02)
#define BMP3_FIFO_STOP_ON_FULL_POS        (0x01)

#define BMP3_FIFO_TIME_EN_MSK             (0x04)
#define BMP3_FIFO_TIME_EN_POS             (0x02)

#define BMP3_FIFO_PRESS_EN_MSK            (0x08)
#define BMP3_FIFO_PRESS_EN_POS            (0x03)

#define BMP3_FIFO_TEMP_EN_MSK             (0x10)
#define BMP3_FIFO_TEMP_EN_POS             (0x04)

#define BMP3_FIFO_FILTER_EN_MSK           (0x18)
#define BMP3_FIFO_FILTER_EN_POS           (0x03)

#define BMP3_FIFO_DOWN_SAMPLING_MSK       (0x07)

#define BMP3_FIFO_FWTM_EN_MSK             (0x08)
#define BMP3_FIFO_FWTM_EN_POS             (0x03)

#define BMP3_FIFO_FULL_EN_MSK             (0x10)
#define BMP3_FIFO_FULL_EN_POS             (0x04)

#define BMP3_INT_OUTPUT_MODE_MSK          (0x01)

#define BMP3_INT_LEVEL_MSK                (0x02)
#define BMP3_INT_LEVEL_POS                (0x01)

#define BMP3_INT_LATCH_MSK                (0x04)
#define BMP3_INT_LATCH_POS                (0x02)

#define BMP3_INT_DRDY_EN_MSK              (0x40)
#define BMP3_INT_DRDY_EN_POS              (0x06)

#define BMP3_I2C_WDT_EN_MSK               (0x02)
#define BMP3_I2C_WDT_EN_POS               (0x01)

#define BMP3_I2C_WDT_SEL_MSK              (0x04)
#define BMP3_I2C_WDT_SEL_POS              (0x02)

#define BMP3_INT_STATUS_FWTM_MSK          (0x01)

#define BMP3_INT_STATUS_FFULL_MSK         (0x02)
#define BMP3_INT_STATUS_FFULL_POS         (0x01)

#define BMP3_INT_STATUS_DRDY_MSK          (0x08)
#define BMP3_INT_STATUS_DRDY_POS          (0x03)

/**\name    UTILITY MACROS  */
#define BMP3_SET_LOW_BYTE                 (0x00FF)
#define BMP3_SET_HIGH_BYTE                (0xFF00)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP3_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BMP3_SET_BITS(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | \
	 ((data << bitname##_POS) & bitname##_MSK))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | \
	 (data & bitname##_MSK))

#define BMP3_GET_BITS(reg_data, bitname)       ((reg_data & (bitname##_MSK)) >> \
		(bitname##_POS))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

#define BMP3_GET_LSB(var)                      (uint8_t)(var & BMP3_SET_LOW_BYTE)
#define BMP3_GET_MSB(var)                      (uint8_t)((var & BMP3_SET_HIGH_BYTE) >> 8)

/**\name Macros related to size */
#define BMP3_CALIB_DATA_LEN          (21)
#define BMP3_P_AND_T_HEADER_DATA_LEN (7)
#define BMP3_P_OR_T_HEADER_DATA_LEN  (4)
#define BMP3_P_T_DATA_LEN            (6)
#define BMP3_GEN_SETT_LEN            (7)
#define BMP3_P_DATA_LEN              (3)
#define BMP3_T_DATA_LEN              (3)
#define BMP3_SENSOR_TIME_LEN         (3)
#define BMP3_FIFO_MAX_FRAMES         (73)

/********************************************************/

/*!
 * @brief Interface selection Enums
 */
enum bmp3_intf {
	/*! SPI interface */
	BMP3_SPI_INTF,

	/*! I2C interface */
	BMP3_I2C_INTF
};

/********************************************************/

/*!
 * @brief Type definitions
 */
typedef int8_t (*bmp3_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bmp3_delay_fptr_t)(uint32_t period);

/********************************************************/

/*!
 * @brief Register Trim Variables
 */
#pragma pack(push,1)
struct bmp3_reg_calib_data {
	/**
	 * @ Trim Variables
	 */

	/**@{*/
	uint16_t par_t1;
	uint16_t par_t2;
	int8_t par_t3;
	int16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int8_t par_p4;
	uint16_t par_p5;
	uint16_t par_p6;
	int8_t par_p7;
	int8_t par_p8;
	int16_t par_p9;
	int8_t par_p10;
	int8_t par_p11;
	int64_t t_lin;

	/**@}*/
};
#pragma pack(pop)

/*!
 * @brief bmp3 advance settings
 */
struct bmp3_adv_settings {
	/*! i2c watch dog enable */
	uint8_t i2c_wdt_en;

	/*! i2c watch dog select */
	uint8_t i2c_wdt_sel;
};

/*!
 * @brief bmp3 odr and filter settings
 */
struct bmp3_odr_filter_settings {
	/*! Pressure oversampling */
	uint8_t press_os;

	/*! Temperature oversampling */
	uint8_t temp_os;

	/*! IIR filter */
	uint8_t iir_filter;

	/*! Output data rate */
	uint8_t odr;
};

/*!
 * @brief bmp3 sensor status flags
 */
struct bmp3_sens_status {
	/*! Command ready status */
	uint8_t cmd_rdy;

	/*! Data ready for pressure */
	uint8_t drdy_press;

	/*! Data ready for temperature */
	uint8_t drdy_temp;
};

/*!
 * @brief bmp3 interrupt status flags
 */
struct bmp3_int_status {
	/*! fifo watermark interrupt */
	uint8_t fifo_wm;

	/*! fifo full interrupt */
	uint8_t fifo_full;

	/*! data ready interrupt */
	uint8_t drdy;
};

/*!
 * @brief bmp3 error status flags
 */
struct bmp3_err_status {
	/*! fatal error */
	uint8_t fatal;

	/*! command error */
	uint8_t cmd;

	/*! configuration error */
	uint8_t conf;
};

/*!
 * @brief bmp3 status flags
 */
struct bmp3_status {
	/*! Interrupt status */
	struct bmp3_int_status intr;

	/*! Sensor status */
	struct bmp3_sens_status sensor;

	/*! Error status */
	struct bmp3_err_status err;

	/*! power on reset status */
	uint8_t pwr_on_rst;
};

/*!
 * @brief bmp3 interrupt pin settings
 */
struct bmp3_int_ctrl_settings {
	/*! Output mode */
	uint8_t output_mode;

	/*! Active high/low */
	uint8_t level;

	/*! Latched or Non-latched */
	uint8_t latch;

	/*! Data ready interrupt */
	uint8_t drdy_en;
};

/*!
 * @brief bmp3 device settings
 */
struct bmp3_settings {
	/*! Power mode which user wants to set */
	uint8_t op_mode;

	/*! Enable/Disable pressure sensor */
	uint8_t press_en;

	/*! Enable/Disable temperature sensor */
	uint8_t temp_en;

	/*! ODR and filter configuration */
	struct bmp3_odr_filter_settings odr_filter;

	/*! Interrupt configuration */
	struct bmp3_int_ctrl_settings int_settings;

	/*! Advance settings */
	struct bmp3_adv_settings adv_settings;
};

/*!
 * @brief bmp3 fifo frame
 */
struct bmp3_fifo_data {
	/*! Data buffer of user defined length is to be mapped here
	 * 512 + 4 */
	uint8_t buffer[516];

	/*! Number of bytes of data read from the fifo */
	uint16_t byte_count;

	/*! Number of frames to be read as specified by the user */
	uint8_t req_frames;

	/*! Will be equal to length when no more frames are there to parse */
	uint16_t start_idx;

	/*! Will contain the no of parsed data frames from fifo */
	uint8_t parsed_frames;

	/*! Configuration error */
	uint8_t config_err;

	/*! Sensor time */
	uint32_t sensor_time;

	/*! FIFO input configuration change */
	uint8_t config_change;

	/*! All available frames are parsed */
	uint8_t frame_not_available;
};

/*!
 * @brief bmp3 fifo configuration
 */
struct bmp3_fifo_settings {
	/*! enable/disable */
	uint8_t mode;

	/*! stop on full enable/disable */
	uint8_t stop_on_full_en;

	/*! time enable/disable */
	uint8_t time_en;

	/*! pressure enable/disable */
	uint8_t press_en;

	/*! temperature enable/disable */
	uint8_t temp_en;

	/*! down sampling rate */
	uint8_t down_sampling;

	/*! filter enable/disable */
	uint8_t filter_en;

	/*! FIFO watermark enable/disable */
	uint8_t fwtm_en;

	/*! FIFO full enable/disable */
	uint8_t ffull_en;
};

/*!
 * @brief bmp3 bmp3 FIFO
 */
struct bmp3_fifo {
	/*! FIFO frame structure */
	struct bmp3_fifo_data data;

	/*! FIFO config structure */
	struct bmp3_fifo_settings settings;
};

#ifdef BMP3_DOUBLE_PRECISION_COMPENSATION

/*!
 * @brief Quantized Trim Variables
 */
struct bmp3_quantized_calib_data {
	/**
	 * @ Quantized Trim Variables
	 */

	/**@{*/
	double par_t1;
	double par_t2;
	double par_t3;
	double par_p1;
	double par_p2;
	double par_p3;
	double par_p4;
	double par_p5;
	double par_p6;
	double par_p7;
	double par_p8;
	double par_p9;
	double par_p10;
	double par_p11;
	double t_lin;

	/**@}*/
};

/*!
 * @brief Calibration data
 */
struct bmp3_calib_data {
	/*! Quantized data */
	struct bmp3_quantized_calib_data quantized_calib_data;

	/*! Register data */
	struct bmp3_reg_calib_data reg_calib_data;
};

/*!
 * @brief bmp3 sensor structure which comprises of temperature and pressure
 * data.
 */
struct bmp3_data {
	/*! Compensated temperature */
	double temperature;

	/*! Compensated pressure */
	double pressure;
};

#else

/*!
 * @brief bmp3 sensor structure which comprises of temperature and pressure
 * data.
 */
struct bmp3_data {
	/*! Compensated temperature */
	int64_t temperature;

	/*! Compensated pressure */
	uint64_t pressure;
};

/*!
 * @brief Calibration data
 */
struct bmp3_calib_data {
	/*! Register data */
	struct bmp3_reg_calib_data reg_calib_data;
};

#endif /* BMP3_DOUBLE_PRECISION_COMPENSATION */

/*!
 * @brief bmp3 sensor structure which comprises of un-compensated temperature
 * and pressure data.
 */
struct bmp3_uncomp_data {
	/*! un-compensated pressure */
	uint32_t pressure;

	/*! un-compensated temperature */
	uint32_t temperature;
};

/*!
 * @brief bmp3 device structure
 */
struct bmp3_dev {
	/*! Chip Id */
	uint8_t chip_id;

	/*! Device Id */
	uint8_t dev_id;

	/*! SPI/I2C interface */
	enum bmp3_intf intf;

	/*! Decide SPI or I2C read mechanism */
	uint8_t dummy_byte;

	/*! Read function pointer */
	bmp3_com_fptr_t read;

	/*! Write function pointer */
	bmp3_com_fptr_t write;

	/*! Delay function pointer */
	bmp3_delay_fptr_t delay_ms;

	/*! Trim data */
	struct bmp3_calib_data calib_data;

	/*! Sensor Settings */
	struct bmp3_settings settings;

	/*! Sensor and interrupt status flags */
	struct bmp3_status status;

	/*! FIFO data and settings structure */
	struct bmp3_fifo *fifo;
};

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMP3_DEFS_H_ */
/** @}*/
/** @}*/
