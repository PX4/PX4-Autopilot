/****************************************************************************
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
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

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file
 * The declarations in this file are released to DSPAL users and are used to
 * make file I/O call's for access to slave devices on a SPI bus.  Many of the
 * data structures are used in the parameter of the IOCTL functions to define the
 * behavior of the particular IOCTL.
 *
 * Sample source files are referenced to demonstrate how a particular IOCTL
 * or data structure is used in the context of the POSIX standard file I/O functions
 * (open/close/read/write/ioctl).
 *
 * @par Reading SPI Data
 * To read data that has accumulated since the last call to read (see the rx_func_ptr_t to define an
 * optional receive data callback) the buffer parameter of the read function must reference a buffer
 * large enough to contain all of the accumulated data.  If the buffer is not large enough, some portion of the
 * accumulated will be copied to the buffer.  The actual length of the data copied to the caller's buffer is
 * specified in the return value of the read function.
 *
 * @par Writing SPI Data
 * To write data to the SPI bus slave device a buffer parameter containing the data to be transmitted
 * must be passed to the write function.  After the data is queued for transmit, the write function will
 * return immediately to the caller.
 *
 * @par
 * Sample source code for read/write data to a SPI slave device is included below:
 * @include spi_test_imp.c
 */

/**
 * @brief
 * The SPI device path uses the following format:
 * /dev/spi-{bus number}
 * Bus numbers at 1 (/dev/spi-1) and go to the max number of BLSPs supported by the SoC.
 */
#define DEV_FS_SPI_DEVICE_TYPE_STRING  "/dev/spi-"

/**
 * The maximum length of any receive or transmit over SPI bus.
 */
#define DSPAL_SPI_TRANSMIT_BUFFER_LENGTH 512
#define DSPAL_SPI_RECEIVE_BUFFER_LENGTH  512

/**
 * @brief
 * List of IOCTL's used for setting SPI options and requesting certain SPI operations that
 * are not performed by the read/write calls.
 */
enum DSPAL_SPI_IOCTLS {
	SPI_IOCTL_INVALID = -1,   /**< invalid IOCTL code, used to return an error */
	SPI_IOCTL_SET_OPTIONS,    /**< used to configure certain options for communicating on the SPI bus */
	SPI_IOCTL_LOOPBACK_TEST,  /**< activate the internal loopback test mode of the spi bus */
	SPI_IOCTL_RDWR,           /**< used to initiate a write/read batch transfer */
	SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ,  /**< use this to set the SPI bus speed in HZ */
	SPI_IOCTL_SET_SPI_MODE,   /**< use this to set the SPI mode */
	SPI_IOCTL_MAX_NUM,        /**< number of valid IOCTL codes defined for the I2C bus */
};

/**
 * @brief
 * List of valid loopback states.
 */
enum DSPAL_SPI_LOOPBACK_TEST_STATE {
	SPI_LOOPBACK_STATE_UNCONFIGURED, /**< initial loopback state indicating that it is neither enabled, nor disabled */
	SPI_LOOPBACK_STATE_DISABLED,     /**< specifies that the loopback state should be disabled */
	SPI_LOOPBACK_STATE_ENABLED,      /**< specifies that the loopback state should be enabled */
};

/**
* In the idle state whether the SPI clk is high or low.
*/
enum SPI_CLOCK_POLARITY_TYPE {
	SPI_CLOCK_IDLE_LOW,	/**< CLK signal is low when idle.*/
	SPI_CLOCK_IDLE_HIGH,	/**< CLK signal is high when idle.*/
	SPI_CLOCK_INVALID = 0x7FFFFFFF
};


/**
* Shift mode, detemines which signal (input or output) is sampled first.
*/
enum SPI_SHIFT_MODE_TYPE {
	SPI_INPUT_FIRST,		/**< In both Master and slave input Bit is shifted in first.*/
	SPI_OUTPUT_FIRST,		/**< In both Master and slave  output Bit is shifted in first*/
	SPI_SHIFT_PADDING = 0xFFFFFFFF,
};

/**
 * Callback function indicating that new data has been received and is ready to be read.
 * @param event
 * Reserved for future use.
 * @param
 * Reserved for future use.
 */
typedef void (*spi_rx_func_ptr_t)(int event, void *);

/**
 * Callback function used to indicate that the transmission of all enqueued data is
 * completed.
 * @param event
 * Reserved for future use.
 * @param
 * Reserved for future use.
 */
typedef void (*spi_tx_func_ptr_t)(int event, void *);


/**
 * Structure passed to the SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ IOCTL call.  Specifies the
 * speed of the SPI bus communcations to the slave device.
 */
struct dspal_spi_ioctl_set_bus_frequency {
	uint32_t bus_frequency_in_hz;  /**< the maximum speed of the bus for high speed data transfers */
};

/**
 * Structure passed to the SPI_IOCTL_SET_SPI_MODE IOCTL call.  Specifies
 * the SPI bus mode to the slave device. if not set, use the default mode 3.
 */
struct dspal_spi_ioctl_set_spi_mode {
	enum SPI_CLOCK_POLARITY_TYPE eClockPolarity;/**< Clock polarity  type to be used for the SPI core.*/

	/* This parameter specifies whether the SPI core operates in OUTPUT
	 * or INPUT FIRST Mode. This specifies whether the shift register
	 * latches the DATA at the input pin on the rising or falling edge */
	enum SPI_SHIFT_MODE_TYPE eShiftMode;/**< Shift mode type to be used for SPI core.*/
};

/**
 * Structure passed to the SPI_IOCTL_SET_OPTIONS IOCTL call.  Specifies certain SPI bus options and capabilities.
 *
 * TODO-JYW: Add a void* parameter to this structure to allow the caller to specify the value
 * of the void* passed in the tx_data_callback and rx_data_callback.
 */
struct dspal_spi_ioctl_set_options {
	uint32_t slave_address;  		/**< the address of the slave device to communicate with */
	int is_tx_data_synchronous; 		/**< not yet supported, should the transmit data callback be called to indicate when data is fully transmitted */
	spi_tx_func_ptr_t tx_data_callback; 	/**< optional, not yet supported, called when transmit transfer is complete */
	spi_rx_func_ptr_t rx_data_callback; 	/**< optional, not yet supported, called when new data is ready to be read */
};

/**
 * Structure passed to the SPI_IOCTL_RDWR IOCTL call.  Specifies the address and length of the
 * read and write buffers used in a combined read/write operation on the SPI bus.
 */
struct dspal_spi_ioctl_read_write {
	void *read_buffer;  		/**< the address of the buffer used for data read from the slave device. */
	uint32_t read_buffer_length; 	/**< the length of the buffer referenced by the read_buffer parameter. */
	void *write_buffer; 		/**< the address of the buffer containing the data to write to the slave device. */
	uint32_t write_buffer_length; 	/**< the length of the buffer referenced by the write_buffer paarameter. */
};

/**
 * Structure passed to the SPI_IOCTL_LOOPBACK_TEST call. Specifies the desired state of the loopback
 * test mode.
 */
struct dspal_spi_ioctl_loopback {
	enum DSPAL_SPI_LOOPBACK_TEST_STATE state; /**< the state indicating if loopback mode is enabled or disabled. */
};
