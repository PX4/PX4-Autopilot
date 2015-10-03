/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <pthread.h>
#include <math.h>

#include <px4_log.h>

#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <dev_fs_lib.h>
#include <dev_fs_lib_i2c.h>

#include "pressure_api.h"
#include "pressure_context.h"
#include "pressure_sensor.h"

static uint64_t get_time_1us(void) {
	uint64_t timestamp = 0;

	struct timespec ts = { 0, 0 };

	clock_gettime(CLOCK_MONOTONIC, &ts);

	timestamp += ts.tv_sec * 1000000;
	timestamp += ts.tv_nsec / 1000;

	return timestamp;
}

static void *read_thread(void *context_parm) {
	struct pressure_context *context;
	int result;

	if (context_parm == NULL) {
		PX4_ERR("error: context is null");
		return (void *) -1;
	}
	context = (struct pressure_context *) context_parm;

	/*
	 * Indicate that the thread is running, used by the bmp280_stop_thread function to
	 * gracefully exit this thread.
	 */
	context->is_thread_running = TRUE;

	while (context->is_thread_running) {
		/* Wait for the next pressure sample. */
		usleep(MAX_INTERVAL_BETWEEN_SAMPLES_IN_USECS);

		/* Read the data from the sensor into the local buffer. */
		result = pressure_sensor_read_data((uint32_t) context);
		if (result < 0) {
			PX4_ERR("error: unable to read pressure sensor data");
			context->last_error = -EIO;
			return (void *) -1;
		}

		/*
		 * Lock access to the cached copy of the data in the context data structure, to
		 * prevent simultaneous reads.
		 */
		pthread_mutex_lock(&context->mutex);

		/*
		 * Save the latest values in the context for access by the caller while we wait
		 * for the next available sample.
		 */
		context->sensor_data.pressure_in_pa = pressure_sensor_get_pressure_in_pa(
				(uint32_t) context);
		context->sensor_data.temperature_in_c =
				pressure_sensor_get_temperature_in_c((uint32_t) context);
		context->sensor_data.last_read_time_in_usecs = get_time_1us();
		context->sensor_data.sensor_read_counter++;

		/* Unlock access to the cached copy of the data. */
		pthread_mutex_unlock(&context->mutex);

		/*
		 * In case there are caller's blocked on access to the data, signal the existence
		 * of a new sample.
		 */
		sem_post(&context->new_data_sem);
	}

	return (void *) 0;
}

static void copy_data_safely(struct pressure_context *context,
		struct pressure_sensor_data *out_data) {
	if (context == NULL || out_data == NULL) {
		PX4_ERR("error: context or out_data is null");
		return;
	}

	/*
	 * Request the lock to assure that the sensor data buffer is not
	 * being updated.
	 */
	if (pthread_mutex_lock(&context->mutex) != 0) {
		PX4_ERR("error: unable to secure the lock");
		return;
	}

	/*
	 * Copy the last sensor data obtained from the baro sensor into the caller's
	 * output buffer.
	 */
	memcpy((void *) out_data, (void *) &context->sensor_data,
			sizeof(struct pressure_sensor_data));

	/*
	 * Free the lock to allow the sensor read thread to periodically update the
	 * sensor data buffer.
	 */
	pthread_mutex_unlock(&context->mutex);
}

static int start_thread(struct pressure_context *context) {
	int result;

	if (context == NULL) {
		PX4_ERR("error: context or out_data is null");
		return -EINVAL;
	}

	result = pthread_create(&context->read_thread_handle, pthread_attr_default,
			read_thread, context);
	if (result != 0) {
		PX4_ERR("error: unable to create the read thread: %d", result);
		return -EINVAL;
	}

	return result;
}

static int stop_thread(struct pressure_context *context) {
	int result;
	void *thread_return_value;

	if (context == NULL) {
		PX4_ERR("error: context or out_data is null");
		context->last_error = -EINVAL;
		return -1;
	}

	/* Change the state of the while loop transition variable for this context. */
	context->is_thread_running = FALSE;

	/* Wait for the thread to exit. */
	result = pthread_join(context->read_thread_handle, &thread_return_value);
	if (result != 0) {
		PX4_ERR("error: unable to wait for the read thread to terminate: %d",
				result);
		context->last_error = -ENOEXEC;
		return -1;
	}

	PX4_INFO("thread successfully stopped, return value: 0x%X",
			thread_return_value);
	return 0;
}

static int i2c_read_reg(int fildes, uint8_t address, uint8_t *out_buffer,
		int length) {
	struct dspal_i2c_ioctl_combined_write_read ioctl_write_read;
	uint8_t write_buffer[1];

	if (fildes == 0) {
		PX4_ERR("error: i2c bus is not yet opened");
		return -1;
	}

	/* Save the address of the register to read from in the write buffer for the combined write. */
	write_buffer[0] = address;
	ioctl_write_read.write_buf = write_buffer;
	ioctl_write_read.write_buf_len = 1;
	ioctl_write_read.read_buf = out_buffer;
	ioctl_write_read.read_buf_len = length;
	int bytes_written = ioctl(fildes, I2C_IOCTL_RDWR, &ioctl_write_read);
	if (bytes_written != length) {
		PX4_ERR(
				"error: read register reports a read of %d bytes, but attempted to set %d bytes",
				bytes_written, length);
		return -1;
	}

	return 0;
}

static int i2c_write_reg(int fildes, uint8_t address, uint8_t *in_buffer,
		int length) {
	uint8_t write_buffer[MAX_LEN_TRANSMIT_BUFFER_IN_BYTES];

	/*
	 * Verify that the length of the caller's buffer does not exceed the local stack
	 * buffer with one additional byte for the register ID.
	 */
	if (length + 1 > MAX_LEN_TRANSMIT_BUFFER_IN_BYTES) {
		PX4_ERR("error: caller's buffer exceeds size of local buffer");
		return -1;
	}
	if (fildes == 0) {
		PX4_ERR("error: i2c bus is not yet opened");
		return -1;
	}

	/* Save the address of the register to read from in the write buffer for the combined write. */
	write_buffer[0] = address;
	memcpy(&write_buffer[1], in_buffer, length);
	int bytes_written = write(fildes, (char *) write_buffer, length + 1);
	if (bytes_written != length + 1) {
		PX4_ERR("error: i2c write failed. Reported %d bytes written",
				bytes_written);
		return -1;
	}

	return 0;
}

int pressure_api_get_last_error(uint32_t handle) {
	struct pressure_context *context = (struct pressure_context *) handle;
	int last_error;

	if (context == NULL) {
		PX4_ERR("error: context parameter is invalid");
		return -EINVAL;
	}

	/* Return the last error, and clear it from the context. */
	last_error = context->last_error;
	context->last_error = 0;

	return last_error;
}

/*
 * If is_new_data_required is true, then block until new sensor data has arrived, otherwise return the currently
 * cached value.
 */
int pressure_api_get_sensor_data(uint32_t handle,
		struct pressure_sensor_data *out_data, bool is_new_data_required) {
	struct pressure_context *context = (struct pressure_context *) handle;

	if (context == NULL) {
		PX4_ERR("error: context parameter is invalid");
		return -1;
	}

	if (out_data == NULL) {
		PX4_ERR("error: out_data parameter is invalid");
		context->last_error = -EINVAL;
		return -1;
	}

	if (is_new_data_required) {
		/* Block here indefinitely until new data is detected. */
		if (sem_wait(&context->new_data_sem) == 0) {
			/* Copy data to the caller's buffer. */
			copy_data_safely(context, out_data);
		} else {
			PX4_ERR("error: unable to block waiting for new sensor data");
			context->last_error = -ENOTSUP;
			return -1;
		}
	} else {
		/* Copy existing data to the caller's buffer, without waiting. */
		copy_data_safely(context, out_data);
	}

	PX4_INFO(
			"read sensor data, time: %lld, read counter: %lld, pressure (Pascals): %u, temp (C): %f",
			out_data->last_read_time_in_usecs, out_data->sensor_read_counter, out_data->pressure_in_pa, out_data->temperature_in_c);
	return 0;
}

int pressure_api_set_altimeter(uint32_t handle,
		float altimeter_setting_in_mbars) {
	struct pressure_context *context = (struct pressure_context *) handle;

	if (context == NULL) {
		PX4_ERR("error: context parameter is invalid");
		return -1;
	}

	context->altimeter_setting_in_mbars = altimeter_setting_in_mbars;
	PX4_INFO("setting optional altimeter setting in mbars: %f",
			altimeter_setting_in_mbars);
	PX4_INFO("WARNING setting altimeter in mbars is untested");

	return 0;
}

int pressure_api_open(const char *i2c_device_path, uint32_t* out_handle) {
	int result = 0;
	int status = 0;
	sem_t new_data_sem;
	pthread_mutexattr_t mutex_attr;
	pthread_mutex_t mutex = 0;
	struct pressure_context *context = NULL;

	if (i2c_device_path == NULL || out_handle == NULL) {
		PX4_ERR("Parameter is null");
		result = -1;
		goto exit;
	}
	*out_handle = 0;

	/* Open the device path specified by the caller. */
	result = pressure_sensor_open(i2c_device_path, i2c_read_reg, i2c_write_reg,
			(uint32_t *) &context);
	if (result == -1) {
		PX4_ERR("Unable to open the device path: %s", i2c_device_path);
		result = -1;
		goto exit;
	}

	/*
	 * Allocate the mutex and semaphore used to update the current sensor readings and
	 * signal when a new reading is available.
	 */
	status = sem_init(&new_data_sem, 0, 0);
	if (status != 0) {
		PX4_ERR("error: unable to allocate a semaphore: %d", status);
		result = -1;
		goto exit;
	}
	status = pthread_mutexattr_init(&mutex_attr);
	if (status != 0) {
		PX4_ERR("error: unable to initialize mutex attributes: %d", status);
		result = -1;
		goto exit;
	}
	status = pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_DEFAULT);
	if (status != 0) {
		PX4_ERR("error: unable to set the type of the mutex: %d", status);
		result = -1;
		goto exit;
	}
	status = pthread_mutex_init(&mutex, &mutex_attr);
	if (status != 0) {
		PX4_ERR("error: unable to allocate/initialize a mutex: %d", status);
		result = -1;
		goto exit;
	}

	strncpy(context->device_path, i2c_device_path, MAX_LEN_DEVICE_PATH_IN_BYTES);
	context->new_data_sem = new_data_sem;
	context->mutex = mutex;

	/*
	 * Start the read thread that will run periodically to sample the sensor data.
	 */
	status = start_thread(context);
	if (status != 0) {
		PX4_ERR("error: starting read thread failed");
		goto exit;
	}

	exit:
	/* Did an error occur above, and if so clean up. */
	if (result != 0) {
		if (context != NULL) {
			pressure_sensor_close((uint32_t) context);
		}
		if (mutex_attr.is_initialized) {
			pthread_mutexattr_destroy(&mutex_attr);
		}
		if (mutex != 0) {
			pthread_mutex_destroy(&mutex);
		}
		if (new_data_sem._magic != 0) {
			sem_destroy(&new_data_sem);
		}
	}
	/* Success, so return the handle in the caller's out variable. */
	else {
		*out_handle = (uint32_t) context;
	}

	return result;
}

void pressure_api_close(uint32_t handle) {
	struct pressure_context *context = (struct pressure_context *) handle;

	if (handle == 0) {
		PX4_ERR("error: invalid parameter");
		return;
	}

	stop_thread(context);
	pthread_mutex_destroy(&context->mutex);
	sem_destroy(&context->new_data_sem);

	/* Close the device specific resources for this driver. */
	pressure_sensor_close(handle);
}
