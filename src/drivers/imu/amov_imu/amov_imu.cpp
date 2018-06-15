/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file amov_imu.cpp
 *
 * Driver for the AMOV-IMU (AMOV Lab, Chengdu, China) connected via
 * SPI or UART.
 *
 * When the device is on the SPI bus the hrt is used to provide thread of
 * execution to the driver.
 *
 * When the device is on the UART bus a high-priority thread is used provide thread of
 * execution to the driver.
 *
 * At most 3 devices are supported for the external UART IMUs for high reliability
 *
 *
 * @author Jin Wu (Zarathustra)
 *
 * @email: jin_wu_uestc@hotmail.com
 * @web: https://github.com/zarathustr
 *       http://www.jinwu.science
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <px4_workqueue.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "amov_imu.h"


#define AMOV_IMU_TIMER_REDUCTION				000

enum AMOV_IMU_DEVICE_TYPE {
    AMOV_IMU_DEVICE_TYPE_SPI = 0,
    AMOV_IMU_DEVICE_TYPE_UART
};

enum AMOV_IMU_BUS {
    AMOV_IMU_BUS_ALL = 0,
    AMOV_IMU_BUS_SPI,
    AMOV_IMU_BUS_UART
};

class AMOV_IMU_gyro;
class AMOV_IMU_mag;

class AMOV_IMU : public device::CDev
{
public:
    AMOV_IMU(device::Device *interface, const char *path_accel,
             const char *path_gyro, const char *path_mag,
             enum Rotation rotation,
                int device_type, ORB_PRIO priority);
    virtual ~AMOV_IMU();

    virtual int                 init();

    virtual ssize_t             read(struct file *filp, char *buffer, size_t buflen);
    virtual int                 ioctl(struct file *filp, int cmd, unsigned long arg);

    void                        task_main();
    void                        status();

protected:
    Device                      *_interface;

    friend class                AMOV_IMU_gyro;
    friend class                AMOV_IMU_mag;

    virtual ssize_t             gyro_read(struct file *filp, char *buffer, size_t buflen);
    virtual int                 gyro_ioctl(struct file *filp, int cmd, unsigned long arg);
    virtual ssize_t             mag_read(struct file *filp, char *buffer, size_t buflen);
    virtual int                 mag_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
    int                         _device_type;
    AMOV_IMU_gyro               *_gyro;
    AMOV_IMU_mag                *_mag;

    struct hrt_call             _call;
    unsigned                    _call_interval;

    ringbuffer::RingBuffer      *_accel_reports;

    struct accel_calibration_s  _accel_scale;
    orb_advert_t                _accel_topic;
    int                         _accel_orb_class_instance;
    int                         _accel_class_instance;

    ringbuffer::RingBuffer      *_gyro_reports;
    struct gyro_calibration_s   _gyro_scale;
    ringbuffer::RingBuffer      *_mag_reports;
    struct mag_calibration_s    _mag_scale;

    unsigned                    _sample_rate;
    perf_counter_t              _accel_reads;
    perf_counter_t              _gyro_reads;
    perf_counter_t              _mag_reads;
    perf_counter_t              _sample_perf;
    perf_counter_t              _bad_transfers;
    perf_counter_t              _bad_registers;
    perf_counter_t              _good_transfers;
    perf_counter_t              _reset_retries;
    perf_counter_t              _duplicates;
    perf_counter_t              _controller_latency_perf;

    uint8_t                     _register_wait;
    uint64_t                    _reset_wait;

    math::LowPassFilter2p       _accel_filter_x;
    math::LowPassFilter2p       _accel_filter_y;
    math::LowPassFilter2p       _accel_filter_z;
    math::LowPassFilter2p       _gyro_filter_x;
    math::LowPassFilter2p       _gyro_filter_y;
    math::LowPassFilter2p       _gyro_filter_z;

    Integrator                  _accel_int;
    Integrator                  _gyro_int;
    uint64_t                    _good_trans;
    uint64_t                    _bad_trans;

    enum Rotation               _rotation;


    // last temperature reading for print_info()
    float                       _last_temperature;

    // keep last accel reading for duplicate detection
    uint16_t                    _last_accel[3];
    bool                        _got_duplicate;
    int                         _task;
    ORB_PRIO                    _priority;
    uint8_t                     _imu_buf_length;
    bool                        _uart_set;

    /**
     * Start automatic measurement.
     */
    void                        start();

    /**
     * Stop automatic measurement.
     */
    void                        stop();

    /**
     * Reset chip.
     *
     * Resets the chip and measurements ranges, but not scale and offset.
     */
    int                         reset();

    /**
     * Static trampoline from the hrt_call context; because we don't have a
     * generic hrt wrapper yet.
     *
     * Called by the HRT in interrupt context at the specified rate if
     * automatic polling is enabled.
     *
     * @param arg		Instance pointer for the driver that is polling.
     */
    static void                 measure_trampoline(void *arg);

    /**
     * Fetch measurements from the sensor and update the report buffers.
     */
    int                         measure();

    static void task_main_trampoline(int argc, char *argv[]);

    /* do not allow to copy this class due to pointer data members */
    AMOV_IMU(const AMOV_IMU &);
    AMOV_IMU operator=(const AMOV_IMU &);

};

/**
 * Helper class implementing the gyro driver node.
 */
class AMOV_IMU_gyro : public device::CDev
{
public:
    AMOV_IMU_gyro(AMOV_IMU *parent, const char *path);
    ~AMOV_IMU_gyro();

    virtual ssize_t             read(struct file *filp, char *buffer, size_t buflen);
    virtual int                 ioctl(struct file *filp, int cmd, unsigned long arg);

    virtual int                 init();

protected:
    friend class                AMOV_IMU;

    void                        parent_poll_notify();

private:
    AMOV_IMU                    *_parent;
    orb_advert_t                _gyro_topic;
    int                         _gyro_orb_class_instance;
    int                         _gyro_class_instance;

    /* do not allow to copy this class due to pointer data members */
    AMOV_IMU_gyro(const AMOV_IMU_gyro &);
    AMOV_IMU_gyro operator=(const AMOV_IMU_gyro &);
};


/**
 * Helper class implementing the mag driver node.
 */
class AMOV_IMU_mag : public device::CDev
{
public:
    AMOV_IMU_mag(AMOV_IMU *parent, const char *path);
    ~AMOV_IMU_mag();

    virtual ssize_t             read(struct file *filp, char *buffer, size_t buflen);
    virtual int                 ioctl(struct file *filp, int cmd, unsigned long arg);

    virtual int                 init();

protected:
    friend class                AMOV_IMU;

    void                        parent_poll_notify();

private:
    AMOV_IMU                    *_parent;
    orb_advert_t                _mag_topic;
    int                         _mag_orb_class_instance;
    int                         _mag_class_instance;

    /* do not allow to copy this class due to pointer data members */
    AMOV_IMU_mag(const AMOV_IMU_mag &);
    AMOV_IMU_mag operator=(const AMOV_IMU_mag &);
};


/** driver 'main' command */
extern "C" { __EXPORT int amov_imu_main(int argc, char *argv[]); }

AMOV_IMU *g_imu = nullptr;

extern IMU3DMPacket _io_buffer_storage[MAX_DEV_NUM];

namespace amov_imu {
static bool    is_spi = false;
static bool    is_uart = false;
static device::Device *interface[MAX_DEV_NUM] = {nullptr, nullptr, nullptr};
static AMOV_IMU * dev[MAX_DEV_NUM] = {nullptr, nullptr, nullptr};
static const char *accelpath[MAX_DEV_NUM] = {
    AMOV_IMU_DEVICE_PATH_ACCEL1,
    AMOV_IMU_DEVICE_PATH_ACCEL2,
    AMOV_IMU_DEVICE_PATH_ACCEL3
};
static const char *gyropath[MAX_DEV_NUM] = {
    AMOV_IMU_DEVICE_PATH_GYRO1,
    AMOV_IMU_DEVICE_PATH_GYRO2,
    AMOV_IMU_DEVICE_PATH_GYRO3
};
static const char *magpath[MAX_DEV_NUM] = {
    AMOV_IMU_DEVICE_PATH_MAG1,
    AMOV_IMU_DEVICE_PATH_MAG2,
    AMOV_IMU_DEVICE_PATH_MAG3
};
static char    uart_name[32], uart_alias[32];
}

AMOV_IMU::AMOV_IMU(device::Device *interface, const char *path_accel,
                   const char *path_gyro, const char *path_mag,
                   enum Rotation rotation,
         int device_type, ORB_PRIO priority) :
    CDev("AMOV_IMU", path_accel),
    _interface(interface),
    _device_type(device_type),
    _gyro(new AMOV_IMU_gyro(this, path_gyro)),
    _mag(new AMOV_IMU_mag(this, path_mag)),
    _call {},
    _call_interval(0),
    _accel_reports(nullptr),
    _accel_scale{},
    _accel_topic(nullptr),
    _accel_orb_class_instance(-1),
    _accel_class_instance(-1),
    _gyro_reports(nullptr),
    _gyro_scale{},
    _mag_reports(nullptr),
    _mag_scale{},
    _sample_rate(1000),
    _accel_reads(perf_alloc(PC_COUNT, "amov_imu_acc_read")),
    _gyro_reads(perf_alloc(PC_COUNT, "amov_imu_gyro_read")),
    _mag_reads(perf_alloc(PC_COUNT, "amov_imu_mag_read")),
    _sample_perf(perf_alloc(PC_ELAPSED, "amov_imu_read")),
    _bad_transfers(perf_alloc(PC_COUNT, "amov_imu_bad_trans")),
    _bad_registers(perf_alloc(PC_COUNT, "amov_imu_bad_reg")),
    _good_transfers(perf_alloc(PC_COUNT, "amov_imu_good_trans")),
    _reset_retries(perf_alloc(PC_COUNT, "amov_imu_reset")),
    _duplicates(perf_alloc(PC_COUNT, "amov_imu_duplicates")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
    _register_wait(0),
    _reset_wait(0),
    _accel_filter_x(AMOV_IMU_ACCEL_DEFAULT_RATE, AMOV_IMU_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_y(AMOV_IMU_ACCEL_DEFAULT_RATE, AMOV_IMU_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_z(AMOV_IMU_ACCEL_DEFAULT_RATE, AMOV_IMU_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_x(AMOV_IMU_GYRO_DEFAULT_RATE, AMOV_IMU_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_y(AMOV_IMU_GYRO_DEFAULT_RATE, AMOV_IMU_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_z(AMOV_IMU_GYRO_DEFAULT_RATE, AMOV_IMU_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_int(1000000 / AMOV_IMU_ACCEL_MAX_OUTPUT_RATE),
    _gyro_int(1000000 / AMOV_IMU_GYRO_MAX_OUTPUT_RATE, true),
    _good_trans(0),
    _bad_trans(0),
    _rotation(rotation),
    _last_temperature(0),
    _last_accel{},
    _got_duplicate(false),
    _task(-1),
    _priority(priority),
    _imu_buf_length(32)
{
    // disable debug() calls
    _debug_enabled = false;

    // set the device type from the interface
    _device_id.devid_s.bus_type = _interface->get_device_bus_type();
    _device_id.devid_s.bus = _interface->get_device_bus();
    _device_id.devid_s.address = _interface->get_device_address();

    _device_id.devid_s.devtype = DRV_ACC_DEVTYPE_AMOV_IMU;
    /* Prime _gyro with parents devid. */
    _gyro->_device_id.devid = _device_id.devid;
    _gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_AMOV_IMU;

    /* Prime _gyro with parents devid. */
    _mag->_device_id.devid = _device_id.devid;
    _mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_AMOV_IMU;

    // copy device type to interface
    _interface->set_device_type(_device_id.devid_s.devtype);

    // default accel scale factors
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    // default gyro scale factors
    _gyro_scale.x_offset = 0;
    _gyro_scale.x_scale  = 1.0f;
    _gyro_scale.y_offset = 0;
    _gyro_scale.y_scale  = 1.0f;
    _gyro_scale.z_offset = 0;
    _gyro_scale.z_scale  = 1.0f;

    // default mag scale factors
    _mag_scale.x_offset = 0;
    _mag_scale.x_scale  = 1.0f;
    _mag_scale.y_offset = 0;
    _mag_scale.y_scale  = 1.0f;
    _mag_scale.z_offset = 0;
    _mag_scale.z_scale  = 1.0f;

    memset(&_call, 0, sizeof(_call));

    g_imu = this;
}

AMOV_IMU::~AMOV_IMU()
{
    /* make sure we are truly inactive */
    stop();

    /* delete the gyro subdriver */
    delete _gyro;

    delete _mag;

    /* free any existing reports */
    if (_accel_reports != nullptr) {
        delete _accel_reports;
    }

    if (_gyro_reports != nullptr) {
        delete _gyro_reports;
    }

    if (_mag_reports != nullptr) {
        delete _mag_reports;
    }

    if (_accel_class_instance != -1) {
        unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
    }

    /* delete the perf counter */
    perf_free(_sample_perf);
    perf_free(_accel_reads);
    perf_free(_gyro_reads);
    perf_free(_mag_reads);
    perf_free(_bad_transfers);
    perf_free(_bad_registers);
    perf_free(_good_transfers);
    perf_free(_reset_retries);
    perf_free(_duplicates);
}

int
AMOV_IMU::init()
{
    /* do init */

    int ret = CDev::init();

    /* if init failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("CDev init failed");
        return ret;
    }

    ret = -ENOMEM;
    /* allocate basic report buffers */
    _accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

    if (_accel_reports == nullptr) {
        goto out;
    }

    _gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

    if (_gyro_reports == nullptr) {
        goto out;
    }

    _mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

    if (_mag_reports == nullptr) {
        goto out;
    }

    ret = -EIO;

    if (reset() != OK) {
        goto out;
    }

    /* Initialize offsets and scales */
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    _gyro_scale.x_offset = 0;
    _gyro_scale.x_scale  = 1.0f;
    _gyro_scale.y_offset = 0;
    _gyro_scale.y_scale  = 1.0f;
    _gyro_scale.z_offset = 0;
    _gyro_scale.z_scale  = 1.0f;

    _mag_scale.x_offset = 0;
    _mag_scale.x_scale  = 1.0f;
    _mag_scale.y_offset = 0;
    _mag_scale.y_scale  = 1.0f;
    _mag_scale.z_offset = 0;
    _mag_scale.z_scale  = 1.0f;


    /* do CDev init for the gyro device node, keep it optional */
    ret = _gyro->init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("gyro init failed");
        return ret;
    }

    /* do CDev init for the mag device node, keep it optional */
    ret = _mag->init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("mag init failed");
        return ret;
    }

    _accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

    measure();

    /* advertise sensor topic, measure manually to initialize valid report */
    struct accel_report arp;
    _accel_reports->get(&arp);

    /* measurement will have generated a report, publish */
    _accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
                       &_accel_orb_class_instance, _priority);

    if (_accel_topic == nullptr) {
        warnx("ADVERT FAIL");
    }


    /* advertise sensor topic, measure manually to initialize valid report */
    struct gyro_report grp;
    _gyro_reports->get(&grp);

    _gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
                 &_gyro->_gyro_orb_class_instance, _priority);

    if (_gyro->_gyro_topic == nullptr) {
        warnx("ADVERT FAIL");
    }

out:
    return ret;
}

int AMOV_IMU::reset()
{
    return OK;
}

ssize_t
AMOV_IMU::read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(accel_report);

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _accel_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_accel_reports->empty()) {
        return -EAGAIN;
    }

    perf_count(_accel_reads);

    /* copy reports out of our buffer to the caller */
    accel_report *arp = reinterpret_cast<accel_report *>(buffer);
    int transferred = 0;

    while (count--) {
        if (!_accel_reports->get(arp)) {
            break;
        }

        transferred++;
        arp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(accel_report));
}

ssize_t
AMOV_IMU::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(gyro_report);

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _gyro_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_gyro_reports->empty()) {
        return -EAGAIN;
    }

    perf_count(_gyro_reads);

    /* copy reports out of our buffer to the caller */
    gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
    int transferred = 0;

    while (count--) {
        if (!_gyro_reports->get(grp)) {
            break;
        }

        transferred++;
        grp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(gyro_report));
}


ssize_t
AMOV_IMU::mag_read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(mag_report);

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _mag_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_mag_reports->empty()) {
        return -EAGAIN;
    }

    perf_count(_mag_reads);

    /* copy reports out of our buffer to the caller */
    mag_report *mrp = reinterpret_cast<mag_report *>(buffer);
    int transferred = 0;

    while (count--) {
        if (!_mag_reports->get(mrp)) {
            break;
        }

        transferred++;
        mrp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(mag_report));
}

int
AMOV_IMU::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    unsigned dummy = arg;

    switch (cmd) {

    case SENSORIOCRESET:
        return reset();

    case SENSORIOCSPOLLRATE: {
            switch (arg) {

            /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL:
                stop();
                _call_interval = 0;
                return OK;

            /* external signalling not supported */
            case SENSOR_POLLRATE_EXTERNAL:

            /* zero would be bad */
            case 0:
                return -EINVAL;

            /* set default/max polling rate */
            case SENSOR_POLLRATE_MAX:
                return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

            case SENSOR_POLLRATE_DEFAULT:
                return ioctl(filp, SENSORIOCSPOLLRATE, AMOV_IMU_ACCEL_DEFAULT_RATE);

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    bool want_start = (_call_interval == 0);

                    /* convert hz to hrt interval via microseconds */
                    unsigned ticks = 1000000 / arg;

                    /* check against maximum sane rate */
                    if (ticks < 1000) {
                        return -EINVAL;
                    }

                    // adjust filters
                    float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
                    float sample_rate = 1.0e6f / ticks;

                    _accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
                    _accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
                    _accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


                    float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
                    _gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
                    _gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
                    _gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

                    /* update interval for next measurement */
                    /* XXX this is a bit shady, but no other way to adjust... */
                    _call_interval = ticks;

                    /*
                      set call interval faster then the sample time. We
                      then detect when we have duplicate samples and reject
                      them. This prevents aliasing due to a beat between the
                      stm32 clock and the AMOV IMU clock
                     */

                    _call.period = _call_interval - AMOV_IMU_TIMER_REDUCTION;

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }
            }
        }

    case SENSORIOCGPOLLRATE:
        if (_call_interval == 0) {
            return SENSOR_POLLRATE_MANUAL;
        }

        return 1000000 / _call_interval;

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = px4_enter_critical_section();

            if (!_accel_reports->resize(arg)) {
                px4_leave_critical_section(flags);
                return -ENOMEM;
            }

            px4_leave_critical_section(flags);

            return OK;
        }

    case ACCELIOCGSAMPLERATE:
        return _sample_rate;

    case ACCELIOCSSAMPLERATE:
        return OK;

    case ACCELIOCSSCALE: {
            /* copy scale, but only if off by a few percent */
            struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
            float sum = s->x_scale + s->y_scale + s->z_scale;

            if (sum > 2.0f && sum < 4.0f) {
                memcpy(&_accel_scale, s, sizeof(_accel_scale));
                return OK;

            } else {
                return -EINVAL;
            }
        }

    case ACCELIOCGSCALE:
        /* copy scale out */
        memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
        return OK;

        case ACCELIOCSRANGE:
                return OK;

        case ACCELIOCSELFTEST:
        return OK;

    case ACCELIOCGEXTERNAL:
        return _interface->ioctl(cmd, dummy);

    default:
        /* give it to the superclass */
        return CDev::ioctl(filp, cmd, arg);
    }
}

int
AMOV_IMU::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{

    switch (cmd) {

    /* these are shared with the accel side */
    case SENSORIOCSPOLLRATE:
    case SENSORIOCGPOLLRATE:
    case SENSORIOCRESET:
        return ioctl(filp, cmd, arg);

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = px4_enter_critical_section();

            if (!_gyro_reports->resize(arg)) {
                px4_leave_critical_section(flags);
                return -ENOMEM;
            }

            px4_leave_critical_section(flags);

            return OK;
        }

    case GYROIOCGSAMPLERATE:
        return _sample_rate;

    case GYROIOCSSAMPLERATE:
        return OK;

    case GYROIOCSSCALE:
        /* copy scale in */
        memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
        return OK;

    case GYROIOCGSCALE:
        /* copy scale out */
        memcpy((struct gyro_calibration_s *) arg, &_gyro_scale, sizeof(_gyro_scale));
        return OK;

    case GYROIOCSRANGE:
        /* XXX not implemented */
        // XXX change these two values on set:
        // _gyro_range_scale = xx
        // _gyro_range_rad_s = xx
        return -EINVAL;

    case GYROIOCSELFTEST:
        return OK;

    default:
        /* give it to the superclass */
        return CDev::ioctl(filp, cmd, arg);
    }
}

int
AMOV_IMU::mag_ioctl(struct file *filp, int cmd, unsigned long arg)
{

    switch (cmd) {

    /* these are shared with the accel side */
    case SENSORIOCSPOLLRATE:
    case SENSORIOCGPOLLRATE:
    case SENSORIOCRESET:
        return ioctl(filp, cmd, arg);

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = px4_enter_critical_section();

            if (!_mag_reports->resize(arg)) {
                px4_leave_critical_section(flags);
                return -ENOMEM;
            }

            px4_leave_critical_section(flags);

            return OK;
        }

    case MAGIOCGSAMPLERATE:
        return _sample_rate;

    case MAGIOCSSAMPLERATE:
        return OK;

    case MAGIOCSSCALE:
        /* copy scale in */
        memcpy(&_mag_scale, (struct mag_calibration_s *) arg, sizeof(_mag_scale));
        return OK;

    case MAGIOCGSCALE:
        /* copy scale out */
        memcpy((struct mag_calibration_s *) arg, &_mag_scale, sizeof(_mag_scale));
        return OK;

    case MAGIOCSRANGE:
        /* XXX not implemented */
        // XXX change these two values on set:
        // _gyro_range_scale = xx
        // _gyro_range_rad_s = xx
        return -EINVAL;

    case MAGIOCSELFTEST:
        return OK;

    default:
        /* give it to the superclass */
        return CDev::ioctl(filp, cmd, arg);
    }
}


void
AMOV_IMU::status()
{
    uint64_t tmp = _bad_trans + _good_trans;
    warnx("Received: %15.0f", (double) tmp);
    warnx("Success Rate: %5.8f %%", (double)(((float) _good_trans) / ((float)tmp) * 100.0f));
}


void
AMOV_IMU::start()
{
    /* make sure we are stopped first */
    uint32_t last_call_interval = _call_interval;
    stop();
    _call_interval = last_call_interval;

    /* discard any stale data in the buffers */
    _accel_reports->flush();
    _gyro_reports->flush();
    _mag_reports->flush();


    if(_device_type == AMOV_IMU_DEVICE_TYPE_SPI)
    {
        /* start polling at the specified rate */
        hrt_call_every(&_call,
               1000,
               _call_interval - AMOV_IMU_TIMER_REDUCTION,
               (hrt_callout)&AMOV_IMU::measure_trampoline, this);
    }
    else if(_device_type == AMOV_IMU_DEVICE_TYPE_UART)
    {
        unsigned dummy;
        _interface->ioctl(99999999, dummy);
        _imu_buf_length = dummy;
        warnx("AMOV IMU buf length = %d", _imu_buf_length);

        if(_task >= 0)
            return;
        int prio;
        char task_name[32];
        switch(_priority)
        {
            case ORB_PRIO_MAX:
                prio = SCHED_PRIORITY_MAX;
                strcpy(task_name, "amov-task-realtime");
                break;
            case ORB_PRIO_VERY_HIGH:
                prio = SCHED_PRIORITY_MAX - 40;
                strcpy(task_name, "amov-task-very-high");
                break;
            case ORB_PRIO_HIGH:
                prio = SCHED_PRIORITY_MAX - 80;
                strcpy(task_name, "amov-task-high");
                break;
            case ORB_PRIO_DEFAULT:
                prio = SCHED_PRIORITY_MAX - 120;
                strcpy(task_name, "amov-task-default");
                break;
            case ORB_PRIO_LOW:
                prio = SCHED_PRIORITY_MAX - 160;
                strcpy(task_name, "amov-task-low");
                break;
            case ORB_PRIO_VERY_LOW:
                prio = SCHED_PRIORITY_MAX - 200;
                strcpy(task_name, "amov-task-very-low");
                break;
            case ORB_PRIO_MIN:
                prio = SCHED_PRIORITY_MAX - 240;
                strcpy(task_name, "amov-task-min");
                break;
            default:
                prio = SCHED_PRIORITY_MAX;
                strcpy(task_name, "amov-task-realtime");
                break;
        }
        /* start the main task */
        _task = px4_task_spawn_cmd(task_name,
               SCHED_DEFAULT,
               prio,
               1800,
               (main_t)&AMOV_IMU::task_main_trampoline,
               nullptr);

        if (_task < 0) {
            DEVICE_DEBUG("AMOV_IMU task start failed: %d", errno);
            return;
        }
    }
}

void
AMOV_IMU::stop()
{
    if(_device_type == AMOV_IMU_DEVICE_TYPE_SPI)
    {
        hrt_cancel(&_call);
    }

    /* reset internal states */
    memset(_last_accel, 0, sizeof(_last_accel));

    /* discard unread data in the buffers */
    if (_accel_reports != nullptr) {
        _accel_reports->flush();
    }

    if (_gyro_reports != nullptr) {
        _gyro_reports->flush();
    }

    if (_mag_reports != nullptr) {
        _mag_reports->flush();
    }
}

void
AMOV_IMU::task_main_trampoline(int argc, char *argv[])
{
    g_imu->task_main();
}

void
AMOV_IMU::task_main()
{
    usleep(1000 * 1000 * 5);

    while(true)
    {
        measure();
    }
}

void
AMOV_IMU::measure_trampoline(void *arg)
{
    AMOV_IMU *dev = reinterpret_cast<AMOV_IMU *>(arg);

    /* make another measurement */
    dev->measure();
}

int
AMOV_IMU::measure()
{
    if (hrt_absolute_time() < _reset_wait) {
        // we're waiting for a reset to complete
        return OK;
    }

    struct AMOVReport amov_report;

    /* start measuring */
    perf_begin(_sample_perf);

    bool found = false;
    int index;
    for(index = 0; index < MAX_DEV_NUM; ++index)
    {
        if(amov_imu::dev[index] == this)
        {
            found = true;
            break;
        }
    }
    if(!found)
        return -EIO;

    /*
     * Fetch the full set of measurements from the AMOV IMU in one pass.
     */
    if(_device_type == AMOV_IMU_DEVICE_TYPE_SPI)
    {
        if(38 != _interface->read(AMOV_IMU_SET_SPEED(AMOV_IMU_REG_DATA, AMOV_IMU_LOW_BUS_SPEED),
                                    &_io_buffer_storage[index],
                                    38))
            return -EIO;
    }
    else if(_device_type == AMOV_IMU_DEVICE_TYPE_UART)
    {
        if(255 != _interface->read(AMOV_IMU_SET_SPEED(AMOV_IMU_REG_DATA, AMOV_IMU_LOW_BUS_SPEED),
                                    &_io_buffer_storage[index],
                                    255))
            return -EIO;
    }
    else
        return -EINVAL;
    // Here, we set a 'found' flag to check the head of the data.
    // For UART, it is natural because the communication is asynchronous
    // For onboard SPI sensors with short wiring, it is able to read the data simutaneously.
    // However, as the wire could be very long, there is inevitable delay during the transmission.
    // Also, the master board is constituted by an STM32F4 MCU so it can not handle parallel SPI commands.
    // So it is a little bit different with common SPI reading process.

    uint8_t *_trans_buf = _io_buffer_storage[index].buf;
    bool success = false;

    if(_device_type == AMOV_IMU_DEVICE_TYPE_SPI)
    {
        for(int i = 0; i < 8; ++i)
        {
            uint32_t crc = calculate_CRC32(_trans_buf + i + 1, _imu_buf_length - 6);
            if(!memcmp(&crc, _trans_buf + i + _imu_buf_length - 5, 4))
            {
                memcpy(&amov_report, _trans_buf + i + 1, _imu_buf_length - 6);
                success = true;
                break;
            }
        }
    }
    else
    {
        uint32_t crc = calculate_CRC32(_trans_buf + 1, _imu_buf_length - 6);
        if(!memcmp(&crc, _trans_buf + _imu_buf_length - 5, 4))
        {
            memcpy(&amov_report, _trans_buf + 1, _imu_buf_length - 6);
            success = true;
        }
    }

    if(!success)
    {
        ++ _bad_trans;
        perf_count(_bad_transfers);
        perf_end(_sample_perf);
        return OK;
    }


    /*
       see if this is duplicate accelerometer data. Note that we
       can't use the data ready interrupt status bit in the status
       register as that also goes high on new gyro data, and when
       we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
       sampled at 8kHz, so we would incorrectly think we have new
       data when we are in fact getting duplicate accelerometer data.
    */
    if (!_got_duplicate && memcmp(&amov_report.accel_x.bytes[0], &_last_accel[0], 4) == 0) {
        // it isn't new data - wait for next timer
                ++_bad_trans;
        perf_end(_sample_perf);
        perf_count(_duplicates);
        _got_duplicate = true;
        return OK;
    }

    memcpy(&_last_accel[0], &amov_report.accel_x.bytes[0], 4);
    _got_duplicate = false;

    ++ _good_trans;
    perf_count(_good_transfers);

    /*
     * Report buffers.
     */
    accel_report        arb;
    gyro_report         grb;
    mag_report          mrb;

    /*
     * Adjust and scale results to m/s^2.
     */
    grb.timestamp = arb.timestamp = mrb.timestamp = hrt_absolute_time();
    grb.error_count = arb.error_count = mrb.error_count = 0;

    /*
     * 1) Scale raw value to SI units using scaling from datasheet.
     * 2) Subtract static offset (in SI units)
     * 3) Scale the statically calibrated values with a linear
     *    dynamically obtained factor
     *
     * Note: the static sensor offset is the number the sensor outputs
     * 	 at a nominally 'zero' input. Therefore the offset has to
     * 	 be subtracted.
     *
     *	 Example: A gyro outputs a value of 74 at zero angular rate
     *	 	  the offset is 74 from the origin and subtracting
     *		  74 from all measurements centers them around zero.
     */


    /* NOTE: Axes have been swapped to match the board a few lines above. */

    arb.x_raw = amov_report.accel_x.f * 8192.0f;
    arb.y_raw = amov_report.accel_y.f * 8192.0f;
    arb.z_raw = amov_report.accel_z.f * 8192.0f;

    float xraw_f = amov_report.accel_x.f;
    float yraw_f = amov_report.accel_y.f;
    float zraw_f = amov_report.accel_z.f;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    float x_in_new = (xraw_f - _accel_scale.x_offset) * _accel_scale.x_scale;
    float y_in_new = (yraw_f - _accel_scale.y_offset) * _accel_scale.y_scale;
    float z_in_new = (zraw_f - _accel_scale.z_offset) * _accel_scale.z_scale;

    arb.x = _accel_filter_x.apply(x_in_new);
    arb.y = _accel_filter_y.apply(y_in_new);
    arb.z = _accel_filter_z.apply(z_in_new);

    matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
    matrix::Vector3f aval_integrated;

    bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
    arb.x_integral = aval_integrated(0);
    arb.y_integral = aval_integrated(1);
    arb.z_integral = aval_integrated(2);

    _last_temperature = (amov_report.temp) / 361.0f + 35.0f;

    arb.temperature_raw = amov_report.temp;
    arb.temperature = _last_temperature;

    /* return device ID */
    arb.device_id = _device_id.devid;

    grb.x_raw = amov_report.gyro_x.f * 16.4f;
    grb.y_raw = amov_report.gyro_y.f * 16.4f;
    grb.z_raw = amov_report.gyro_z.f * 16.4f;

    xraw_f = amov_report.gyro_x.f;
    yraw_f = amov_report.gyro_y.f;
    zraw_f = amov_report.gyro_z.f;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    float x_gyro_in_new = (xraw_f - _gyro_scale.x_offset) * _gyro_scale.x_scale;
    float y_gyro_in_new = (yraw_f - _gyro_scale.y_offset) * _gyro_scale.y_scale;
    float z_gyro_in_new = (zraw_f - _gyro_scale.z_offset) * _gyro_scale.z_scale;

    grb.x = _gyro_filter_x.apply(x_gyro_in_new);
    grb.y = _gyro_filter_y.apply(y_gyro_in_new);
    grb.z = _gyro_filter_z.apply(z_gyro_in_new);

    matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
    matrix::Vector3f gval_integrated;

    bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
    grb.x_integral = gval_integrated(0);
    grb.y_integral = gval_integrated(1);
    grb.z_integral = gval_integrated(2);

    grb.temperature_raw = amov_report.temp;
    grb.temperature = _last_temperature;

    /* return device ID */
    grb.device_id = _gyro->_device_id.devid;


    mrb.x_raw = amov_report.mag_x.f * 1000.0f;
    mrb.y_raw = amov_report.mag_y.f * 1000.0f;
    mrb.z_raw = amov_report.mag_z.f * 1000.0f;

    xraw_f = amov_report.mag_x.f;
    yraw_f = amov_report.mag_y.f;
    zraw_f = amov_report.mag_z.f;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    float x_mag_in_new = (xraw_f - _mag_scale.x_offset) * _mag_scale.x_scale;
    float y_mag_in_new = (yraw_f - _mag_scale.y_offset) * _mag_scale.y_scale;
    float z_mag_in_new = (zraw_f - _mag_scale.z_offset) * _mag_scale.z_scale;

    mrb.x = x_mag_in_new;
    mrb.y = y_mag_in_new;
    mrb.z = z_mag_in_new;
    mrb.temperature = _last_temperature;

    /* return device ID */
    mrb.device_id = _mag->_device_id.devid;

    _accel_reports->force(&arb);
    _gyro_reports->force(&grb);
    _mag_reports->force(&mrb);

    bool mag_notify = true;

    if(isnan(arb.x) || isnan(arb.y) || isnan(arb.z) ||
       isnan(grb.x) || isnan(grb.y) || isnan(grb.z) ||
       isnan(mrb.x) || isnan(mrb.y) || isnan(mrb.z) ||
       !((arb.x * arb.x + arb.y * arb.y + arb.z * arb.z < 721770.0f) && // Maximum 50G
         (grb.x * grb.x + grb.y * grb.y + grb.z * grb.z < 3654.0f))) // Maximum 2000dps
    {
        accel_notify = false;
        gyro_notify = false;
        mag_notify = false;
    }

    if(!(_imu_buf_length == AMOV_IMU_MODEL_IMU2_LEN ||
         _imu_buf_length == AMOV_IMU_MODEL_IMU3_LEN ||
         _imu_buf_length == AMOV_IMU_MODEL_AHRS2_LEN ||
         _imu_buf_length == AMOV_IMU_MODEL_AHRS3_LEN ||
         _imu_buf_length == AMOV_IMU_MODEL_INSGPS1_LEN ||
         _imu_buf_length == AMOV_IMU_MODEL_INSGPS2_LEN))
        mag_notify = false;

    /* notify anyone waiting for data */
    if (accel_notify) {
        poll_notify(POLLIN);
    }

    if (gyro_notify) {
        _gyro->parent_poll_notify();
    }

    if (mag_notify) {
        _mag->parent_poll_notify();
    }

    if (accel_notify && !(_pub_blocked)) {
        /* log the time of this report */
        perf_begin(_controller_latency_perf);
        /* publish it */
        orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
    }

    if (gyro_notify && !(_pub_blocked)) {
        /* publish it */
        orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
    }

    if (mag_notify && !(_pub_blocked)) {
        if(_mag->_mag_topic == nullptr)
        {
            _mag->_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrb,
                     &_mag->_mag_orb_class_instance, _priority);

            if (_mag->_mag_topic == nullptr) {
                warnx("ADVERT FAIL");
            }
        }

        if(_mag->_mag_topic != nullptr)
        {
            /* publish it */
            orb_publish(ORB_ID(sensor_mag), _mag->_mag_topic, &mrb);
        }
    }

    /* stop measuring */
    perf_end(_sample_perf);
    return OK;
}


AMOV_IMU_gyro::AMOV_IMU_gyro(AMOV_IMU *parent, const char *path) :
    CDev("AMOV_IMU_gyro", path),
    _parent(parent),
    _gyro_topic(nullptr),
    _gyro_orb_class_instance(-1),
    _gyro_class_instance(-1)
{
}

AMOV_IMU_gyro::~AMOV_IMU_gyro()
{
    if (_gyro_class_instance != -1) {
        unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
    }
}

int
AMOV_IMU_gyro::init()
{
    int ret;

    // do base class init
    ret = CDev::init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("gyro init failed");
        return ret;
    }

    _gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

    return ret;
}

void
AMOV_IMU_gyro::parent_poll_notify()
{
    poll_notify(POLLIN);
}

ssize_t
AMOV_IMU_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
    return _parent->gyro_read(filp, buffer, buflen);
}

int
AMOV_IMU_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {
    case DEVIOCGDEVICEID:
        return (int)CDev::ioctl(filp, cmd, arg);
        break;

    default:
        return _parent->gyro_ioctl(filp, cmd, arg);
    }
}



AMOV_IMU_mag::AMOV_IMU_mag(AMOV_IMU *parent, const char *path) :
    CDev("AMOV_IMU_mag", path),
    _parent(parent),
    _mag_topic(nullptr),
    _mag_orb_class_instance(-1),
    _mag_class_instance(-1)
{
}

AMOV_IMU_mag::~AMOV_IMU_mag()
{
    if (_mag_class_instance != -1) {
        unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
    }
}

int
AMOV_IMU_mag::init()
{
    int ret;

    // do base class init
    ret = CDev::init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("mag init failed");
        return ret;
    }

    _mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

    return ret;
}

void
AMOV_IMU_mag::parent_poll_notify()
{
    poll_notify(POLLIN);
}

ssize_t
AMOV_IMU_mag::read(struct file *filp, char *buffer, size_t buflen)
{
    return _parent->mag_read(filp, buffer, buflen);
}

int
AMOV_IMU_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {
    case DEVIOCGDEVICEID:
        return (int)CDev::ioctl(filp, cmd, arg);
        break;

    default:
        return _parent->mag_ioctl(filp, cmd, arg);
    }
}

/**
 * Local functions in support of the shell command.
 */
namespace amov_imu
{
bool start_bus(enum Rotation rotation, int device_type, bool is_dma, ORB_PRIO priority);
void stop(enum AMOV_IMU_BUS busid);
void reset(int index);
void status(AMOV_IMU * _dev);
void usage();

bool
start_bus(enum Rotation rotation, int device_type, bool is_dma, ORB_PRIO priority)
{
    int fd = -1;

    device::Device *inter = nullptr;
    bool found = false;
    int i;
    for(i = 0; i < MAX_DEV_NUM; ++i)
        if(interface[i] == nullptr)
        {
            inter = interface[i];
            found = true;
            break;
        }

    if (!found) {
            warnx("Devices already started. Reached maximum device number!");
            return false;
    }

    if(is_spi)
    {
#ifdef PX4_SPI_BUS_EXT
        inter = AMOV_IMU_SPI_interface(PX4_SPI_BUS_EXT, device_type, true);
#endif
#ifdef PX4_SPIDEV_EXT_AMOV_IMU
        inter = AMOV_IMU_SPI_interface(PX4_SPIDEV_EXT_AMOV_IMU, device_type, true);
#endif
    }
    if(is_uart)
    {
        warnx("AMOV IMU on UART!");
        inter = AMOV_IMU_UART_interface(uart_name, uart_alias, is_dma);
    }

    if (inter == nullptr) {
            if(is_spi)
                warnx("MEM: no device on SPI bus");
            else if(is_uart)
                warnx("MEM: no device on UART bus");
            return false;
    }

    if (inter->init() != OK) {
            delete inter;
            inter = nullptr;
            if(is_spi)
                warnx("Init error: no device on SPI bus");
            else if(is_uart)
                warnx("Init error: no device on UART bus");
            return false;
    }

    interface[i] = inter;


    AMOV_IMU *_dev = nullptr;
    found = false;
    for(i = 0; i < MAX_DEV_NUM; ++i)
        if(dev[i] == nullptr)
        {
            _dev = dev[i];
            found = true;
            break;
        }

    if (!found) {
            warnx("Devices already started. Reached maximum device number!");
            return false;
    }
    _dev = new AMOV_IMU(inter, accelpath[i], gyropath[i], magpath[i], rotation, device_type, priority);

    if (_dev == nullptr) {
        delete inter;
        return false;
    }

    if (OK != _dev->init()) {
        goto fail;
    }

    /* set the poll rate to default, starts automatic data collection */

    fd = open(accelpath[i], O_RDONLY);

    if (fd < 0) {
        goto fail;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        goto fail;
    }

    close(fd);

    dev[i] = _dev;

    return true;

fail:

    if (fd >= 0) {
        close(fd);
    }

    if (_dev != nullptr) {
        delete _dev;
        _dev = nullptr;
    }

    return false;
}

void
stop(enum AMOV_IMU_BUS busid)
{
    device::Device *inter = nullptr;
    int i;
    for(i = 0; i < MAX_DEV_NUM; ++i)
        if(interface[i] == nullptr)
        {
            inter = interface[i];
            break;
        }

    if (inter == nullptr) {
            warnx("Devices not started. Reached maximum device number!");
            return;
    }

    if (inter != nullptr) {
        delete inter;
        inter = nullptr;
                interface[i] = nullptr;

    } else {
        /* warn, but not an error */
        warnx("already stopped.");
    }

    exit(0);
}

void
reset(int index)
{
    int fd = open(accelpath[index], O_RDONLY);

    if (fd < 0) {
        err(1, "failed ");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
        err(1, "driver reset failed");
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "driver poll restart failed");
    }

    close(fd);

    exit(0);
}

void status(AMOV_IMU * _dev)
{
    if (_dev == nullptr) {
            warnx("Devices not started.");
            return;
    }
    else
    {
        _dev->status();
    }
}

void
usage()
{
    warnx("missing command: try 'start', 'stop',");
    warnx("'reset1', 'reset2', 'reset3',");
    warnx("'status1', 'status2', 'status3',");
    warnx("options:");
    warnx("    -S SPI bus");
    warnx("    -X UART bus name (1: USART1 4: UART4)");
    warnx("    -I UART alias 0: /dev/ttyS0");
    warnx("    -A DMA sets true");
    warnx("    -P Priority (ORB_PRIO_MIN = 1");
    warnx("                 ORB_PRIO_VERY_LOW = 25,");
    warnx("                 ORB_PRIO_LOW = 50,");
    warnx("                 ORB_PRIO_DEFAULT = 75,");
    warnx("                 ORB_PRIO_HIGH = 100,");
    warnx("                 ORB_PRIO_VERY_HIGH = 125,");
    warnx("                 ORB_PRIO_MAX = 255)");
    warnx("    -R rotation");
}

}

int
amov_imu_main(int argc, char *argv[])
{
    enum AMOV_IMU_BUS busid = AMOV_IMU_BUS_ALL;
    int device_type = AMOV_IMU_DEVICE_TYPE_SPI;
    int ch;
    bool is_dma = false;
    enum Rotation rotation = ROTATION_NONE;
    ORB_PRIO priority = ORB_PRIO_MAX;

    int myoptind = 1;
    const char *myoptarg = NULL;
    while ((ch = px4_getopt(argc, argv, "SX:I:AP:R:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {

        case 'S':
            busid = AMOV_IMU_BUS_SPI;
            device_type = AMOV_IMU_DEVICE_TYPE_SPI;
            amov_imu::is_spi = true;
            break;

        case 'X': {
            busid = AMOV_IMU_BUS_UART;
            device_type = AMOV_IMU_DEVICE_TYPE_UART;
            amov_imu::is_uart = true;
            int tmp = atoi(myoptarg);
            if(tmp == 1)
                strcpy(amov_imu::uart_name, "USART1");
            else if(tmp == 2)
                strcpy(amov_imu::uart_name, "USART2");
            else if(tmp == 3)
                strcpy(amov_imu::uart_name, "USART3");
            else if(tmp == 4)
                strcpy(amov_imu::uart_name, "UART4");
            else if(tmp == 5)
                strcpy(amov_imu::uart_name, "UART5");
            else if(tmp == 6)
                strcpy(amov_imu::uart_name, "USART6");
            else if(tmp == 7)
                strcpy(amov_imu::uart_name, "UART7");
            else if(tmp == 8)
                strcpy(amov_imu::uart_name, "UART8");
            break;
        }
        case 'I': {
            strcpy(amov_imu::uart_alias, "/dev/ttyS");
            strcat(amov_imu::uart_alias, myoptarg);
            break;
        }
        case 'A':
                is_dma = true;
                break;
        case 'P':
                priority = (ORB_PRIO) atoi(myoptarg);
                break;
        case 'R':
            rotation = (enum Rotation)atoi(myoptarg);
            break;

        default:
            amov_imu::usage();
            exit(0);
        }
    }

    const char *verb = argv[myoptind];

    /*
     * Start/load the driver.

     */
    if (!strcmp(verb, "start")) {
        amov_imu::start_bus(rotation, device_type, is_dma, priority);
    }

    if (!strcmp(verb, "stop")) {
        amov_imu::stop(busid);
    }

        if (!strcmp(verb, "status1")) {
        amov_imu::status(amov_imu::dev[0]);
    }

        if (!strcmp(verb, "status2")) {
        amov_imu::status(amov_imu::dev[1]);
    }

        if (!strcmp(verb, "status3")) {
        amov_imu::status(amov_imu::dev[2]);
    }


    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset1")) {
        amov_imu::reset(0);
    }
        if (!strcmp(verb, "reset2")) {
        amov_imu::reset(1);
    }
        if (!strcmp(verb, "reset3")) {
        amov_imu::reset(2);
    }
    exit(1);
}
