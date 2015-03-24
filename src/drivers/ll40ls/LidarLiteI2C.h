#pragma once

//Forward declaration
class RingBuffer;

#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <systemlib/perf_counter.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_range_finder.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

/* Configuration Constants */
#define LL40LS_BUS          PX4_I2C_BUS_EXPANSION
#define LL40LS_BASEADDR     0x62 /* 7-bit address */
#define LL40LS_BASEADDR_OLD     0x42 /* previous 7-bit address */
#define LL40LS_DEVICE_PATH_INT  "/dev/ll40ls_int"
#define LL40LS_DEVICE_PATH_EXT  "/dev/ll40ls_ext"

/* LL40LS Registers addresses */

#define LL40LS_MEASURE_REG      0x00        /* Measure range register */
#define LL40LS_MSRREG_RESET     0x00        /* reset to power on defaults */
#define LL40LS_MSRREG_ACQUIRE       0x04        /* Value to initiate a measurement, varies based on sensor revision */
#define LL40LS_MAX_ACQ_COUNT_REG       0x02     /* maximum acquisition count register */
#define LL40LS_DISTHIGH_REG     0x8F        /* High byte of distance register, auto increment */
#define LL40LS_WHO_AM_I_REG         0x11
#define LL40LS_WHO_AM_I_REG_VAL         0xCA
#define LL40LS_SIGNAL_STRENGTH_REG  0x5b

/* Device limits */
#define LL40LS_MIN_DISTANCE (0.00f)
#define LL40LS_MAX_DISTANCE (60.00f)

// normal conversion wait time
#define LL40LS_CONVERSION_INTERVAL 50*1000UL /* 50ms */

// maximum time to wait for a conversion to complete.
#define LL40LS_CONVERSION_TIMEOUT 100*1000UL /* 100ms */

class LidarLiteI2C : public device::I2C
{
public:
    LidarLiteI2C(int bus, const char *path, int address = LL40LS_BASEADDR);
    virtual ~LidarLiteI2C();

    virtual int         init();

    virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
    virtual int         ioctl(struct file *filp, int cmd, unsigned long arg);

    /**
    * Diagnostics - print some basic information about the driver.
    */
    void                print_info();

    /**
     * print registers to console
     */
    void                    print_registers();

protected:
    virtual int         probe();
    virtual int         read_reg(uint8_t reg, uint8_t &val);

private:
    float               _min_distance;
    float               _max_distance;
    work_s              _work;
    RingBuffer          *_reports;
    bool                _sensor_ok;
    unsigned            _measure_ticks;
    bool                _collect_phase;
    int                 _class_instance;

    orb_advert_t        _range_finder_topic;

    perf_counter_t      _sample_perf;
    perf_counter_t      _comms_errors;
    perf_counter_t      _buffer_overflows;
    perf_counter_t      _sensor_resets;
    perf_counter_t      _sensor_zero_resets;
    uint16_t        _last_distance;
    uint16_t        _zero_counter;
    uint64_t        _acquire_time_usec;
    volatile bool       _pause_measurements;

    /**< the bus the device is connected to */
    int         _bus;

    /**
    * Test whether the device supported by the driver is present at a
    * specific address.
    *
    * @param address    The I2C bus address to probe.
    * @return       True if the device is present.
    */
    int                 probe_address(uint8_t address);

    /**
    * Initialise the automatic measurement state machine and start it.
    *
    * @note This function is called at open and error time.  It might make sense
    *       to make it more aggressive about resetting the bus in case of errors.
    */
    void                start();

    /**
    * Stop the automatic measurement state machine.
    */
    void                stop();

    /**
    * Set the min and max distance thresholds if you want the end points of the sensors
    * range to be brought in at all, otherwise it will use the defaults LL40LS_MIN_DISTANCE
    * and LL40LS_MAX_DISTANCE
    */
    void                set_minimum_distance(float min);
    void                set_maximum_distance(float max);
    float               get_minimum_distance();
    float               get_maximum_distance();

    /**
    * Perform a poll cycle; collect from the previous measurement
    * and start a new one.
    */
    void                cycle();
    int                 measure();
    int                 collect();
    int                 reset_sensor();

    /**
    * Static trampoline from the workq context; because we don't have a
    * generic workq wrapper yet.
    *
    * @param arg        Instance pointer for the driver that is polling.
    */
    static void     cycle_trampoline(void *arg);
};
