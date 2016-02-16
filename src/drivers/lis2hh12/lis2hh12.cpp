

#include <px4_config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>


#include <board_config.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>
#include <drivers/device/integrator.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

//#include "lis2hh12.h"

#define DIR_READ                0x80
#define DIR_WRITE               0x00

#define LIS2HH12_DEVICE_PATH_ACCEL		"/dev/lis2hh12_accel"
#define LIS2HH12_DEVICE_PATH_ACCEL_EXT	"/dev/lis2hh12_accel_ext"


// LIS2HH12 registers
#define LISREG_WHO_AM_I			0x0F
#define LISREG_TEMP_L			0x0B
#define LISREG_TEMP_H			0x0C
#define LISREG_ACT_THS			0x1E
#define LISREG_ACT_DUR			0x1F
#define LISREG_CTRL_1			0x20
#define LISREG_CTRL_2			0x21
#define LISREG_CTRL_3			0x22
#define LISREG_CTRL_4			0x23
#define LISREG_CTRL_5			0x24
#define LISREG_CTRL_6			0x25
#define LISREG_CTRL_7			0x26
#define LISREG_STATUS			0x27
#define LISREG_OUT_X_L			0x28
#define LISREG_OUT_X_H			0x29
#define LISREG_OUT_Y_L			0x2A
#define LISREG_OUT_Y_H			0x2B
#define LISREG_OUT_Z_L			0x2C
#define LISREG_OUT_Z_H			0x2D
#define LISREG_FIFO_CTRL		0x2E
#define LISREG_FIFO_SRC			0x2F
#define LISREG_IG_CFG1			0x30
#define LISREG_IG_SRC1			0x31
#define LISREG_IG_THS_X1		0x32
#define LISREG_IG_THS_Y1		0x33
#define LISREG_IG_THS_Z1		0x34
#define LISREG_IG_DUR1			0x35
#define LISREG_IG_CFG2			0x36
#define LISREG_IG_SRC2			0x37
#define LISREG_IG_THS2			0x38
#define LISREG_IG_DUR2			0x39
#define LISREG_XL_REFER			0x3A
#define LISREG_XH_REFER			0x3B
#define LISREG_YL_REFER			0x3C
#define LISREG_YH_REFER			0x3D
#define LISREG_ZL_REFER			0x3E
#define LISREG_ZH_REFER			0x3F

#define LIS2HH12_ADDRESS		0x1E

//BIT configuration

#define LIS_WHO_AM_I            0x41

//CTRL1
#define LIS_HR                  (1<<7)
#define LIS_PWR_DWN             (0<<6) | (0<<5) | (0<<4)
#define LIS_ODR_10              (0<<6) | (0<<5) | (1<<4)
#define LIS_ODR_50              (0<<6) | (1<<5) | (0<<4)
#define LIS_ODR_100             (0<<6) | (1<<5) | (1<<4)
#define LIS_ODR_200             (1<<6) | (0<<5) | (0<<4)
#define LIS_ODR_400             (1<<6) | (0<<5) | (1<<4)
#define LIS_ODR_800             (1<<6) | (1<<5) | (0<<4)
#define LIS_BDU                 (1<<3)
#define LIS_ALL_AXES_EN         (1<<2) | (1<<1) | (1<<0)

//CTRL3
#define LIS_INT1_IG_EN          (1<<3)
#define LIS_INT1_DRDY           (1<<0)

//CTRL4
#define LIS_BW_400HZ            (0<<7) | (0<<6)
#define LIS_BW_200HZ            (0<<7) | (1<<6)
#define LIS_BW_100Hz            (1<<7) | (0<<6)
#define LIS_BW_50HZ             (1<<7) | (1<<6)
#define LIS_RANGE_2_G           (0<<5) | (0<<4)
#define LIS_RANGE_4_G           (1<<5) | (0<<4)
#define LIS_RANGE_8_G           (1<<5) | (1<<4)
#define LIS_BW_SCALE_ODR        (1<<3)
#define LIS_IF_ADD_INC          (1<<2)

//CTRL5
#define LIS_DEBUG               (1<<7)
#define LIS_SOFT_RESET          (1<<6)
#define LIS_DEC_NO              (0<<5) | (0<<4)
#define LIS_DEC_2               (0<<5) | (1<<4)
#define LIS_DEC_4               (1<<5) | (0<<4)
#define LIS_DEC_8               (1<<5) | (1<<4)
#define LIS_ST_NEG              (0<<3) | (1<<2)
#define LIS_ST_POS              (1<<3) | (0<<2)
#define LIS_H_LACTIVE           (1<<1)
#define LIS_PP_OD               (1<<0)

//CTRL6
#define LIS_BOOT                (1<<7)

//IG_SRC1
#define LIS_IA                  (1<<6)

//STATUS
#define LIS_STATUS_ZYXDA        (1<<3)

#define LIS2HH12_TIMER_REDUCTION				200

#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


#define LIS2HH12_ONE_G					9.80665f
#define LIS2HH12_ACCEL_DEFAULT_RANGE_G		8
#define LIS2HH12_ACCEL_DEFAULT_RATE			800
#define LIS2HH12_ACCEL_MAX_RATE 			800

#define LIS2HH12_DEFAULT_ONCHIP_FILTER_FREQ	50
#define LIS2HH12_DEFAULT_DRIVER_FILTER_FREQ	30

#define SENSOR_BOARD_ROTATION_DEFAULT   0

class LIS2HH12 : public device::I2C
{
public:
    LIS2HH12(int bus, const char *path, enum Rotation rotation, uint16_t address);
    virtual ~LIS2HH12();

    virtual int		init();

    virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
    virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

    /**
     * Diagnostics - print some basic information about the driver.
     */
    void			print_info();

    // print register dump
    void			print_registers();

    // trigger an error
    void			test_error();

/*protected:
    Device			*_interface;*/

protected:
    virtual int		probe();

private:

    struct work_s	_work;
    struct hrt_call		_call;
    unsigned		_call_interval;
    unsigned        _measure_ticks;

    ringbuffer::RingBuffer	*_reports;

    struct accel_scale	_accel_scale;
    float			_accel_range_scale;
    float			_accel_range_m_s2;
    orb_advert_t		_accel_topic;
    int			_orb_class_instance;
    int			_class_instance;

    unsigned		_current_rate;
    unsigned		_orientation;

    unsigned		_read;

    perf_counter_t		_sample_perf;
    perf_counter_t		_errors;
    perf_counter_t		_bad_registers;
    perf_counter_t		_duplicates;

    uint8_t			_register_wait;

    math::LowPassFilter2p	_accel_filter_x;
    math::LowPassFilter2p	_accel_filter_y;
    math::LowPassFilter2p	_accel_filter_z;

    Integrator		_accel_int;

    enum Rotation		_rotation;

    // this is used to support runtime checking of key
    // configuration registers to detect SPI bus errors and sensor
    // reset
#define LIS2HH12_NUM_CHECKED_REGISTERS 8
    static const uint8_t	_checked_registers[LIS2HH12_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_values[LIS2HH12_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_next;

    /**SENSOR_BOARD_ROTATION_DEFAULT
     * Start automatic measurement.
     */
    void			start();

    /**
     * Stop automatic measurement.
     */
    void			stop();

    /**
     * Reset the driver
     */
    void			reset();

    /**
     * disable I2C on the chip
     */
    //void			disable_i2c();

    /**
     * Get the internal / external state
     *
     * @return true if the sensor is not on the main MCU board
     */
    bool			is_external() { return (_bus == PX4_I2C_BUS_EXPANSION); }

    /**
     * Static trampoline from the hrt_call context; because we don't have a
     * generic hrt wrapper yet.
     *
     * Called by the HRT in interrupt context at the specified rate if
     * automatic polling is enabled.
     *
     * @param arg		Instance pointer for the driver that is polling.
     */
    //void		measure_trampoline_lis(void *arg);
    static void		measure_trampoline(void *arg);

    void cycle();

    /**
     * check key registers for correct values
     */
    void			check_registers(void);

    /**
     * Fetch measurements from the sensor and update the report ring.
     */
    void			measure();

    /**
     * Read a register from the LIS2HH12
     *
     * @param		The register to read.
     * @return		The value that was read.
     */
    uint8_t			read_reg(unsigned reg);

    /**
     * Write a register in the LIS2HH12
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void			write_reg(unsigned reg, uint8_t value);

    /**
     * Modify a register in the LIS2HH12
     *
     * Bits are cleared before bits are set.
     *
     * @param reg		The register to modify.
     * @param clearbits	Bits in the register to clear.
     * @param setbits	Bits in the register to set.
     */
    void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

    /**
     * Write a register in the LIS2HH12, updating _checked_values
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void			write_checked_reg(unsigned reg, uint8_t value);

    /**
     * Set the L3GD20 measurement range.
     *
     * @param max_dps	The measurement range is set to permit reading at least
     *			this rate in degrees per second.
     *			Zero selects the maximum supported range.
     * @return		OK if the value can be supported, -ERANGE otherwise.
     */
    int			set_range(unsigned max_g);

    /**
     * Set the L3GD20 internal sampling frequency.
     *
     * @param frequency	The internal sampling frequency is set to not less than
     *			this value.
     *			Zero selects the maximum rate supported.
     * @return		OK if the value can be supported.
     */
    int			set_samplerate(unsigned frequency);

    /**
     * Set the lowpass filter of the driver
     *
     * @param samplerate	The current samplerate
     * @param frequency	The cutoff frequency for the lowpass filter
     */
    void			set_driver_lowpass_filter(float samplerate, float bandwidth);

    /**
     * Self test
     *
     * @return 0 on success, 1 on failure
     */
    int 			self_test();

    /* this class does not allow copying */
    LIS2HH12(const LIS2HH12 &);
    LIS2HH12 operator=(const LIS2HH12 &);
};

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t LIS2HH12::_checked_registers[LIS2HH12_NUM_CHECKED_REGISTERS] = { LISREG_WHO_AM_I,
                                                                               LISREG_CTRL_1,
                                                                               LISREG_CTRL_2,
                                                                               LISREG_CTRL_3,
                                                                               LISREG_CTRL_4,
                                                                               LISREG_CTRL_5,
                                                                               LISREG_CTRL_6,
                                                                               LISREG_CTRL_7
                                                                             };
/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int lis2hh12_main(int argc, char *argv[]);


LIS2HH12::LIS2HH12(int bus, const char *path, enum Rotation rotation, uint16_t address) :
    I2C("LIS2HH12", path, bus, address, 100*1000),
    _work{},
    _call{},
    _call_interval(0),
    _measure_ticks(0),
    _reports(nullptr),
    _accel_scale{},
    _accel_range_scale(0.0f),
    _accel_range_m_s2(0.0f),
    _accel_topic(nullptr),
    _orb_class_instance(-1),
    _class_instance(-1),
    _current_rate(0),
    _orientation(SENSOR_BOARD_ROTATION_DEFAULT),
    _read(0),
    _sample_perf(perf_alloc(PC_ELAPSED, "lis2hh12_read")),
    _errors(perf_alloc(PC_COUNT, "lis2hh12_errors")),
    _bad_registers(perf_alloc(PC_COUNT, "lis2hh12_bad_registers")),
    _duplicates(perf_alloc(PC_COUNT, "lis2hh12_duplicates")),
    _register_wait(0),
    _accel_filter_x( LIS2HH12_ACCEL_DEFAULT_RATE, LIS2HH12_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_y(LIS2HH12_ACCEL_DEFAULT_RATE, LIS2HH12_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_z(LIS2HH12_ACCEL_DEFAULT_RATE, LIS2HH12_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_int(1000000 / LIS2HH12_ACCEL_MAX_RATE, true),
    _rotation(rotation),
    _checked_next(0)
{
    // enable debug() calls
    _debug_enabled = true;

    _device_id.devid_s.devtype = DRV_ACC_DEVTYPE_LIS2HH12;

    // default scale factors
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    // work_cancel in the dtor will explode if we don't do this...
    memset(&_work, 0, sizeof(_work));
}

LIS2HH12::~LIS2HH12()
{
    /* make sure we are truly inactive */
    stop();

    if (_reports != nullptr) {
        delete _reports;
    }

    if (_class_instance != -1) {
        unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _class_instance);
    }

    // free perf counters
    perf_free(_sample_perf);
    perf_free(_errors);
    perf_free(_bad_registers);
    perf_free(_duplicates);


}

int
LIS2HH12::init()
{
    warnx("ENTREI");
    int ret = ERROR;

    /* do SPI init (and probe) first */
    if (I2C::init() != OK) {
        warnx("FALHOU I2C");
        goto out;
    }
    warnx("PASSEI I2C");
    /* allocate basic report buffers */
    _reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

    if (_reports == nullptr) {
        goto out;
    }

    _class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

    reset();

    measure();

    /* advertise sensor topic, measure manually to initialize valid report */
    struct accel_report arp;
    _reports->get(&arp);

    _accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
                      &_orb_class_instance, (is_external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);

    if (_accel_topic == nullptr) {
        DEVICE_DEBUG("failed to create sensor_accel publication");
    }

    warnx("SAI 1");
    ret = OK;
out:
    warnx("SAI INIT");
    return ret;
}

int
LIS2HH12::probe()
{
    /* read dummy value to void to clear SPI statemachine on sensor */
    (void)read_reg(LISREG_WHO_AM_I);

    /* verify that the device is attached and functioning */
    bool success = (read_reg(LISREG_WHO_AM_I) == LIS_WHO_AM_I);
    printf("WHOAMI:          %u\n", read_reg(LISREG_WHO_AM_I));
    if (success) {
        _checked_values[0] = LIS_WHO_AM_I;
        printf("WHOAMI:          %u\n", _checked_values[0]);
        return OK;
    }

    return -EIO;
}

ssize_t
LIS2HH12::read(struct file *filp, char *buffer, size_t buflen)
{
    warnx("VIM LER");
    unsigned count = buflen / sizeof(struct accel_report);
    struct accel_report *abuf = reinterpret_cast<struct accel_report *>(buffer);
    int ret = 0;

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is enabled */

    //if (_call_interval > 0) {
    if (_measure_ticks > 0) {

        /*
         * While there is space in the caller's buffer, and reports, copy them.
         * Note that we may be pre-empted by the measurement code while we are doing this;
         * we are careful to avoid racing with it.
         */
        while (count--) {
            if (_reports->get(abuf)) {
                ret += sizeof(*abuf);
                abuf++;
            }
        }

        /* if there was no data, warn the caller */
        return ret ? ret : -EAGAIN;
    }

    /* manual measurement */
    _reports->flush();
    measure();

    /* measurement will have generated a report, copy it out */
    if (_reports->get(abuf)) {
        ret = sizeof(*abuf);
    }

    return ret;
}

int
LIS2HH12::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    //warnx("IOCTL");
    switch (cmd) {

    case SENSORIOCSPOLLRATE: {
            switch (arg) {

            /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL:
                stop();
                //_call_interval = 0;
                _measure_ticks =0;
                return OK;

            /* external signalling not supported */
            case SENSOR_POLLRATE_EXTERNAL:

            /* zero would be bad */
            case 0:
                return -EINVAL;

            /* set default/max polling rate */
            case SENSOR_POLLRATE_MAX:
            case SENSOR_POLLRATE_DEFAULT:
            {
                 bool want_start = (_measure_ticks == 0);

                //_measure_ticks = USEC2TICK(1000000 / LIS2HH12_ACCEL_DEFAULT_RATE);
                 //RESOLVER
                _measure_ticks = 5;
                 if (want_start) {
                     start();
                 }

                 return OK;
            }


                //return ioctl(filp, SENSORIOCSPOLLRATE, LIS2HH12_ACCEL_DEFAULT_RATE);

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    //bool want_start = (_call_interval == 0);
                    bool want_start = (_measure_ticks == 0);

                    /* convert hz to hrt interval via microseconds */
                    //unsigned ticks = 1000000 / arg;

                    /* convert hz to tick interval via microseconds */
                    unsigned ticks = USEC2TICK(1000000 / arg);

                    /* check against maximum rate */
                    if (ticks < USEC2TICK(1000000 / LIS2HH12_ACCEL_MAX_RATE)) {
                        return -EINVAL;
                    }

                    /* check against maximum sane rate */
                    //if (ticks < 1000) {
                     //   return -EINVAL;
                    //}

                    /* update interval for next measurement */
                    /* XXX this is a bit shady, but no other way to adjust... */
                    //_call_interval = ticks;
                    _measure_ticks = ticks;

                    //_call.period = _call_interval - LIS2HH12_TIMER_REDUCTION;

                    /* adjust filters */
                    //float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
                    //float sample_rate = 1.0e6f / ticks;
                    //set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }
            }
        }

    case SENSORIOCGPOLLRATE:
        //if (_call_interval == 0) {
        if (_measure_ticks == 0) {
            return SENSOR_POLLRATE_MANUAL;
        }

        //return 1000000 / _call_interval;
        return 1000000 / TICK2USEC(_measure_ticks);

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = irqsave();

            if (!_reports->resize(arg)) {
                irqrestore(flags);
                return -ENOMEM;
            }

            irqrestore(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _reports->size();

    case SENSORIOCRESET:
        reset();
        return OK;

    case ACCELIOCSSAMPLERATE:
        //return set_samplerate(arg);
        set_samplerate(arg);
        return ioctl(filp, SENSORIOCSPOLLRATE, arg);

    case ACCELIOCGSAMPLERATE:
        //return _current_rate;
        return 1000000 / TICK2USEC(_measure_ticks);

    case ACCELIOCSLOWPASS: {
            float cutoff_freq_hz = arg;
            //float sample_rate = 1.0e6f / _call_interval;
            float sample_rate = 1.0e6f / _measure_ticks;
            set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

            return OK;
        }

    case ACCELIOCGLOWPASS:
        return static_cast<int>(_accel_filter_x.get_cutoff_freq());

    case ACCELIOCSSCALE:
        /* copy scale in */
        memcpy(&_accel_scale, (struct accel_scale *) arg, sizeof(_accel_scale));
        return OK;

    case ACCELIOCGSCALE:
        /* copy scale out */
        memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
        return OK;

    case ACCELIOCSRANGE:
        /* arg should be in dps */
        return set_range(arg);

    case ACCELIOCGRANGE:
        /* convert to g and round */
        return (unsigned long)((_accel_range_m_s2) / LIS2HH12_ONE_G + 0.5f);

    case ACCELIOCSELFTEST:
        return self_test();

    default:
        /* give it to the superclass */
        return I2C::ioctl(filp, cmd, arg);
    }
}

uint8_t
LIS2HH12::read_reg(unsigned reg)
{
    //uint8_t cmd[2];
    uint8_t cmd = reg;
    //uint8_t ext = 1;

    //cmd[0] = reg | DIR_READ;
    //cmd = reg;

    //cmd[1] = 0;

    transfer(&cmd, sizeof(cmd), &cmd, sizeof(cmd));

    //ext = 0x41;
    return cmd;
}

void
LIS2HH12::write_reg(unsigned reg, uint8_t value)
{
    uint8_t	cmd[2];

    cmd[0] = reg;
    cmd[1] = value;

    //transfer(&cmd[0], sizeof(cmd), nullptr, 0);
    transfer(&cmd[0], 2, nullptr, 0);
}


void
LIS2HH12::write_checked_reg(unsigned reg, uint8_t value)
{
    write_reg(reg, value);

    for (uint8_t i = 0; i < LIS2HH12_NUM_CHECKED_REGISTERS; i++) {
        if (reg == _checked_registers[i]) {
            _checked_values[i] = value;
        }
    }
}

void
LIS2HH12::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t	val;

    val = read_reg(reg);
    val &= ~clearbits;
    val |= setbits;
    write_checked_reg(reg, val);
}

int LIS2HH12::set_range(unsigned max_g)
{
    uint8_t setbits = 0;
    uint8_t clearbits = LIS_RANGE_8_G;
    float lsb_per_g;

    if (max_g <= 2) {
        setbits = LIS_RANGE_2_G;
        lsb_per_g = 16384;
        _accel_range_m_s2 = 2 * LIS2HH12_ONE_G;

    } else if (max_g <= 4) {
        setbits = LIS_RANGE_4_G;
        lsb_per_g = 8192;
        _accel_range_m_s2 = 4 * LIS2HH12_ONE_G;

    } else if (max_g > 4) {
        setbits = LIS_RANGE_8_G;
        lsb_per_g = 4096;
        _accel_range_m_s2 = 8 * LIS2HH12_ONE_G;

    } else {
        return -EINVAL;
    }

    _accel_range_scale = LIS2HH12_ONE_G / lsb_per_g;

    modify_reg(LISREG_CTRL_4, clearbits, setbits);

    warnx("RANGE");
    return OK;
}

int LIS2HH12::set_samplerate(unsigned frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = LIS_ODR_800 | LIS_ODR_10;

    if (frequency == 0) {
        frequency = LIS2HH12_ACCEL_DEFAULT_RATE;
    }

    if (frequency <= 10) {
        setbits = LIS_ODR_10;
        _current_rate = 10;

    } else if (frequency <= 50) {
        setbits = LIS_ODR_50;
        _current_rate= 50;

    } else if (frequency <= 100) {
        setbits = LIS_ODR_100;
        _current_rate = 100;

    } else if (frequency <= 200) {
        setbits = LIS_ODR_200;
        _current_rate = 200;

    } else if (frequency <= 400) {
        setbits = LIS_ODR_400;
        _current_rate = 400;

    } else if (frequency > 400) {
        setbits = LIS_ODR_800;
        _current_rate = 800;

    } else {
        return -EINVAL;
    }

    modify_reg(LISREG_CTRL_1, clearbits, setbits);

    warnx("SAMPLERATE");

    return OK;
}

void
LIS2HH12::set_driver_lowpass_filter(float samplerate, float bandwidth)
{
    _accel_filter_x.set_cutoff_frequency(samplerate, bandwidth);
    _accel_filter_y.set_cutoff_frequency(samplerate, bandwidth);
    _accel_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}

void
LIS2HH12::start()
{
    /* make sure we are stopped first */
    stop();

    /* reset the report ring */
    _reports->flush();



    /* start polling at the specified rate */
    /*hrt_call_every(&_call,
               1000,
               _call_interval - LIS2HH12_TIMER_REDUCTION,
               (hrt_callout)&LIS2HH12::measure_trampoline, this);*/

    work_queue(HPWORK,
           &_work,
           (worker_t)&LIS2HH12::measure_trampoline,
           this,
               1);
           //USEC2TICK(1000000 / _current_rate));

    warnx("START RATE: %d", _current_rate);
    //work_queue(HPWORK, &_work, (worker_t)&HMC5883::cycle_trampoline, this, 1);

    warnx("CHEGUEI");
}

void
LIS2HH12::stop()
{
    work_cancel(HPWORK, &_work);
    //hrt_cancel(&_call);
}


void
LIS2HH12::reset()
{

    warnx("RESET");
    write_reg(LISREG_CTRL_5, LIS_SOFT_RESET);

    write_checked_reg(LISREG_CTRL_1, LIS_BDU | LIS_ALL_AXES_EN);
    write_checked_reg(LISREG_CTRL_2, 0);
    write_checked_reg(LISREG_CTRL_3, LIS_INT1_IG_EN | LIS_INT1_DRDY);
    //write_checked_reg(LISREG_CTRL_4, LIS_IF_ADD_INC);
    write_checked_reg(LISREG_CTRL_4, 0);
    write_checked_reg(LISREG_CTRL_5, 0);
    write_checked_reg(LISREG_CTRL_6, 0);
    write_checked_reg(LISREG_CTRL_7, 0);

    set_samplerate(LIS2HH12_ACCEL_DEFAULT_RATE);
    set_range(LIS2HH12_ACCEL_DEFAULT_RANGE_G);
    set_driver_lowpass_filter(LIS2HH12_ACCEL_DEFAULT_RATE, LIS2HH12_DEFAULT_DRIVER_FILTER_FREQ);

    _read = 0;
    warnx("FIM RESET");
}

void
LIS2HH12::cycle()
{
    if (_measure_ticks > USEC2TICK(1000000 / _current_rate)) {
        measure();
    }

    work_queue(HPWORK,
           &_work,
           (worker_t)&LIS2HH12::measure_trampoline,
           this,
           _measure_ticks - USEC2TICK(1000000 / _current_rate));

    warnx("RATE: %d", _current_rate);
}

void
LIS2HH12::measure_trampoline(void *arg)
{
    LIS2HH12 *dev = (LIS2HH12 *)arg;

    /* make another measurement */
    //dev->measure();
    dev->cycle();
}

void
LIS2HH12::check_registers(void)
{
    warnx("CHECK dos REGISTOS");
    uint8_t v;

    if ((v = read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {
        /*
          if we get the wrong value then we know the SPI bus
          or sensor is very sick. We set _register_wait to 20
          and wait until we have seen 20 good values in a row
          before we consider the sensor to be OK again.
         */
        perf_count(_bad_registers);

        /*
          try to fix the bad register value. We only try to
          fix one per loop to prevent a bad sensor hogging the
          bus. We skip zero as that is the WHO_AM_I, which
          is not writeable
         */
        if (_checked_next != 0) {
            write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
        }

        _register_wait = 20;
    }

    _checked_next = (_checked_next + 1) % LIS2HH12_NUM_CHECKED_REGISTERS;
}


void
LIS2HH12::measure()
{

    /* status register and data as read back from the device */
#pragma pack(push, 1)
    struct {
        uint8_t		cmd;
        //int8_t		temp;
        //uint8_t		status;
        int16_t		x;
        int16_t		y;
        int16_t		z;
    } raw_report;
#pragma pack(pop)

    accel_report report;

    /* start the performance counter */
    perf_begin(_sample_perf);

    check_registers();

    /* fetch data from the sensor */
    memset(&raw_report, 0, sizeof(raw_report));

    uint8_t status = read_reg(LISREG_STATUS);

    uint8_t templ = read_reg(LISREG_TEMP_L);
    uint8_t temph = (read_reg(LISREG_TEMP_H) & 0x0F);

    int16_t temp = (temph<<8) | templ;

    raw_report.cmd = LISREG_OUT_X_L;
    //transfer((uint8_t *)&raw_report, sizeof(raw_report), (uint8_t *)&raw_report, sizeof(raw_report));
    //transfer((uint8_t *)&raw_report, 1, (uint8_t *)&raw_report, sizeof(raw_report));


    raw_report.x = (read_reg(LISREG_OUT_X_H) << 8) | read_reg(LISREG_OUT_X_L);
    raw_report.y = (read_reg(LISREG_OUT_Y_H) << 8) | read_reg(LISREG_OUT_Y_L);
    raw_report.z = (read_reg(LISREG_OUT_Z_H) << 8) | read_reg(LISREG_OUT_Z_L);

    if (!(status && LIS_STATUS_ZYXDA)) {
        perf_end(_sample_perf);
        perf_count(_duplicates);
        return;
    }

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
    report.timestamp = hrt_absolute_time();
    report.error_count = perf_event_count(_bad_registers);

    report.x_raw = raw_report.x;
    report.y_raw = -raw_report.y;
    report.z_raw = raw_report.z;

    report.temperature_raw = temp;

    float xraw_f = report.x_raw;
    float yraw_f = report.y_raw;
    float zraw_f = report.z_raw;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    float xin = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
    float yin = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
    float zin = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

    report.x = _accel_filter_x.apply(xin);
    report.y = _accel_filter_y.apply(yin);
    report.z = _accel_filter_z.apply(zin);

    math::Vector<3> aval(xin, yin, zin);
    math::Vector<3> aval_integrated;

    bool accel_notify = _accel_int.put(report.timestamp, aval, aval_integrated, report.integral_dt);
    report.x_integral = aval_integrated(0);
    report.y_integral = aval_integrated(1);
    report.z_integral = aval_integrated(2);

    report.temperature = 22.5f + temp * 0.125f;

    report.scaling = _accel_range_scale;
    report.range_m_s2 = _accel_range_m_s2;

    _reports->force(&report);

    if (accel_notify) {
        /* notify anyone waiting for data */
        poll_notify(POLLIN);

        /* publish for subscribers */
        if (!(_pub_blocked)) {
            /* publish it */
            orb_publish(ORB_ID(sensor_accel), _accel_topic, &report);
        }
    }

    _read++;

    /* stop the perf counter */
    perf_end(_sample_perf);





    /*warnx("accel x: \t% 9.5f\tm/s^2", (double)report.x);
    warnx("accel y: \t% 9.5f\tm/s^2", (double)report.y);
    warnx("accel z: \t% 9.5f\tm/s^2", (double)report.z);
    warnx("temp: \t%d\tC", (int)report.temperature);
    warnx("accel x: \t%d\traw", (int)report.x_raw);
    warnx("accel y: \t%d\traw", (int)report.y_raw);
    warnx("accel z: \t%d\traw", (int)report.z_raw);
    warnx("temp: \t%d\traw", (int)report.temperature_raw);
    warnx("accel range: %8.4f m/s^2", (double)report.range_m_s2);*/

}

void
LIS2HH12::print_info()
{
    printf("accel reads:          %u\n", _read);
    perf_print_counter(_sample_perf);
    perf_print_counter(_errors);
    perf_print_counter(_bad_registers);
    perf_print_counter(_duplicates);
    printf("poll interval:  %u ticks\n", _measure_ticks);
    _reports->print_info("report queue");
    ::printf("checked_next: %u\n", _checked_next);

    for (uint8_t i = 0; i < LIS2HH12_NUM_CHECKED_REGISTERS; i++) {
        uint8_t v = read_reg(_checked_registers[i]);

        if (v != _checked_values[i]) {
            ::printf("reg %02x:%02x should be %02x\n",
                 (unsigned)_checked_registers[i],
                 (unsigned)v,
                 (unsigned)_checked_values[i]);
        }
    }
}

void
LIS2HH12::print_registers()
{
    printf("LIS2HH12 registers\n");

    for (uint8_t reg = 0; reg <= 0x40; reg++) {
        uint8_t v = read_reg(reg);
        printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

        if ((reg + 1) % 16 == 0) {
            printf("\n");
        }
    }

    printf("\n");
}

void
LIS2HH12::test_error()
{
    // trigger a deliberate error
    //write_reg(ADDR_CTRL_REG3, 0);
    write_reg(LISREG_CTRL_5, LIS_SOFT_RESET);
}

int
LIS2HH12::self_test()
{
    if (_read == 0) {
        return 1;
    }

    /* inspect accel offsets */
    if (fabsf(_accel_scale.x_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.y_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.z_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f) {
        return 1;
    }

    return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace lis2hh12
{

LIS2HH12	*g_dev;

void	usage();
void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	regdump();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */


void
start(bool external_bus, enum Rotation rotation)
{
    int fd;

    if (g_dev != nullptr) {
        errx(0, "already started");
    }

    /* create the driver */
    if (external_bus) {
/*#ifdef PX4_SPI_BUS_EXT
        g_dev = new L3GD20(PX4_SPI_BUS_EXT, L3GD20_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_EXT_GYRO, rotation);
#else
        errx(0, "External SPI not available");
#endif*/

    } else {
        g_dev = new LIS2HH12(PX4_I2C_BUS_ONBOARD, LIS2HH12_DEVICE_PATH_ACCEL, rotation, PX4_I2C_OBDEV_LIS2HH12);
    }

    if (g_dev == nullptr) {
        goto fail;
    }

    if (OK != g_dev->init()) {
        goto fail;

    }

    /* set the poll rate to default, starts automatic data collection */
    fd = open(LIS2HH12_DEVICE_PATH_ACCEL, O_RDONLY);

    if (fd < 0) {
        goto fail;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        goto fail;

    }

    close(fd);

    warnx("START COMPLETO");
    exit(0);
fail:

    if (g_dev != nullptr) {
        delete g_dev;
        g_dev = nullptr;
    }

    errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
    int fd_accel = -1;
    struct accel_report a_report;
    ssize_t sz;

    /* get the driver */
    fd_accel = open(LIS2HH12_DEVICE_PATH_ACCEL, O_RDONLY);

    if (fd_accel < 0) {
        err(1, "%s open failed", LIS2HH12_DEVICE_PATH_ACCEL);
    }

    /* reset to manual polling */
    if (ioctl(fd_accel, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
        err(1, "reset to manual polling");
    }

    /* do a simple demand read */
    sz = read(fd_accel, &a_report, sizeof(a_report));

    if (sz != sizeof(a_report)) {
        err(1, "immediate accel read failed");
    }

    warnx("accel x: \t% 9.5f\tm/s^2", (double)a_report.x);
    warnx("accel y: \t% 9.5f\tm/s^2", (double)a_report.y);
    warnx("accel z: \t% 9.5f\tm/s^2", (double)a_report.z);
    warnx("temp: \t%d\tC", (int)a_report.temperature);
    warnx("accel x: \t%d\traw", (int)a_report.x_raw);
    warnx("accel y: \t%d\traw", (int)a_report.y_raw);
    warnx("accel z: \t%d\traw", (int)a_report.z_raw);
    warnx("temp: \t%d\traw", (int)a_report.temperature_raw);
    warnx("accel range: %8.4f m/s^2", (double)a_report.range_m_s2);

    if (ioctl(fd_accel, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "reset to default polling");
    }

    close(fd_accel);

    /* XXX add poll-rate tests here too */
    errx(0, "PASS");
}



/**
 * Reset the driver.
 */
void
reset()
{
    int fd = open(LIS2HH12_DEVICE_PATH_ACCEL, O_RDONLY);

    if (fd < 0) {
        err(1, "failed ");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
        err(1, "driver reset failed");
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "accel pollrate reset failed");
    }

    close(fd);

    exit(0);
}


/**
 * Print a little info about the driver.
 */
void
info()
{
    if (g_dev == nullptr) {
        errx(1, "driver not running\n");
    }

    printf("state @ %p\n", g_dev);
    g_dev->print_info();

    exit(0);
}

/**
 * Dump the register information
 */
void
regdump(void)
{
    if (g_dev == nullptr) {
        errx(1, "driver not running");
    }

    printf("regdump @ %p\n", g_dev);
    g_dev->print_registers();

    exit(0);
}

/**
 * trigger an error
 */
void
test_error(void)
{
    if (g_dev == nullptr) {
        errx(1, "driver not running");
    }

    printf("regdump @ %p\n", g_dev);
    g_dev->test_error();

    exit(0);
}

void
usage()
{
    warnx("missing command: try 'start', 'info', 'test', 'reset', 'testerror' or 'regdump'");
    warnx("options:");
    warnx("    -X    (external bus)");
    warnx("    -R rotation");
}

} // namespace


int
lis2hh12_main(int argc, char *argv[])
{
    bool external_bus = false;
    int ch;
    enum Rotation rotation = ROTATION_NONE;

    /* jump over start/off/etc and look at options first */
    while ((ch = getopt(argc, argv, "XR:")) != EOF) {
        switch (ch) {
        case 'X':
            external_bus = true;
            break;

        case 'R':
            rotation = (enum Rotation)atoi(optarg);
            break;

        default:
            lis2hh12::usage();
            exit(0);
        }
    }

    const char *verb = argv[optind];

    /*
     * Start/load the driver.

     */
    if (!strcmp(verb, "start")) {
        lis2hh12::start(external_bus, rotation);
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(verb, "test")) {
        lis2hh12::test();
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        lis2hh12::reset();
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info")) {
        lis2hh12::info();
    }

    /*
     * Print register information.
     */
    if (!strcmp(verb, "regdump")) {
        lis2hh12::regdump();
    }

    /*
     * trigger an error
     */
    if (!strcmp(verb, "testerror")) {
        lis2hh12::test_error();
    }

    errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
}
