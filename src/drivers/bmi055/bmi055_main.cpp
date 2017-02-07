#include "bmi055.hpp"

/** driver 'main' command */
extern "C" { __EXPORT int bmi055_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */


namespace bmi055
{

BMI055_accel    *g_acc_dev_int; // on internal bus (accel)
BMI055_accel    *g_acc_dev_ext; // on external bus (accel)
BMI055_gyro     *g_gyr_dev_int; // on internal bus (gyro)
BMI055_gyro     *g_gyr_dev_ext; // on external bus (gyro)


void    start(bool, enum Rotation);
void    stop(bool);
void    test(bool);
void    reset(bool);
void    info(bool);
void    regdump(bool);
void    testerror(bool);
void    usage();


/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
    int fd_acc;
    int fd_gyr;
    BMI055_accel **g_dev_acc_ptr = external_bus ? &g_acc_dev_ext : &g_acc_dev_int;
    BMI055_gyro **g_dev_gyr_ptr = external_bus ? &g_gyr_dev_ext : &g_gyr_dev_int;
    const char *path_accel = external_bus ? BMI055_DEVICE_PATH_ACCEL_EXT : BMI055_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? BMI055_DEVICE_PATH_GYRO_EXT : BMI055_DEVICE_PATH_GYRO;


    if (*g_dev_acc_ptr != nullptr)
        /* if already started, the still command succeeded */
    {
        errx(0, "already started");
    }

    /* create the driver */
    if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_BMI)
        *g_dev_acc_ptr = new BMI055_accel(PX4_SPI_BUS_EXT, path_accel, (spi_dev_e)PX4_SPIDEV_EXT_BMI, rotation);
#else
        errx(0, "External SPI not available");
#endif

    } else {
        *g_dev_acc_ptr = new BMI055_accel(PX4_SPI_BUS_SENSORS, path_accel, (spi_dev_e)PX4_SPIDEV_BMI055_ACC, rotation);
    }

    if (*g_dev_acc_ptr == nullptr) {
        goto fail_accel;
    }

    if (OK != (*g_dev_acc_ptr)->init()) {
        warnx("error 2");
        goto fail_accel;
    }

    /* set the poll rate to default, starts automatic data collection */
    fd_acc = open(path_accel, O_RDONLY);

    if (fd_acc < 0) {
        goto fail_accel;
    }

    if (ioctl(fd_acc, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        warnx("error 4");
        goto fail_accel;
    }

    close(fd_acc);

    if (*g_dev_gyr_ptr != nullptr)
        /* if already started, the still command succeeded */
    {
        errx(0, "gyro sensor already started");
    }

    /* create the driver */
    if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_BMI)
        *g_dev_ptr = new BMI055_gyro(PX4_SPI_BUS_EXT, path_gyro, (spi_dev_e)PX4_SPIDEV_EXT_BMI, rotation);
#else
        errx(0, "External SPI not available");
#endif

    } else {
        *g_dev_gyr_ptr = new BMI055_gyro(PX4_SPI_BUS_SENSORS, path_gyro, (spi_dev_e)PX4_SPIDEV_BMI055_GYR, rotation);
    }

    if (*g_dev_gyr_ptr == nullptr) {
        goto fail_gyro;
    }

    if (OK != (*g_dev_gyr_ptr)->init()) {
        goto fail_gyro;
    }

    /* set the poll rate to default, starts automatic data collection */
    fd_gyr = open(path_gyro, O_RDONLY);

    if (fd_gyr < 0) {
        goto fail_gyro;
    }

    if (ioctl(fd_gyr, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        goto fail_gyro;
    }

    close(fd_gyr);
    exit(0);

fail_accel:

    if (*g_dev_acc_ptr != nullptr) {
        delete(*g_dev_acc_ptr);
        *g_dev_acc_ptr = nullptr;
    }

    errx(1, "accel driver start failed");

fail_gyro:

    if (*g_dev_gyr_ptr != nullptr) {
        delete(*g_dev_gyr_ptr);
        *g_dev_gyr_ptr = nullptr;
    }

    errx(1, "gyro driver start failed");

}

void
stop(bool external_bus)
{
    BMI055_accel **g_dev_acc_ptr = external_bus ? &g_acc_dev_ext : &g_acc_dev_int;
    BMI055_gyro **g_dev_gyr_ptr = external_bus ? &g_gyr_dev_ext : &g_gyr_dev_int;

    if (*g_dev_acc_ptr != nullptr) {
        delete *g_dev_acc_ptr;
        *g_dev_acc_ptr = nullptr;

    } else {
        /* warn, but not an error */
        warnx("accel sensor already stopped.");
    }

    if (*g_dev_gyr_ptr != nullptr) {
        delete *g_dev_gyr_ptr;
        *g_dev_gyr_ptr = nullptr;

    } else {
        /* warn, but not an error */
        warnx("gyro sensor already stopped.");
    }

    exit(0);

}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
    const char *path_accel = external_bus ? BMI055_DEVICE_PATH_ACCEL_EXT : BMI055_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? BMI055_DEVICE_PATH_GYRO_EXT : BMI055_DEVICE_PATH_GYRO;
    accel_report a_report;
    gyro_report g_report;
    ssize_t sz;

    /* get the accel driver */
    int fd_acc = open(path_accel, O_RDONLY);

    if (fd_acc < 0)
        err(1, "%s open failed (try 'bmi055 start')",
            path_accel);

    /* get the gyro driver */
    int fd_gyr = open(path_gyro, O_RDONLY);

    if (fd_gyr < 0) {
        err(1, "%s gyro open failed", path_gyro);
    }

    /* reset to manual polling */
    if (ioctl(fd_acc, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
        err(1, "accel reset to manual polling");
    }

    /* do a simple demand read */
    sz = read(fd_acc, &a_report, sizeof(a_report));

    if (sz != sizeof(a_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(a_report));
        err(1, "immediate accel read failed");
    }

    warnx("single read");
    warnx("time:     %lld", a_report.timestamp);
    warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
    warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
    warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
    warnx("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
    warnx("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
    warnx("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
    warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
          (double)(a_report.range_m_s2 / BMI055_ONE_G));
    warnx("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);
    warnx("temp:  \t%d\traw 0x%0x", (short)g_report.temperature_raw, (unsigned short)a_report.temperature_raw);


    /* reset to default polling */
    if (ioctl(fd_acc, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "accel reset to default polling");
    }

    close(fd_acc);

    /* reset to manual polling */
    if (ioctl(fd_gyr, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
        err(1, "gyro reset to manual polling");
    }


    /* do a simple demand read */
    sz = read(fd_gyr, &g_report, sizeof(g_report));

    if (sz != sizeof(g_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(g_report));
        err(1, "immediate gyro read failed");
    }

    warnx("gyr x: \t% 9.5f\trad/s", (double)g_report.x);
    warnx("gyr y: \t% 9.5f\trad/s", (double)g_report.y);
    warnx("gyr z: \t% 9.5f\trad/s", (double)g_report.z);
    warnx("gyr x: \t%d\traw", (int)g_report.x_raw);
    warnx("gyr y: \t%d\traw", (int)g_report.y_raw);
    warnx("gyr z: \t%d\traw", (int)g_report.z_raw);
    warnx("gyr range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
          (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));


    /* reset to default polling */
    if (ioctl(fd_gyr, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "gyro reset to default polling");
    }

    close(fd_gyr);

    /* XXX add poll-rate tests here too */
    reset(external_bus);
    errx(0, "PASS");

}

/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
    const char *path_accel = external_bus ? BMI055_DEVICE_PATH_ACCEL_EXT : BMI055_DEVICE_PATH_ACCEL;
    const char *path_gyro = external_bus ? BMI055_DEVICE_PATH_GYRO_EXT : BMI055_DEVICE_PATH_GYRO;

    int fd_acc = open(path_accel, O_RDONLY);

    if (fd_acc < 0) {
        err(1, "Opening accel file failed ");
    }

    int fd_gyr = open(path_gyro, O_RDONLY);

    if (fd_gyr < 0) {
        err(1, "Opening gyro file failed ");
    }

    if (ioctl(fd_acc, SENSORIOCRESET, 0) < 0) {
        err(1, "accel driver reset failed");
    }


    if (ioctl(fd_acc, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "accel driver poll restart failed");
    }

    close(fd_acc);

    if (ioctl(fd_gyr, SENSORIOCRESET, 0) < 0) {
        err(1, "gyro driver reset failed");
    }

    if (ioctl(fd_gyr, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "gyro driver poll restart failed");
    }

    close(fd_gyr);

    exit(0);
}


/**
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
    BMI055_accel **g_dev_acc_ptr = external_bus ? &g_acc_dev_ext : &g_acc_dev_int;
    BMI055_gyro **g_dev_gyr_ptr = external_bus ? &g_gyr_dev_ext : &g_gyr_dev_int;

    if (*g_dev_acc_ptr == nullptr) {
        errx(1, "accel driver not running");
    }

    if (*g_dev_gyr_ptr == nullptr) {
        errx(1, "gyro driver not running");
    }

    printf("state @ %p\n", *g_dev_acc_ptr);
    (*g_dev_acc_ptr)->print_info();

    printf("state @ %p\n", *g_dev_gyr_ptr);
    (*g_dev_gyr_ptr)->print_info();

    exit(0);
}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
    BMI055_accel **g_dev_acc_ptr = external_bus ? &g_acc_dev_ext : &g_acc_dev_int;
    BMI055_gyro **g_dev_gyr_ptr = external_bus ? &g_gyr_dev_ext : &g_gyr_dev_int;


    if (*g_dev_acc_ptr == nullptr) {
        errx(1, "accel driver not running");
    }

    if (*g_dev_gyr_ptr == nullptr) {
        errx(1, "gyro driver not running");
    }

    printf("regdump @ %p\n", *g_dev_acc_ptr);
    (*g_dev_acc_ptr)->print_registers();

    printf("regdump @ %p\n", *g_dev_gyr_ptr);
    (*g_dev_gyr_ptr)->print_registers();

    exit(0);
}


/**
 * deliberately produce an error to test recovery
 */
void
testerror(bool external_bus)
{
    BMI055_accel **g_dev_acc_ptr = external_bus ? &g_acc_dev_ext : &g_acc_dev_int;
    BMI055_gyro **g_dev_gyr_ptr = external_bus ? &g_gyr_dev_ext : &g_gyr_dev_int;

    if (*g_dev_acc_ptr == nullptr) {
        errx(1, "accel driver not running");
    }

    if (*g_dev_gyr_ptr == nullptr) {
        errx(1, "gyro driver not running");
    }

    (*g_dev_acc_ptr)->test_error();
    (*g_dev_gyr_ptr)->test_error();

    exit(0);

}

void
usage()
{
    warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'testerror'");
    warnx("options:");
    warnx("    -X    (external bus)");
    warnx("    -R rotation");
}

}//namespace


int
bmi055_main(int argc, char *argv[])
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
            bmi055::usage();
            exit(0);
        }
    }

    const char *verb = argv[optind];

    /*
     * Start/load the driver.

     */
    if (!strcmp(verb, "start")) {
        bmi055::start(external_bus, rotation);
    }

    if (!strcmp(verb, "stop")) {
        bmi055::stop(external_bus);
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(verb, "test")) {
        bmi055::test(external_bus);
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        bmi055::reset(external_bus);
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info")) {
        bmi055::info(external_bus);
    }

    /*
     * Print register information.
     */
    if (!strcmp(verb, "regdump")) {
        bmi055::regdump(external_bus);
    }

    if (!strcmp(verb, "testerror")) {
        bmi055::testerror(external_bus);
    }

    bmi055::usage();
    exit(1);
}

