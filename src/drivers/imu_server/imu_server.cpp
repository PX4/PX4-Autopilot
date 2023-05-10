#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/imu_server.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/matrix/matrix/math.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "imu_server.hpp"

static px4_task_t  _thread_tid;
static const char *_thread_name = "imu_server_thread";

static IMU_Server  _server;

using namespace matrix;

// Copied from another file. Needs to stay the same.
typedef struct icm4x6xx_imu_data_t {
	float accl_ms2[3];         ///< XYZ acceleration in m/s^2
	float gyro_rad[3];         ///< XYZ gyro rotation in rad/s
    float temp_c;         ///< temperature in degrees Celcius
	uint64_t timestamp_monotonic_ns; ///< Monotonic timestamp
	uint64_t dummy0;   ///< Not used for VIO
	uint64_t dummy1;   ///< Not used for VIO
} __attribute__((packed)) icm4x6xx_imu_data_t;

static int _imu_server_thread(int argc, char *argv[]) {

    PX4_INFO("imu_server thread starting");

    const char *imu_fifo = "/dev/imu-pipe0";
    struct stat pipe_stat;
    int stat_rc = lstat(imu_fifo, &pipe_stat);
    if (stat_rc == 0) {
        // The path already exists. Make sure it is a pipe.
        if ( ! S_ISFIFO(pipe_stat.st_mode)) {
            PX4_ERR("Error: %s exists but it is not a pipe", imu_fifo);
            return PX4_ERROR;
        } else {
            PX4_INFO("%s exists and it is a pipe", imu_fifo);
        }
    } else {
        // The pipe does not exist yet. Create it.
        int mkfifo_rc = mkfifo(imu_fifo, 0666);
        if (mkfifo_rc) {
            PX4_ERR("Error: Couldn't create pipe %s", imu_fifo);
            return PX4_ERROR;
        } else {
            PX4_INFO("Created pipe %s", imu_fifo);
        }
    }

    // The call to open will block until a reader attaches to the pipe
    int fifo_fd = open(imu_fifo, O_WRONLY);
    if (fifo_fd == -1) {
        PX4_ERR("Error: Couldn't open pipe %s", imu_fifo);
        return PX4_ERROR;
    } else {
        PX4_INFO("Opened pipe %s for writing", imu_fifo);
    }

    int imu_server_fd = orb_subscribe(ORB_ID(imu_server));

    imu_server_s          received_data;
    icm4x6xx_imu_data_t   imu_data;

    memset(&imu_data, 0, sizeof(imu_data));

	uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel)};
	const uint32_t accel_device_id = sensor_accel_sub.get().device_id;
	calibration::Accelerometer _accel_calibration(accel_device_id);
    PX4_INFO("IMU Server found accel device id %u", accel_device_id);

	uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro)};
	const uint32_t gyro_device_id = sensor_gyro_sub.get().device_id;
	calibration::Gyroscope _gyro_calibration(gyro_device_id);
    PX4_INFO("IMU Server found gyro device id %u", gyro_device_id);

    px4_pollfd_struct_t fds[1] = { { .fd = imu_server_fd,  .events = POLLIN } };
    while (true) {
    	px4_poll(fds, 1, 1000);
    	if (fds[0].revents & POLLIN) {
            orb_copy(ORB_ID(imu_server), imu_server_fd, &received_data);
            // PX4_INFO("Got imu data %lu", received_data.timestamp);
            for (int i = 0; i < 10; i++) {
		        const Vector3f accel_corrected = _accel_calibration.Correct(Vector3f{received_data.accel_x[i], received_data.accel_y[i], received_data.accel_z[i]});
		        const Vector3f gyro_corrected  = _gyro_calibration.Correct(Vector3f{received_data.gyro_x[i], received_data.gyro_y[i], received_data.gyro_z[i]});

                imu_data.timestamp_monotonic_ns = received_data.ts[i] * 1000;
                for (int j = 0; j < 3; j++) {
                    imu_data.accl_ms2[j] = accel_corrected(j);
                    imu_data.gyro_rad[j] = gyro_corrected(j);
                }

                // Write the data to the fifo
                size_t data_len = sizeof(icm4x6xx_imu_data_t);
                ssize_t bytes_written = write(fifo_fd, &imu_data, data_len);
                size_t unsigned_bytes_written = (bytes_written > 0) ? (size_t) bytes_written : 0;
                if ((bytes_written > 0) && (unsigned_bytes_written == data_len)) {
                    // PX4_INFO("Wrote %ld IMU data bytes to %s", unsigned_bytes_written, imu_fifo);
                } else {
                    PX4_ERR("Error: Couldn't write %lu bytes to the pipe", data_len);
                    PX4_ERR("       write returned %ld", bytes_written);
                    break;
                }

                // PX4_INFO("%d %lu %.2f %.2f %.2f %.2f %.2f %.2f",
                //          i, received_data.ts[i],
                //          (double) received_data.accel_x[i], (double) received_data.accel_y[i], (double) received_data.accel_z[i],
                //          (double) received_data.gyro_x[i], (double) received_data.gyro_y[i], (double) received_data.gyro_z[i]);
                // PX4_INFO("%d %lu %.2f %.2f %.2f %.2f %.2f %.2f",
                //          i, received_data.ts[i],
                //          (double) imu_data.accl_ms2[0], (double) imu_data.accl_ms2[1], (double) imu_data.accl_ms2[2],
                //          (double) imu_data.gyro_rad[0], (double) imu_data.gyro_rad[1], (double) imu_data.gyro_rad[2]);
            }
    	}
    }

    close(fifo_fd);

    PX4_INFO("imu_server thread ending");

    return PX4_OK;
}

int imu_server_main(int argc, char *argv[]) {

    PX4_INFO("imu_server_main");

    if ( ! _server.is_running()) {
        _server.start();
        _thread_tid = px4_task_spawn_cmd(_thread_name,
    			                         SCHED_DEFAULT,
    			                         SCHED_PRIORITY_PARAMS,
    			                         (1024 * 4),
    			                         _imu_server_thread,
    			                         NULL);
    }

    return 0;

}
