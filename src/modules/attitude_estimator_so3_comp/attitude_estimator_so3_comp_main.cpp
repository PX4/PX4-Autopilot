/*
 * Author: Hyon Lim <limhyon@gmail.com, hyonlim@snu.ac.kr>
 *
 * @file attitude_estimator_so3_comp_main.c
 *
 * Implementation of nonlinear complementary filters on the SO(3).
 * This code performs attitude estimation by using accelerometer, gyroscopes and magnetometer.
 * Result is provided as quaternion, 1-2-3 Euler angle and rotation matrix.
 * 
 * Theory of nonlinear complementary filters on the SO(3) is based on [1].
 * Quaternion realization of [1] is based on [2].
 * Optmized quaternion update code is based on Sebastian Madgwick's implementation.
 * 
 * References
 *  [1] Mahony, R.; Hamel, T.; Pflimlin, Jean-Michel, "Nonlinear Complementary Filters on the Special Orthogonal Group," Automatic Control, IEEE Transactions on , vol.53, no.5, pp.1203,1218, June 2008
 *  [2] Euston, M.; Coote, P.; Mahony, R.; Jonghyuk Kim; Hamel, T., "A complementary filter for attitude estimation of a fixed-wing UAV," Intelligent Robots and Systems, 2008. IROS 2008. IEEE/RSJ International Conference on , vol., no., pp.340,345, 22-26 Sept. 2008
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "attitude_estimator_so3_comp_params.h"
#ifdef __cplusplus
}
#endif

extern "C" __EXPORT int attitude_estimator_so3_comp_main(int argc, char *argv[]);

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int attitude_estimator_so3_comp_task;				/**< Handle of deamon task / thread */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
static bool bFilterInit = false;

//! Auxiliary variables to reduce number of repeated operations
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;

//! Serial packet related
static int uart;
static int baudrate;

/**
 * Mainloop of attitude_estimator_so3_comp.
 */
int attitude_estimator_so3_comp_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: attitude_estimator_so3_comp {start|stop|status} [-d <devicename>] [-b <baud rate>]\n"
		"-d and -b options are for separate visualization with raw data (quaternion packet) transfer\n"
		"ex) attitude_estimator_so3_comp start -d /dev/ttyS1 -b 115200\n");
	exit(1);
}

/**
 * The attitude_estimator_so3_comp app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int attitude_estimator_so3_comp_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("attitude_estimator_so3_comp already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		attitude_estimator_so3_comp_task = task_spawn("attitude_estimator_so3_comp",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_MAX - 5,
					      12400,
					      attitude_estimator_so3_comp_thread_main,
					      (const char **)argv);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;

		while(thread_running){
			usleep(200000);
			printf(".");
		}
		printf("terminated.");
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tattitude_estimator_so3_comp app is running\n");

		} else {
			printf("\tattitude_estimator_so3_comp app not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float number) {
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt) {
	float recipNorm;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

	//! Make filter converge to initial solution faster
	//! This function assumes you are in static position.
	//! WARNING : in case air reboot, this can cause problem. But this is very
	//!	      unlikely happen.
	if(bFilterInit == false)
	{
		NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz);
		bFilterInit = true;
	}
        	
	//! If magnetometer measurement is available, use it.
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		float hx, hy, hz, bx, bz;
		float halfwx, halfwy, halfwz;
	
		// Normalise magnetometer measurement
		// Will sqrt work better? PX4 system is powerful enough?
    		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    		mx *= recipNorm;
    		my *= recipNorm;
    		mz *= recipNorm;
    
    		// Reference direction of Earth's magnetic field
    		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 - q2q2);
    		bx = sqrt(hx * hx + hy * hy);
    		bz = hz;
    
    		// Estimated direction of magnetic field
    		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
    		// Error is sum of cross product between estimated direction and measured direction of field vectors
    		halfex += (my * halfwz - mz * halfwy);
    		halfey += (mz * halfwx - mx * halfwz);
    		halfez += (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float halfvx, halfvy, halfvz;
	
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += ay * halfvz - az * halfvy;
		halfey += az * halfvx - ax * halfvz;
		halfez += ax * halfvy - ay * halfvx;
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
			gyro_bias[1] += twoKi * halfey * dt;
			gyro_bias[2] += twoKi * halfez * dt;
			gx += gyro_bias[0];	// apply integral feedback
			gy += gyro_bias[1];
			gz += gyro_bias[2];
		}
		else {
			gyro_bias[0] = 0.0f;	// prevent integral windup
			gyro_bias[1] = 0.0f;
			gyro_bias[2] = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	//! Integrate rate of change of quaternion
#if 0
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
#endif 

	// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
	//! q_k = q_{k-1} + dt*\dot{q}
	//! \dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

	q0 += dt*dq0;
	q1 += dt*dq1;
	q2 += dt*dq2;
	q3 += dt*dq3;
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
       	q0q1 = q0 * q1;
       	q0q2 = q0 * q2;
       	q0q3 = q0 * q3;
       	q1q1 = q1 * q1;
       	q1q2 = q1 * q2;
   	q1q3 = q1 * q3;
       	q2q2 = q2 * q2;
       	q2q3 = q2 * q3;
       	q3q3 = q3 * q3;   
}

void send_uart_byte(char c)
{
	write(uart,&c,1);
}

void send_uart_bytes(uint8_t *data, int length)
{
	write(uart,data,(size_t)(sizeof(uint8_t)*length));
}

void send_uart_float(float f) {
  uint8_t * b = (uint8_t *) &f;

  //! Assume float is 4-bytes
  for(int i=0; i<4; i++) {
    
    uint8_t b1 = (b[i] >> 4) & 0x0f;
    uint8_t b2 = (b[i] & 0x0f);
    
    uint8_t c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    uint8_t c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    send_uart_bytes(&c1,1);
    send_uart_bytes(&c2,1);
  }
}

void send_uart_float_arr(float *arr, int length)
{
	for(int i=0;i<length;++i)
	{
		send_uart_float(arr[i]);
		send_uart_byte(',');
	}
}

int open_uart(int baud, const char *uart_name, struct termios *uart_config_original, bool *is_usb)
{
	int speed;
	
	switch (baud) {
        case 0:      speed = B0;      break;
        case 50:     speed = B50;     break;
        case 75:     speed = B75;     break;
        case 110:    speed = B110;    break;
        case 134:    speed = B134;    break;
        case 150:    speed = B150;    break;
        case 200:    speed = B200;    break;
        case 300:    speed = B300;    break;
        case 600:    speed = B600;    break;
        case 1200:   speed = B1200;   break;
        case 1800:   speed = B1800;   break;
        case 2400:   speed = B2400;   break;
        case 4800:   speed = B4800;   break;
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:
                printf("ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\n\t9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600\n\n", baud);
                return -EINVAL;
        }

	printf("[so3_comp_filt] UART is %s, baudrate is %d\n", uart_name, baud);
        uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
        struct termios uart_config;
        int termios_state;
        *is_usb = false;

	/* make some wild guesses including that USB serial is indicated by either /dev/ttyACM0 or /dev/console */
        if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/console") != OK) {
                /* Back up the original uart configuration to restore it after exit */
                if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
                        printf("ERROR getting baudrate / termios config for %s: %d\n", uart_name, termios_state);
                        close(uart);
                        return -1;
                }

                /* Fill the struct for the new configuration */
                tcgetattr(uart, &uart_config);

                /* Clear ONLCR flag (which appends a CR for every LF) */
                uart_config.c_oflag &= ~ONLCR;

                /* Set baud rate */
                if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
                        printf("ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
                        close(uart);
                        return -1;
                }


                if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
                        printf("ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
                        close(uart);
                        return -1;
                }

        } else {
                *is_usb = true;
        }

        return uart;
}

/*
 * [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
 */

/*
 * EKF Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
int attitude_estimator_so3_comp_thread_main(int argc, char *argv[])
{

const unsigned int loop_interval_alarm = 6500;	// loop interval in microseconds

	//! Serial debug related
	int ch;
	struct termios uart_config_original;
	bool usb_uart;
	bool debug_mode = false;
	char *device_name = "/dev/ttyS2";
	baudrate = 115200;

	//! Time constant
	float dt = 0.005f;
	
	/* output euler angles */
	float euler[3] = {0.0f, 0.0f, 0.0f};

	float Rot_matrix[9] = {1.f,  0,  0,
			      0,  1.f,  0,
			      0,  0,  1.f
			     };		/**< init: identity matrix */

	float acc[3] = {0.0f, 0.0f, 0.0f};
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	//! -d <device_name>, default : /dev/ttyS2
	//! -b <baud_rate>,   default : 115200
	while ((ch = getopt(argc,argv,"d:b:")) != EOF){
		switch(ch){
			case 'b':
				baudrate = strtoul(optarg, NULL, 10);
				if(baudrate == 0)
					printf("invalid baud rate '%s'",optarg);
				break;
			case 'd':
				device_name = optarg;
				debug_mode = true;
				break;
			default:
				usage("invalid argument");
		}
	}

	if(debug_mode){
		printf("Opening debugging port for 3D visualization\n");
		uart = open_uart(baudrate, device_name, &uart_config_original, &usb_uart);
		if (uart < 0)
                	printf("could not open %s", device_name);
		else
			printf("Open port success\n");
	}

	// print text
	printf("Nonlinear SO3 Attitude Estimator initialized..\n\n");
	fflush(stdout);

	int overloadcounter = 19;

	/* store start time to guard against too slow update rates */
	uint64_t last_run = hrt_absolute_time();

	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));

	//! Initialize attitude vehicle uORB message.
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	struct vehicle_status_s state;
	memset(&state, 0, sizeof(state));

	uint64_t last_data = 0;
	uint64_t last_measurement = 0;

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	/* rate-limit raw data updates to 200Hz */
	orb_set_interval(sub_raw, 4);

	/* subscribe to param changes */
	int sub_params = orb_subscribe(ORB_ID(parameter_update));

	/* subscribe to system state*/
	int sub_state = orb_subscribe(ORB_ID(vehicle_status));

	/* advertise attitude */
	orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);

	int loopcounter = 0;
	int printcounter = 0;

	thread_running = true;

	/* advertise debug value */
	// struct debug_key_value_s dbg = { .key = "", .value = 0.0f };
	// orb_advert_t pub_dbg = -1;

	float sensor_update_hz[3] = {0.0f, 0.0f, 0.0f};
	// XXX write this out to perf regs

	/* keep track of sensor updates */
	uint32_t sensor_last_count[3] = {0, 0, 0};
	uint64_t sensor_last_timestamp[3] = {0, 0, 0};

	struct attitude_estimator_so3_comp_params so3_comp_params;
	struct attitude_estimator_so3_comp_param_handles so3_comp_param_handles;

	/* initialize parameter handles */
	parameters_init(&so3_comp_param_handles);

	uint64_t start_time = hrt_absolute_time();
	bool initialized = false;

	float gyro_offsets[3] = { 0.0f, 0.0f, 0.0f };
	unsigned offset_count = 0;

	/* register the perf counter */
	perf_counter_t so3_comp_loop_perf = perf_alloc(PC_ELAPSED, "attitude_estimator_so3_comp");

	/* Main loop*/
	while (!thread_should_exit) {

		struct pollfd fds[2];
		fds[0].fd = sub_raw;
		fds[0].events = POLLIN;
		fds[1].fd = sub_params;
		fds[1].events = POLLIN;
		int ret = poll(fds, 2, 1000);

		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else if (ret == 0) {
			/* check if we're in HIL - not getting sensor data is fine then */
			orb_copy(ORB_ID(vehicle_status), sub_state, &state);

			if (!state.flag_hil_enabled) {
				fprintf(stderr,
					"[att so3_comp] WARNING: Not getting sensors - sensor app running?\n");
			}

		} else {

			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);

				/* update parameters */
				parameters_update(&so3_comp_param_handles, &so3_comp_params);
			}

			/* only run filter if sensor values changed */
			if (fds[0].revents & POLLIN) {

				/* get latest measurements */
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);

				if (!initialized) {

					gyro_offsets[0] += raw.gyro_rad_s[0];
					gyro_offsets[1] += raw.gyro_rad_s[1];
					gyro_offsets[2] += raw.gyro_rad_s[2];
					offset_count++;

					if (hrt_absolute_time() - start_time > 3000000LL) {
						initialized = true;
						gyro_offsets[0] /= offset_count;
						gyro_offsets[1] /= offset_count;
						gyro_offsets[2] /= offset_count;
					}

				} else {

					perf_begin(so3_comp_loop_perf);

					/* Calculate data time difference in seconds */
					dt = (raw.timestamp - last_measurement) / 1000000.0f;
					last_measurement = raw.timestamp;
					uint8_t update_vect[3] = {0, 0, 0};

					/* Fill in gyro measurements */
					if (sensor_last_count[0] != raw.gyro_counter) {
						update_vect[0] = 1;
						sensor_last_count[0] = raw.gyro_counter;
						sensor_update_hz[0] = 1e6f / (raw.timestamp - sensor_last_timestamp[0]);
						sensor_last_timestamp[0] = raw.timestamp;
					}

					gyro[0] =  raw.gyro_rad_s[0] - gyro_offsets[0];
					gyro[1] =  raw.gyro_rad_s[1] - gyro_offsets[1];
					gyro[2] =  raw.gyro_rad_s[2] - gyro_offsets[2];

					/* update accelerometer measurements */
					if (sensor_last_count[1] != raw.accelerometer_counter) {
						update_vect[1] = 1;
						sensor_last_count[1] = raw.accelerometer_counter;
						sensor_update_hz[1] = 1e6f / (raw.timestamp - sensor_last_timestamp[1]);
						sensor_last_timestamp[1] = raw.timestamp;
					}

					acc[0] = raw.accelerometer_m_s2[0];
					acc[1] = raw.accelerometer_m_s2[1];
					acc[2] = raw.accelerometer_m_s2[2];

					/* update magnetometer measurements */
					if (sensor_last_count[2] != raw.magnetometer_counter) {
						update_vect[2] = 1;
						sensor_last_count[2] = raw.magnetometer_counter;
						sensor_update_hz[2] = 1e6f / (raw.timestamp - sensor_last_timestamp[2]);
						sensor_last_timestamp[2] = raw.timestamp;
					}

					mag[0] = raw.magnetometer_ga[0];
					mag[1] = raw.magnetometer_ga[1];
					mag[2] = raw.magnetometer_ga[2];

					uint64_t now = hrt_absolute_time();
					unsigned int time_elapsed = now - last_run;
					last_run = now;

					if (time_elapsed > loop_interval_alarm) {
						//TODO: add warning, cpu overload here
						// if (overloadcounter == 20) {
						// 	printf("CPU OVERLOAD DETECTED IN ATTITUDE ESTIMATOR EKF (%lu > %lu)\n", time_elapsed, loop_interval_alarm);
						// 	overloadcounter = 0;
						// }

						overloadcounter++;
					}

					static bool const_initialized = false;

					/* initialize with good values once we have a reasonable dt estimate */
					if (!const_initialized && dt < 0.05f && dt > 0.005f) {
						dt = 0.005f;
						parameters_update(&so3_comp_param_handles, &so3_comp_params);
						const_initialized = true;
					}

					/* do not execute the filter if not initialized */
					if (!const_initialized) {
						continue;
					}

					uint64_t timing_start = hrt_absolute_time();

					// NOTE : Accelerometer is reversed.
					// Because proper mount of PX4 will give you a reversed accelerometer readings.
					NonlinearSO3AHRSupdate(gyro[0],gyro[1],gyro[2],-acc[0],-acc[1],-acc[2],mag[0],mag[1],mag[2],so3_comp_params.Kp,so3_comp_params.Ki, dt);

					// Convert q->R.
					Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
        				Rot_matrix[1] = 2.0 * (q1*q2 + q0*q3);	// 12
        				Rot_matrix[2] = 2.0 * (q1*q3 - q0*q2);	// 13
        				Rot_matrix[3] = 2.0 * (q1*q2 - q0*q3);	// 21
        				Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
        				Rot_matrix[5] = 2.0 * (q2*q3 + q0*q1);	// 23
        				Rot_matrix[6] = 2.0 * (q1*q3 + q0*q2);	// 31
        				Rot_matrix[7] = 2.0 * (q2*q3 - q0*q1);	// 32
        				Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

					//1-2-3 Representation.
					//Equation (290) 
					//Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
					// Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
					euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	//! Roll
					euler[1] = -asinf(Rot_matrix[2]);	//! Pitch
					euler[2] = atan2f(Rot_matrix[1],Rot_matrix[0]);		//! Yaw
					
					/* swap values for next iteration, check for fatal inputs */
					if (isfinite(euler[0]) && isfinite(euler[1]) && isfinite(euler[2])) {
						/* Do something */
					} else {
						/* due to inputs or numerical failure the output is invalid, skip it */
						continue;
					}

					if (last_data > 0 && raw.timestamp - last_data > 12000) printf("[attitude estimator so3_comp] sensor data missed! (%llu)\n", raw.timestamp - last_data);

					last_data = raw.timestamp;

					/* send out */
					att.timestamp = raw.timestamp;

					// XXX Apply the same transformation to the rotation matrix
					att.roll = euler[0] - so3_comp_params.roll_off;
					att.pitch = euler[1] - so3_comp_params.pitch_off;
					att.yaw = euler[2] - so3_comp_params.yaw_off;

					//! Euler angle rate. But it needs to be investigated again.
					/*
					att.rollspeed = 2.0f*(-q1*dq0 + q0*dq1 - q3*dq2 + q2*dq3);
					att.pitchspeed = 2.0f*(-q2*dq0 + q3*dq1 + q0*dq2 - q1*dq3);
					att.yawspeed = 2.0f*(-q3*dq0 -q2*dq1 + q1*dq2 + q0*dq3);
					*/
					att.rollspeed = gyro[0];
					att.pitchspeed = gyro[1];
					att.yawspeed = gyro[2];

					att.rollacc = 0;
					att.pitchacc = 0;
					att.yawacc = 0;

					//! Quaternion
					att.q[0] = q0;
					att.q[1] = q1;
					att.q[2] = q2;
					att.q[3] = q3;
					att.q_valid = true;

					/* TODO: Bias estimation required */
					memcpy(&att.rate_offsets, &(gyro_bias), sizeof(att.rate_offsets));

					/* copy rotation matrix */
					memcpy(&att.R, Rot_matrix, sizeof(Rot_matrix));
					att.R_valid = true;

					if (isfinite(att.roll) && isfinite(att.pitch) && isfinite(att.yaw)) {
						// Broadcast
						orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);

					} else {
						warnx("NaN in roll/pitch/yaw estimate!");
					}

					perf_end(so3_comp_loop_perf);

					//! This will print out debug packet to visualization software
					if(debug_mode)
					{
						float quat[4];
						quat[0] = q0;
						quat[1] = q1;
						quat[2] = q2;
						quat[3] = q3;
						send_uart_float_arr(quat,4);
						send_uart_byte('\n');
					}
				}
			}
		}

		loopcounter++;
	}// while

	thread_running = false;

	/* Reset the UART flags to original state */
        if (!usb_uart)
                tcsetattr(uart, TCSANOW, &uart_config_original);

	return 0;
}
