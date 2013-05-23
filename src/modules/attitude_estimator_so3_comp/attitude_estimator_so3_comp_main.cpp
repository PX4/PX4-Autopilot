/*
 * @file attitude_estimator_so3_comp_main.c
 *
 * Nonlinear SO3 filter for Attitude Estimation.
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
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
static float gravity_vector[3] = {0.0f,0.0f,0.0f};	/** estimated gravity vector */

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

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
        	
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

	//! If magnetometer measurement is available, use it.
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		float hx, hy, bx, bz;
		float halfwx, halfwy, halfwz;
	
		// Normalise magnetometer measurement
    		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    		mx *= recipNorm;
    		my *= recipNorm;
    		mz *= recipNorm;
    
    		// Reference direction of Earth's magnetic field
    		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    		bx = sqrt(hx * hx + hy * hy);
    		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    
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
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	q0 +=(-q1 * gx - q2 * gy - q3 * gz);
	q1 += (q0 * gx + q2 * gz - q3 * gy);
	q2 += (q0 * gy - q1 * gz + q3 * gx);
	q3 += (q0 * gz + q1 * gy - q2 * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
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

					MahonyAHRSupdate(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2],mag[0],mag[1],mag[2],so3_comp_params.Kp,so3_comp_params.Ki, dt);

					float aSq = q0*q0; // 1
					float bSq = q1*q1; // 2
					float cSq = q2*q2; // 3
					float dSq = q3*q3; // 4
					float a = q0;
					float b = q1;
					float c = q2;
					float d = q3;

					Rot_matrix[0] = 2*aSq - 1 + 2*bSq;	// 11
        				//Rot_matrix[1] = 2.0 * (b * c - a * d);	// 12
        				//Rot_matrix[2] = 2.0 * (a * c + b * d);	// 13
        				Rot_matrix[3] = 2.0 * (b * c - a * d);	// 21
        				//Rot_matrix[4] = aSq - bSq + cSq - dSq;	// 22
        				//Rot_matrix[5] = 2.0 * (c * d - a * b);	// 23
        				Rot_matrix[6] = 2.0 * (b * d + a * c);	// 31
        				Rot_matrix[7] = 2.0 * (c * d - a * b);	// 32
        				Rot_matrix[8] = 2*aSq - 1 + 2*dSq;	// 33

					gravity_vector[0] = 2*(q1*q3-q0*q2);
					gravity_vector[1] = 2*(q0*q1+q2*q3);
					gravity_vector[2] = aSq - bSq - cSq + dSq;

					//euler[0] = atan2f(Rot_matrix[7], Rot_matrix[8]);
					//euler[1] = -asinf(Rot_matrix[6]);
					//euler[2] = atan2f(Rot_matrix[3],Rot_matrix[0]);

					// Euler angle directly from quaternion.
					euler[0] = -atan2f(gravity_vector[1], sqrtf(gravity_vector[0]*gravity_vector[0] + gravity_vector[2]*gravity_vector[2]));	// roll
					euler[1] = atan2f(gravity_vector[0], sqrtf(gravity_vector[1]*gravity_vector[1] + gravity_vector[2]*gravity_vector[2]));	// pitch
					euler[2] = -atan2f(2*(q1*q2-q0*q3),2*(q0*q0+q1*q1)-1);	// yaw
				
					//euler[2] = atan2(2 * q1*q2 - 2 * q0*q3, 2 * q0*q0 + 2 * q1*q1 - 1); // psi
  					//euler[1] = -asin(2 * q1*q3 + 2 * q0*q2); // theta
					//euler[0] = atan2(2 * q2*q3 - 2 * q0*q1, 2 * q0*q0 + 2 * q3*q3 - 1); // phi	

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

					/* FIXME : This can be a problem for rate controller. Rate in body or inertial? */
					att.rollspeed = gyro[0];
					att.pitchspeed = gyro[1];
					att.yawspeed = gyro[2];
					att.rollacc = 0;
					att.pitchacc = 0;
					att.yawacc = 0;

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
