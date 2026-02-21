#ifndef VIO_DATA_T_H
#define VIO_DATA_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// VIO
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Common interface for vio server-client communication
 *
 *             VINS and QVIO servers may offer different algorithm-specific
 *             interfaces but should also offer this common generic interface
 *             for simple applications.
 *
 *             The variable names in the data structure for describing
 *             translations and rotations adhere to the following naming
 *             convention for clarity and readability.
 *
 *             Translations are written as T_A_wrt_B meaning the translation
 *             (position) of the origin of frame A with respect to frame B.
 *             Rotations are written as R_A_to_B meaning a rotation matrix which
 *             rotates a vector in the coordinate frame of A to be represented
 *             as a vector in coordinate frame B.
 *
 *             The following coordinate frames are used:
 *
 *             VIO FRAME: Centered wherever the IMU was when VIO started and
 *             aligned with wherever the IMU was VIO started. This is NOT
 *             aligned with gravity and the user must make use of the provided
 *             gravity vector if they wish to align the VIO data to gravity.
 *
 *             IMU FRAME: Centered about IMU and aligned with the VOXL “common”
 *             IMU coordinate frame, NOT to the imu frame from any one specific
 *             chip since VOXL has multiple imus. T_imu_wrt_VIO and R_imu_to_vio
 *             describe the position and rotation of the IMU frame with respect
 *             to VIO frame. For a diagram explaining the VOXL “common” imu
 *             frame see https://docs.modalai.com/camera-imu-coordinate-frames/
 *
 *             CAMERA FRAME: Standard open-cv camera frame with X right, Y down,
 *             and Z out the lens. This relation is initially read from a config
 *             file but is optimized further in real time by the VIO algorithm
 *             allowing the user to make use of better estimation of the camera
 *             mounting angle to compensate for manufacturing tolerances.
 *
 *             VIO GRAVITY ALIGNED FRAME: Centered wherever VIO started with yaw
 *             aligned to IMU but roll/pitch aligned with gravity. VIO FRAME
 *             does not align with Z along gravity but with wherever the IMU was
 *             on initialization. If you application requires alignment with
 *             gravity then you can follow the example here:
 *             https://gitlab.com/voxl-public/voxl-vision-px4/-/blob/78feacef924ce64e4e1c4c3f3a8ae14d1b38630b/src/geometry.c#L321
 */

/**
 * The overall status of the VIO algorithm is described with three different
 * fields. The primary ‘state’ field is used to check the overall state of the
 * algorithm. The ‘state’ field can be VIO_STATE_FAILED, VIO_STATE_INITIALIZING,
 * or VIO_STATE_OK.
 *
 * When ‘state’ reports VIO_STATE_FAILED, the error_code field is set. Similarly
 * to posix errno, the ‘error_code’ contains the last thrown internal error. If
 * state==VIO_STATE_OK then the value contained in ‘error_code’ is undefined and
 * can be ignored.
 *
 * While the algorithm is running normally and state==VIO_STATE_OK then the user
 * can monitor the overall quality of the position estimate with the ‘quality’
 * field. This is a unitless measure of the VIO quality that is always positive
 * and increases with more accurate pose estimates. This is mostly derived from
 * the covariance matrix but also takes into account the number of features
 * being tracked and uncertainties in the algorithm’s behavior while
 * initializing.
 */
#define VIO_STATE_FAILED 0
#define VIO_STATE_INITIALIZING 1
#define VIO_STATE_OK 2

// codes 0-15 match mvVISLAM codes, don't change those!!
#define ERROR_CODE_COVARIANCE (1 << 0)       // Reset: cov not pos definite
#define ERROR_CODE_IMU_OOB (1 << 1)          // Reset: IMU exceeded range (out of bounds)
#define ERROR_CODE_IMU_BW (1 << 2)           // Reset: IMU bandwidth too low
#define ERROR_CODE_NOT_STATIONARY (1 << 3)   // Reset: not stationary at initialization
#define ERROR_CODE_NO_FEATURES (1 << 4)      // Reset: no features for x seconds
#define ERROR_CODE_CONSTRAINT (1 << 5)       // Reset: insufficient constraints from features
#define ERROR_CODE_FEATURE_ADD (1 << 6)      // Reset: failed to add new features
#define ERROR_CODE_VEL_INST_CERT (1 << 7)    // Reset: exceeded instant velocity uncertainty
#define ERROR_CODE_VEL_WINDOW_CERT (1 << 8)  // Reset: exceeded velocity uncertainty
#define ERROR_CODE_DROPPED_IMU (1 << 10)     // Dropped IMU samples
#define ERROR_CODE_BAD_CAM_CAL (1 << 11)     // Intrinsic camera cal questionable
#define ERROR_CODE_LOW_FEATURES (1 << 12)    // Insufficient good features to initialize
#define ERROR_CODE_DROPPED_CAM (1 << 13)     // Dropped camera frame
#define ERROR_CODE_DROPPED_GPS_VEL (1 << 14) // Dropped GPS velocity sample
#define ERROR_CODE_BAD_TIMESTAMP (1 << 15)   // Sensor measurements with bad time stamps
#define ERROR_CODE_IMU_MISSING (1 << 16)     // Missing IMU data
#define ERROR_CODE_CAM_MISSING (1 << 17)     // Missing camera frames
#define ERROR_CODE_CAM_BAD_RES (1 << 18)     // camera resolution unsupported
#define ERROR_CODE_CAM_BAD_FORMAT (1 << 19)  // camera format unsupported
#define ERROR_CODE_UNKNOWN (1 << 20)         // Unknown error
#define ERROR_CODE_STALLED (1 << 21)         // frame processing stalled

/**
 * frames of reference for VIO data
 *
 * VIO_FRAME_IMU_UNALIGNED is what QC MVVISLAM publiblishes. Its origin is
 * located at and aligned with the IMU on init. It is NOT aligned with gravity,
 * or with the body of the drone. This is what qvio used to publish (<=v1.0.5)
 * and required voxl-vision-hub to look up how the imu was mounted in the drone
 * and rotate based on this and the gravity vector.
 *
 * VIO_FRAME_BODY_ALIGNED is what voxl-openvins-server and voxl-qvio-server
 * (>=1.1.0) publish. the origin is still centered about the IMU but is aligned
 * with gravity in the FRD coordinate frame. The reason for this is that openvins
 * naturally tries to align its output with gravity so there is no sense in
 * rotating it back to an unaligned IMU frame to mimic MVVISLAM. Therefore in
 * November 2024 we migrate both voxl-qvio-server and voxl-openvins-server to
 * publishing gravity aligned VIO data and voxl-vision-hub uses the "frame" field
 * of the vio data struct to handle both cases accordingly.
 */
#define VIO_FRAME_UNKNOWN 0
#define VIO_FRAME_IMU_UNALIGNED 1
#define VIO_FRAME_BODY_ALIGNED 2

/**
 * The following commands can be sent to the VIO server over its control pipe.
 * Commanding a reset will force the algorithm to reinitialize. This will take
 * an indeterminant amount of time and may require the IMU to detect a
 * stationary position before the reset completes if the server is configured to
 * only initialize when stationary. A hard reset will go back to reporting a
 * position of 0,0,0 after reinitializing. A soft reset will try to continue
 * reporting the last estiamted position after reset.
 *
 * More commands may be added to expand functionality in the future
 */
#define RESET_VIO_SOFT "reset_vio_soft"
#define RESET_VIO_HARD "reset_vio_hard"

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 *
 */
#include "magic_number.h"
#ifndef VIO_MAGIC_NUMBER
#error "VIO_MAGIC_NUMBER not defined!"
#endif

/**
 * This is the data structure that vio servers should make available to clients
 * on the data pipe.
 *
 * totals 324 bytes
 *
 *
 *  covariance matrix entries defined as follows:
    uint8 COVARIANCE_MATRIX_X_VARIANCE=0
    uint8 COVARIANCE_MATRIX_Y_VARIANCE=6
    uint8 COVARIANCE_MATRIX_Z_VARIANCE=11
    uint8 COVARIANCE_MATRIX_ROLL_VARIANCE=15
    uint8 COVARIANCE_MATRIX_PITCH_VARIANCE=18
    uint8 COVARIANCE_MATRIX_YAW_VARIANCE=20
    uint8 COVARIANCE_MATRIX_VX_VARIANCE=0
    uint8 COVARIANCE_MATRIX_VY_VARIANCE=6
    uint8 COVARIANCE_MATRIX_VZ_VARIANCE=11
    uint8 COVARIANCE_MATRIX_ROLLRATE_VARIANCE=15
    uint8 COVARIANCE_MATRIX_PITCHRATE_VARIANCE=18
    uint8 COVARIANCE_MATRIX_YAWRATE_VARIANCE=20
 */
typedef struct vio_data_t
{
    uint32_t magic_number;         ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
    int32_t quality;               ///< Quality is be >0 in normal use with a larger number indicating higher quality. A positive quality does not guarantee the algorithm has initialized completely.
    int64_t timestamp_ns;          ///< Timestamp in clock_monotonic system time of the provided pose.
    float T_imu_wrt_vio[3];        ///< Translation of the IMU with respect to VIO frame in meters.
    float R_imu_to_vio[3][3];      ///< Rotation matrix from IMU to VIO frame.
    float pose_covariance[21];     ///<  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
    float vel_imu_wrt_vio[3];      ///< Velocity of the imu with respect to the VIO frame.
    float velocity_covariance[21]; ///<  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
    float imu_angular_vel[3];      ///< Angular velocity of the IMU about its X Y and Z axes respectively. Essentially filtered gyro values with internal biases applied.
    float gravity_vector[3];       ///< Estimation of the current gravity vector in VIO frame. Use this to estimate the rotation between VIO frame and a gravity-aligned VIO frame if desired.
    float T_cam_wrt_imu[3];        ///< Location of the optical center of the camera with respect to the IMU.
    float R_cam_to_imu[3][3];      ///< Rotation matrix from camera frame to IMU frame.
    uint32_t error_code;           ///< bitmask that can indicate multiple errors. may still contain errors if state==VIO_STATE_OK
    uint16_t n_feature_points;     ///< Number of optical feature points currently being tracked.
    uint8_t state;                 ///< This is used to check the overall state of the algorithm. Can be VIO_STATE_FAILED, VIO_STATE_INITIALIZING, or VIO_STATE_OK.
    uint8_t frame;                 ///< Frame of reference
} __attribute__((packed)) vio_data_t;

/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 26 packets which is
 * perhaps more than necessary but only takes a little under 1 page of memory
 * which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define VIO_RECOMMENDED_READ_BUF_SIZE (sizeof(vio_data_t) * 26)

/**
 * We recommend VIO servers use a 64k pipe size. This means every client would
 * get their own buffer of 14 seconds of VIO data at 30hz. Clients can
 * increase this buffer if they wish. voxl-qvio-server uses this as its default.
 * 64K is also the Linux Kernel default pipe size.
 */
#define VIO_RECOMMENDED_PIPE_SIZE (64 * 1024)

/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a vio_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a vio_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an vio_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
vio_data_t *pipe_validate_vio_data_t(char *data, int bytes, int *n_packets);

// max number of features reported by the extended data struct. More features
// may be available. TODO output a full point cloud.
#define VIO_MAX_REPORTED_FEATURES 64

typedef enum vio_point_quality_t
{
    LOW,    ///< additional low-quality points collected for e.g. collision avoidance
    MEDIUM, ///< Points that are not "in state"
    HIGH    ///< Points that are "in state"
} vio_point_quality_t;

typedef struct vio_feature_t
{
    uint32_t id;              ///< unique ID for feature point
    int32_t cam_id;           ///< ID of camera which the point was seen from (typically first)
    float pix_loc[2];         ///< pixel location in the last frame
    float tsf[3];             ///< location of feature in vio frame (relative to init location)
    float p_tsf[3][3];        ///< covarience of feature location
    float depth;              ///< distance from camera to point
    float depth_error_stddev; ///< depth error in meters
    vio_point_quality_t point_quality;
} vio_feature_t;




/**
 * This is the extended version of the vio_data_t struct, adding in extra debug fields
 * along with feature locations + quality.
 *
 * totals 5268 bytes
 */
typedef struct ext_vio_data_t
{
    vio_data_t v;
    int32_t last_cam_frame_id;
    int64_t last_cam_timestamp_ns;
    float imu_cam_time_shift_s;
    float gravity_covariance[3][3];
    float gyro_bias[3];
    float accl_bias[3];
    uint32_t n_total_features; ///< total features, in-state and out-of-state listed in the following array
    vio_feature_t features[VIO_MAX_REPORTED_FEATURES];
} __attribute__((packed)) ext_vio_data_t;

/**
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define EXT_VIO_RECOMMENDED_READ_BUF_SIZE (sizeof(ext_vio_data_t) * 10)

/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a vio_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a vio_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an vio_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
ext_vio_data_t *pipe_validate_ext_vio_data_t(char *data, int bytes, int *n_packets);

/**
 * @brief      print a human-readable string representation of a VIO state to
 *             stdout
 *
 * @param[in]  s     state to print
 */
void pipe_print_vio_state(int s);

/**
 * @brief      print a human-readable string representation of a VIO error code
 *             to stdout
 *
 * @param[in]  e     error to print
 */
void pipe_print_vio_error(uint32_t e);

/**
 * @brief      populates a user-allocated buffer with a human-readable string
 *             representing a provided VIO Error Code e.
 *
 *             e is a bitfield that may contain many errors, all of which will
 *             be printed up to the specified length of the buffer. We suggest a
 *             buffer length of about 256, but you can go above or below
 *             depending on your application. We do not allow anything smaller
 *             than 20 since that's pretty useless.
 *
 * @param[in]  e        VIO error code
 * @param      str      user-allocated buffer
 * @param[in]  buf_len  The buffer length (must be >= 20)
 *
 * @return     0 on success, -1 on failure
 */
int pipe_construct_vio_error_string(uint32_t e, char *str, size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif
