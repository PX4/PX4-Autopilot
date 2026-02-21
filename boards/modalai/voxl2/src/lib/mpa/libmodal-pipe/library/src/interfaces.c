/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/



#include <stdio.h>
#include <string.h>

#include <modal_pipe_interfaces.h>
#include <modal_pipe_deprecated.h>

// most of these validators are about the same, clean this up with a macro
#define DEFINE_PIPE_VALIDATOR(FUNC_NAME, TYPE, MAGIC_CONST)                  \
TYPE* FUNC_NAME(char* data, int bytes, int* n_packets)                              \
{                                                                                   \
    TYPE* new_ptr = (TYPE*) data;                                                   \
    *n_packets = 0;                                                                 \
                                                                                    \
    if (bytes < 0) {                                                                 \
        fprintf(stderr, "ERROR validating %s data: number of bytes = %d\n", #TYPE, bytes); \
        return NULL;                                                                \
    }                                                                               \
    if (data == NULL) {                                                             \
        fprintf(stderr, "ERROR validating %s data: got NULL data pointer\n", #TYPE); \
        return NULL;                                                                \
    }                                                                               \
    if (bytes % sizeof(TYPE)) {                                                     \
        fprintf(stderr, "ERROR validating %s data: read partial packet\n", #TYPE);  \
        fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n",      \
                bytes, sizeof(TYPE));                                               \
        return NULL;                                                                \
    }                                                                               \
                                                                                    \
    int n_packets_tmp = bytes / sizeof(TYPE);                                       \
    int i, n_failed = 0;                                                            \
    for (i = 0; i < n_packets_tmp; i++) {                                           \
        if (new_ptr[i].magic_number != MAGIC_CONST) n_failed++;                    \
    }                                                                               \
    if (n_failed > 0) {                                                             \
        fprintf(stderr, "ERROR validating %s data: %d of %d packets failed\n",      \
                #TYPE, n_failed, n_packets_tmp);                                    \
        return NULL;                                                                \
    }                                                                               \
                                                                                    \
    *n_packets = n_packets_tmp;                                                     \
    return new_ptr;                                                                 \
}




DEFINE_PIPE_VALIDATOR(pipe_validate_tag_detection_t, tag_detection_t, TAG_DETECTION_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_tof2_data_t, tof2_data_t, TOF2_MAGIC_NUMBER)

// DEPRECATED
DEFINE_PIPE_VALIDATOR(pipe_validate_tof_data_t, tof_data_t, TOF_MAGIC_NUMBER)

DEFINE_PIPE_VALIDATOR(pipe_validate_rc_channels_data_t, rc_channels_data_t, RC_CHANNELS_MAGIC_NUMBER);
DEFINE_PIPE_VALIDATOR(pipe_validate_baro_data_t, baro_data_t, BARO_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_crsf_raw_data_t, crsf_raw_data_t, CRSF_RAW_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_imu_data_t, imu_data_t, IMU_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_pose_4dof_t, pose_4dof_t, POSE_4DOF_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_pose_vel_6dof_t, pose_vel_6dof_t, POSE_VEL_6DOF_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_pose_vel_6dof2_t, pose_vel_6dof2_t, POSE_VEL_6DOF2_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_vio_data_t, vio_data_t, VIO_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_vfc_data_t, vfc_data_t, VFC_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_cpu_stats2_t, cpu_stats2_t, CPU_STATS2_MAGIC_NUMBER)
DEFINE_PIPE_VALIDATOR(pipe_validate_ot_data_t, object_tracking_data_t, OT_MAGIC_NUMBER)



const char* pipe_tag_location_type_to_string(int i)
{
    switch(i){
        case TAG_LOCATION_UNKNOWN:
            return "unknown";
        case TAG_LOCATION_FIXED:
            return "fixed";
        case TAG_LOCATION_STATIC:
            return "static";
        case TAG_LOCATION_DYNAMIC:
            return "dynamic";
        default:
            return "unknown";
    }
}

const char* pipe_image_format_to_string(int i)
{
    switch(i){
        case IMAGE_FORMAT_RAW8:
            return "RAW8";
        case IMAGE_FORMAT_NV12:
            return "NV12";
        case IMAGE_FORMAT_STEREO_RAW8:
            return "STEREO_RAW8";
        case IMAGE_FORMAT_H264:
            return "H264";
        case IMAGE_FORMAT_H265:
            return "H265";
        case IMAGE_FORMAT_RAW16:
            return "RAW16";
        case IMAGE_FORMAT_NV21:
            return "NV21";
        case IMAGE_FORMAT_JPG:
            return "JPG";
        case IMAGE_FORMAT_YUV422:
            return "YUV422";
        case IMAGE_FORMAT_YUV420:
            return "YUV420";
        case IMAGE_FORMAT_RGB:
            return "RGB";
        case IMAGE_FORMAT_FLOAT32:
            return "FLOAT32";
        case IMAGE_FORMAT_STEREO_NV21:
            return "STEREO_NV21";
        case IMAGE_FORMAT_STEREO_RGB :
            return "STEREO_RGB";
        case IMAGE_FORMAT_YUV422_UYVY:
            return "YUV422_UYVY";
        case IMAGE_FORMAT_STEREO_NV12:
            return "STEREO_NV12";
        case IMAGE_FORMAT_RAW10:
            return "RAW10";
        case IMAGE_FORMAT_RAW12:
            return "RAW12";
        case IMAGE_FORMAT_RAW14:
            return "RAW14";
        default:
            return "Unknown Format";
    }
}



const char* pipe_point_cloud_format_to_string(int i)
{
    switch(i){
        case POINT_CLOUD_FORMAT_FLOAT_XYZ:
            return "Float XYZ";
        case POINT_CLOUD_FORMAT_FLOAT_XYZC:
            return "Float XYZC";
        case POINT_CLOUD_FORMAT_FLOAT_XYZRGB:
            return "Float XYZ, Int RGB";
        case POINT_CLOUD_FORMAT_FLOAT_XYZCRGB:
            return "Float XYZC, Int RGB";
        case POINT_CLOUD_FORMAT_FLOAT_XY:
            return "Float XY";
        case POINT_CLOUD_FORMAT_FLOAT_XYC:
            return "Float XYC";
        default:
            return "Unknown Format";
    }
}

int pipe_point_cloud_meta_to_size_bytes(point_cloud_metadata_t meta)
{
    switch( meta.format ){
        case POINT_CLOUD_FORMAT_FLOAT_XYZ:
            return meta.n_points * 3 * sizeof(float);

        case POINT_CLOUD_FORMAT_FLOAT_XYZC:
            return meta.n_points * 4 * sizeof(float);

        case POINT_CLOUD_FORMAT_FLOAT_XYZRGB:
            return meta.n_points * ((3 * sizeof(float)) + 3);

        case POINT_CLOUD_FORMAT_FLOAT_XYZCRGB:
            return meta.n_points * ((4 * sizeof(float)) + 3);

        case POINT_CLOUD_FORMAT_FLOAT_XY:
            return meta.n_points * 2 * sizeof(float);

        case POINT_CLOUD_FORMAT_FLOAT_XYC:
            return meta.n_points * 3 * sizeof(float);

    }

    fprintf(stderr, "ERROR in %s, invalid point cloud format: %d\n", __FUNCTION__, meta.format);
    return -1;
}


#ifdef EN_MAVLINK_SUPPORT // mavlink support is optional
mavlink_message_t* pipe_validate_mavlink_message_t(char* data, int bytes, int* n_packets)
{
    // cast raw data from buffer to an vio_data_t array so we can read data
    // without memcpy. Also write out packets read as 0 until we validate data.
    mavlink_message_t* new_ptr = (mavlink_message_t*) data;
    *n_packets = 0;

    // basic sanity checks
    if(bytes<0){
        fprintf(stderr, "ERROR validating mavlink_message_t data received through pipe: number of bytes = %d\n", bytes);
        return NULL;
    }
    if(data==NULL){
        fprintf(stderr, "ERROR validating mavlink_message_t data received through pipe: got NULL data pointer\n");
        return NULL;
    }
    if(bytes%sizeof(mavlink_message_t)){
        fprintf(stderr, "ERROR validating mavlink_message_t data received through pipe: read partial packet\n");
        fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(mavlink_message_t));
        return NULL;
    }

    // calculate number of packets locally until we validate each packet
    int n_packets_tmp = bytes/sizeof(mavlink_message_t);

    // check if any packets failed the magic number check
    int i, n_failed = 0;
    int failed_magic = 0;
    for(i=0;i<n_packets_tmp;i++){
        if(new_ptr[i].magic != MAVLINK_STX && new_ptr[i].magic != MAVLINK_STX_MAVLINK1){
            n_failed++;
            failed_magic = new_ptr[i].magic;
        }
    }
    if(n_failed>0){
        fprintf(stderr, "ERROR validating mavlink_message_t data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
        fprintf(stderr, "last magic number received was %d, expected MAVLINK_STX=%d\n", failed_magic, MAVLINK_STX);
        return NULL;
    }

    // if we get here, all good. Write out the number of packets read and return
    // the new cast pointer. It's the same pointer the user provided but cast to
    // the right type for simplicity and easy of use.
    *n_packets = n_packets_tmp;
    return new_ptr;
}
#endif




ext_vio_data_t* pipe_validate_ext_vio_data_t(char* data, int bytes, int* n_packets){
    // cast raw data from buffer to an vio_data_t array so we can read data
    // without memcpy. Also write out packets read as 0 until we validate data.
    ext_vio_data_t* new_ptr = (ext_vio_data_t*) data;
    *n_packets = 0;

    // basic sanity checks
    if(bytes<0){
        fprintf(stderr, "ERROR validating VIO data received through pipe: number of bytes = %d\n", bytes);
        return NULL;
    }
    if(data==NULL){
        fprintf(stderr, "ERROR validating VIO data received through pipe: got NULL data pointer\n");
        return NULL;
    }
    if(bytes%sizeof(ext_vio_data_t)){
        fprintf(stderr, "ERROR validating EXT VIO data received through pipe: read partial packet\n");
        fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(ext_vio_data_t));
        return NULL;
    }

    // calculate number of packets locally until we validate each packet
    int n_packets_tmp = bytes/sizeof(ext_vio_data_t);

    // check if any packets failed the magic number check
    int i, n_failed = 0;
    for(i=0;i<n_packets_tmp;i++){
        if(new_ptr[i].v.magic_number != VIO_MAGIC_NUMBER) n_failed++;
    }
    if(n_failed>0){
        fprintf(stderr, "ERROR validating VIO data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
        return NULL;
    }

    // if we get here, all good. Write out the number of packets read and return
    // the new cast pointer. It's the same pointer the user provided but cast to
    // the right type for simplicity and easy of use.
    *n_packets = n_packets_tmp;
    return new_ptr;
}


void pipe_print_vio_state(int s)
{
    if     (s == VIO_STATE_FAILED)          printf("FAIL");
    else if(s == VIO_STATE_INITIALIZING)    printf("INIT");
    else if(s == VIO_STATE_OK)              printf("OKAY");
    else printf("UNKNOWN_VIO_STATE");
    return;
}


void pipe_print_vio_error(uint32_t e)
{
    char str[256];

    // shortcut for no error code
    if(e==0) return;

    if(pipe_construct_vio_error_string(e, str, 256)){
        return;
    }

    printf("%s", str);
    return;
}

int pipe_construct_vio_error_string(uint32_t e, char* str, size_t buf_len)
{
    if(str==NULL){
        fprintf(stderr, "ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }
    if(buf_len<20){
        fprintf(stderr, "ERROR in %s, buffer length must be >=20, ideally 64 to 256\n", __FUNCTION__);
        return -1;
    }

    // start with empty string
    str[0]=0;

    // shortcut for no error code
    if(e==0) return 0;

    if(e & ERROR_CODE_COVARIANCE)     strncat(str, "COV_ERROR ",        buf_len-strlen(str)-1);
    if(e & ERROR_CODE_IMU_OOB)        strncat(str, "IMU_OOB ",          buf_len-strlen(str)-1);
    if(e & ERROR_CODE_IMU_BW)         strncat(str, "IMU_BW ",           buf_len-strlen(str)-1);
    if(e & ERROR_CODE_NOT_STATIONARY) strncat(str, "NOT_STATIONARY ",   buf_len-strlen(str)-1);
    if(e & ERROR_CODE_NO_FEATURES)    strncat(str, "NO_FEATURES ",      buf_len-strlen(str)-1);
    if(e & ERROR_CODE_CONSTRAINT)     strncat(str, "CONSTRAINT_ERROR ", buf_len-strlen(str)-1);
    if(e & ERROR_CODE_FEATURE_ADD)    strncat(str, "FEATURE_ADD_ERROR ",buf_len-strlen(str)-1);
    if(e & ERROR_CODE_VEL_INST_CERT)  strncat(str, "VEL_INST_CERT ",    buf_len-strlen(str)-1);
    if(e & ERROR_CODE_VEL_WINDOW_CERT)strncat(str, "VEL_WINDOW_CERT ",  buf_len-strlen(str)-1);
    if(e & ERROR_CODE_DROPPED_IMU)    strncat(str, "DROPPED_IMU ",      buf_len-strlen(str)-1);
    if(e & ERROR_CODE_BAD_CAM_CAL)    strncat(str, "BAD_CAM_CAL ",      buf_len-strlen(str)-1);
    if(e & ERROR_CODE_LOW_FEATURES)   strncat(str, "LOW_FEATURES ",     buf_len-strlen(str)-1);
    if(e & ERROR_CODE_DROPPED_CAM)    strncat(str, "DROPPED_CAM ",      buf_len-strlen(str)-1);
    if(e & ERROR_CODE_DROPPED_GPS_VEL)strncat(str, "DROPPED_GPS_VEL ",  buf_len-strlen(str)-1);
    if(e & ERROR_CODE_BAD_TIMESTAMP)  strncat(str, "BAD_TIMESTAMP ",    buf_len-strlen(str)-1);
    if(e & ERROR_CODE_IMU_MISSING)    strncat(str, "IMU_MISSING ",      buf_len-strlen(str)-1);
    if(e & ERROR_CODE_CAM_MISSING)    strncat(str, "CAM_MISSING ",      buf_len-strlen(str)-1);
    if(e & ERROR_CODE_CAM_BAD_RES)    strncat(str, "CAM_BAD_RES ",      buf_len-strlen(str)-1);
    if(e & ERROR_CODE_CAM_BAD_FORMAT) strncat(str, "CAM_BAD_FORMAT ",   buf_len-strlen(str)-1);
    if(e & ERROR_CODE_UNKNOWN)        strncat(str, "UNKNOWN_ERROR ",    buf_len-strlen(str)-1);
    if(e & ERROR_CODE_STALLED)        strncat(str, "STALLED ",          buf_len-strlen(str)-1);
    return 0;
}




// DEPRECATED
const char* modal_image_format_name(int i)
{
    return pipe_image_format_to_string(i);
}


// DEPRECATED
imu_data_t* modal_imu_validate_pipe_data(char* data, int bytes, int* n_packets)
{
    return pipe_validate_imu_data_t(data, bytes, n_packets);
}

// DEPRECATED
pose_4dof_t* modal_pose_4dof_validate_pipe_data(char* data, int bytes, int* n_packets)
{
    return pipe_validate_pose_4dof_t(data, bytes, n_packets);
}

// DEPRECATED
pose_vel_6dof_t* modal_pose_vel_6dof_validate_pipe_data(char* data, int bytes, int* n_packets)
{
    return pipe_validate_pose_vel_6dof_t(data, bytes, n_packets);
}

// DEPRECATED
vio_data_t* modal_vio_validate_pipe_data(char* data, int bytes, int* n_packets)
{
    return pipe_validate_vio_data_t(data, bytes, n_packets);
}

// DEPRECATED
void modal_vio_print_state(int s)
{
    return pipe_print_vio_state(s);
}

// DEPRECATED
void modal_vio_print_error_code(int e)
{
    return pipe_print_vio_error(e);
}
