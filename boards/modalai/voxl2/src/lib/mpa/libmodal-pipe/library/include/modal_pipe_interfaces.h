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



#ifndef MODAL_PIPE_INTERFACES_H
#define MODAL_PIPE_INTERFACES_H

#ifdef __cplusplus
extern "C" {
#endif

// Include all separated interface headers
#include <pipe_interfaces/camera_image_metadata_t.h>

#include <pipe_interfaces/tof_data_t.h>
#include <pipe_interfaces/tof2_data_t.h>
#include <pipe_interfaces/imu_data_t.h>
#include <pipe_interfaces/point_cloud_metadata_t.h>
#include <pipe_interfaces/tag_detection_t.h>
#include <pipe_interfaces/pose_4dof_t.h>
#include <pipe_interfaces/pose_vel_6dof_t.h>
#include <pipe_interfaces/pose_vel_6dof2_t.h>
#include <pipe_interfaces/vio_data_t.h>
#include <pipe_interfaces/vfc_data_t.h>
#include <pipe_interfaces/baro_data_t.h>
#include <pipe_interfaces/rc_channels_t.h>
#include <pipe_interfaces/crsf_raw_t.h>
#include <pipe_interfaces/cpu_stats2_t.h>
#include <pipe_interfaces/object_tracking_t.h>

#if defined MAVLINK_H || defined EN_MAVLINK_SUPPORT
#include "pipe_interfaces/mavlink_message_t.h"
#endif

#ifdef EN_ION_BUF
#include "pipe_interfaces/mpa_ion_buf_t.h"
#endif

#ifdef __cplusplus
}
#endif

#endif // MODAL_PIPE_INTERFACES_H
