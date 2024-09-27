/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * OSD RSSI column
 *
 * Selects which column to place RSSI element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_RSSI_COL, -1);

/**
 * OSD RSSI row
 *
 * Selects which row to place RSSI element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_RSSI_ROW, -1);

/**
 *OSD Current Draw column
 *
 * Selects which column to place Current Draw element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_CURR_COL, -1);

/**
 * OSD Current Draw row
 *
 * Selects which row to place Current Draw element in.
 *
 * @group OSD
 * @value -1 - Disabled
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_CURR_ROW, -1);

/**
 *OSD Battery column
 *
 * Selects which column to place Battery element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_BATT_COL, -1);

/**
 * OSD Battery row
 *
 * Selects which row to place Battery element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_BATT_ROW, -1);

/**
 *OSD Cell Battery column
 *
 * Selects which column to place Cell Battery element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_CBATT_COL, -1);

/**
 * OSD Cell Battery row
 *
 * Selects which row to place Cell Battery element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_CBATT_ROW, -1);

/**
 *OSD Disarmed Message column
 *
 * Selects which column to place Disarmed Message element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_DIS_COL, -1);

/**
 * OSD Disarmed Message row
 *
 * Selects which row to place Disarmed Message element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_DIS_ROW, -1);

/**
 *OSD Status Message column
 *
 * Selects which column to place Status Message element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_STATUS_COL, -1);

/**
 * OSD Status Message row
 *
 * Selects which row to place Status Message element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_STATUS_ROW, -1);

/**
 *OSD Flight Mode column
 *
 * Selects which column to place Flight Mode element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_FM_COL, -1);

/**
 * OSD Flight Mode row
 *
 * Selects which row to place Flight Mode element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_FM_ROW, -1);

/**
 *OSD Latitude column
 *
 * Selects which column to place Latitude element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_LAT_COL, -1);

/**
 * OSD Latitude row
 *
 * Selects which row to place Latitude element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_LAT_ROW, -1);

/**
 *OSD Longitude column
 *
 * Selects which column to place Longitude element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_LONG_COL, -1);

/**
 * OSD Longitude row
 *
 * Selects which row to place Longitude element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_LONG_ROW, -1);

/**
 *OSD direction/distance to home column
 *
 * Selects which column to place direction/distance to home element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_HOME_COL, -1);

/**
 * OSD direction/distance to home row
 *
 * Selects which row to place direction/distance to home element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_HOME_ROW, -1);

/**
 *OSD crosshair column
 *
 * Selects which column to place crosshair element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_CH_COL, -1);

/**
 * OSD crosshair row
 *
 * Selects which row to place crosshair element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_CH_ROW, -1);

/**
 *OSD heading angle column
 *
 * Selects which column to place heading angle element in.
 *
 * @group OSD
 * @min -1
 * @max 51
 */
PARAM_DEFINE_INT32(OSD_HDG_COL, -1);

/**
 * OSD heading angle row
 *
 * Selects which row to place heading angle element in.
 *
 * @group OSD
 * @min -1
 * @max 19
 */
PARAM_DEFINE_INT32(OSD_HDG_ROW, -1);

/**
 * VTX channel
 *
 * Selects which VTX channel to broadcast on.
 *
 * @group OSD
 * @min 1
 * @max 8
 */
PARAM_DEFINE_INT32(OSD_CHANNEL, 1);

/**
 * VTX band
 *
 * Selects which VTX band to broadcast on.
 *
 * @group OSD
 * @value 5 - Raceband
 * @value 4 - FatShark
 * @value 3 - Boscam E
 */
PARAM_DEFINE_INT32(OSD_BAND, 5);

/**
 * Remote OSD enable
 *
 * Allow external entities to display OSD strings
 *
 * @group OSD
 * @value 0 - Disabled
 * @value 1 - Enabled
 */
PARAM_DEFINE_INT32(OSD_REMOTE, 0);
