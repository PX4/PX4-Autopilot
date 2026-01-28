/****************************************************************************
 *
 *   Copyright (c) 2021-2024 PX4 Development Team. All rights reserved.
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
 * Amphenol DLVR Low Voltage Digital Pressure Sensor Series (external I2C) - only continuous-sampling sensors
 *
 * @reboot_required true
 * @group Sensors
 * @value 0  Sensor disabled, when explicitly started treated as DLVR L10D
 * @value 1  DLVR F50D
 * @value 2  DLVR L01D
 * @value 3  DLVR L02D
 * @value 4  DLVR L05D
 * @value 5  DLVR L10D
 * @value 6  DLVR L20D
 * @value 7  DLVR L30D
 * @value 8  DLVR L60D
 * @value 9  DLVR L01G
 * @value 10 DLVR L02G
 * @value 11 DLVR L05G
 * @value 12 DLVR L10G
 * @value 13 DLVR L20G
 * @value 14 DLVR L30G
 * @value 15 DLVR L60G
 */
PARAM_DEFINE_INT32(SENS_EN_DLVR, 5);
