/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *                 Author: David Sidrane <david_s5@nscdg.com>
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
 * @file board_internal_common.h
 *
 * Provide the internal common board interfaces that should only be used
 * in the board source.
 */

#pragma once

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Name: board_adc_init
 *
 * Description:
 *   boards may provide this function to allow complex version-ing.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *
 *      OK, or -1 if the function failed.
 */

__EXPORT int board_adc_init(void);

/************************************************************************************
 * Name: board_adc_sample
 *
 * Description:
 *   boards provide this function to allow complex version-ing.
 *
 * Input Parameters:
 *   channel  - The number of the adc channel to read.
 *
 * Returned Value:
 *    The ADC DN read for the channel or 0xffff if there
 *    is an error reading the channel.
 */

__EXPORT uint16_t board_adc_sample(unsigned channel);


/************************************************************************************
 * Name: board_gpio_init
 *
 * Description:
 *   Board may provide a list of GPI pins to get initialized
 *
 * Input Parameters:
 *  list    - A list of GPIO pins to be initialized
 *  count   - Size of the list
 *
 * Returned Value:
 *   Nothing
 *
 ************************************************************************************/

__EXPORT void board_gpio_init(const uint32_t list[], int count);

/************************************************************************************
  * Name: board_determine_hw_info
 *
 * Description:
 *	Uses the HW revision and version detection added in FMUv5.
 *	See https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY
 *	HW REV and VER ID tab.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0  - on success or negated errorno
 *   The values for integer value of this boards hardware revision.
 *   and integer value of this boards hardware version are set.
 *
 *   A value of 0 is the default for boards supporting the BOARD_HAS_HW_VERSIONING API.
 *   but not having R1 and R2.
 *
 ************************************************************************************/

__EXPORT int board_determine_hw_info(void);
