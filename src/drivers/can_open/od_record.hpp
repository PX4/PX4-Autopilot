/****************************************************************************
 *
 *   Copyright (c) 2020-2022 enVgo. All rights reserved.
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

/*
	Automatically timestamps OD records
*/

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "CANopen.h"
#include "OD.h"

class ODRecord
{
public:
	enum od_record_mode {
		OD_RECORD_TO_UORB = 0,
		UORB_TO_OD_RECORD
	};

	ODRecord(od_record_mode mode) :
		_mode(mode) {};
	~ODRecord() {};

	static void set_timestamp(uint64_t timestamp)
	{
		_current_timestamp = timestamp;
	};

protected:
	static constexpr uint8_t SUBIDX_TIMESTAMP{100};
	static uint64_t _current_timestamp;

	od_record_mode _mode;

	static ODR_t od_read_record(OD_stream_t *stream, void *buf,
				    OD_size_t count, OD_size_t *countRead)
	{
		if (stream == NULL || buf == NULL || countRead == NULL) {
			return ODR_DEV_INCOMPAT;
		}

		/* Object was passed by OD_extensionIO_init, use correct type. */
		uint64_t *timestamp = (uint64_t *)stream->object;

		if(stream->subIndex == SUBIDX_TIMESTAMP) {
			OD_size_t varSize = sizeof(uint64_t);

			if (count < varSize || stream->dataLength != varSize) {
				return ODR_DEV_INCOMPAT;
			}

			memcpy(buf, timestamp, varSize);

			*countRead = varSize;
			return ODR_OK;
		} else {
			return OD_readOriginal(stream, buf, count, countRead);
		}
	};

	static ODR_t od_write_record(OD_stream_t *stream, const void *buf,
				     OD_size_t count, OD_size_t *countWritten)
	{
		if (stream == NULL || buf == NULL || countWritten == NULL) {
			return ODR_DEV_INCOMPAT;
		}

		/* Object was passed by OD_extensionIO_init, use correct type. */
		uint64_t *timestamp = (uint64_t *)stream->object;

		*timestamp = _current_timestamp;

		/* write value to the original location in the Object Dictionary */
		return OD_writeOriginal(stream, buf, count, countWritten);
	};
};
