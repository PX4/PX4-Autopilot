/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#ifndef DRIVERS_IMU_ADIS16477_ADIS16477_GYRO_HPP_
#define DRIVERS_IMU_ADIS16477_ADIS16477_GYRO_HPP_

#include "ADIS16477.hpp"

#include <drivers/device/CDev.hpp>
#include <drivers/drv_gyro.h>

/**
 * Helper class implementing the gyro driver node.
 */
class ADIS16477_gyro : public device::CDev
{
public:
	ADIS16477_gyro(ADIS16477 *parent, const char *path);
	virtual ~ADIS16477_gyro();

	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class ADIS16477;

private:
	ADIS16477			*_parent{nullptr};
	orb_advert_t		_gyro_topic{nullptr};

	int					_gyro_orb_class_instance{-1};
	int					_gyro_class_instance{-1};

	/* do not allow to copy this class due to pointer data members */
	ADIS16477_gyro(const ADIS16477_gyro &);
	ADIS16477_gyro operator=(const ADIS16477_gyro &);

};

#endif /* DRIVERS_IMU_ADIS16477_ADIS16477_GYRO_HPP_ */
