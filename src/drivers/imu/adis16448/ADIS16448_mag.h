/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file ADIS16448_mag.h
 *
 * Driver for the Analog device ADIS16448 connected via SPI.
 *
 * @author Amir Melzer
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

/* Forward class declaration. */
class ADIS16448;

/**
 * Helper class implementing the mag driver node.
 */
class ADIS16448_mag : public device::CDev
{
public:
	ADIS16448_mag(ADIS16448 *parent, const char *path);
	virtual ~ADIS16448_mag();

	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual int init();

protected:
	friend class ADIS16448;

	void parent_poll_notify();
private:
	ADIS16448 *_parent{nullptr};
	orb_advert_t _mag_topic{nullptr};
	int _mag_orb_class_instance{-1};
	int _mag_class_instance{-1};

	// Disallow copy construction and move assignment of this class due to pointer data members.
	ADIS16448_mag(const ADIS16448_mag &) = delete;
	ADIS16448_mag operator=(const ADIS16448_mag &) = delete;
};
