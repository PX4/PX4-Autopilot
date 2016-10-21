/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include <px4_posix.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "navio_gpio.h"

namespace navio_gpio
{
extern "C" __EXPORT int navio_gpio_main(int argc, char *argv[]);

int Gpio::start(void)
{
	int mem_fd;

	if ((mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC)) < 0) {
		PX4_WARN("failed to open gpiomem");
		return -1;
	}

	if ((_gpio_map = mmap(NULL,
			      GPIO_BLOCK_SIZE,
			      PROT_READ | PROT_WRITE,
			      MAP_SHARED,
			      mem_fd,
			      GPIO_PHYS_ADDR)) == MAP_FAILED) {
		PX4_WARN("failed to mmap GPIO region");
		close(mem_fd);
		return -1;
	}

	close(mem_fd);

	return 0;
}

int Gpio::stop(void)
{
	if (munmap(_gpio_map, GPIO_BLOCK_SIZE) < 0) {
		return -1;
	}

	return 0;
}

void Gpio::atomic_modify(uintptr_t addr,
			 unsigned int shift,
			 unsigned int mask,
			 unsigned int value)
{
	uint32_t tmp;

	m_lock.lock();

	tmp = *(volatile uint32_t *)addr;
	tmp = (tmp & ~(mask << shift)) | (value << shift);
	*(volatile uint32_t *)addr = tmp;

	m_lock.unlock();
}


int Gpio::configgpio(uint32_t pinset)
{
	unsigned int pin;
	unsigned int cnf;

	uintptr_t addr;
	unsigned int shift;

	pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
	cnf = (pinset & GPIO_CNF_MASK) >> GPIO_CNF_SHIFT;

	addr = (uintptr_t)_gpio_map + GPIO_GPFSEL0_OFFSET + pin / 10;
	shift = (pin % 10) * 3;

	atomic_modify(addr, shift, GPIO_CNF_MASK >> GPIO_CNF_SHIFT, cnf);

	return 0;
}


int Gpio::unconfiggpio(uint32_t pinset)
{
	return Gpio::configgpio(pinset);
}


bool Gpio::gpioread(uint32_t pinset)
{
	unsigned int pin;

	uintptr_t addr;
	unsigned int shift;

	pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

	addr = (uintptr_t)_gpio_map + GPIO_GPLEV0_OFFSET + pin / 32;
	shift = pin % 32;

	return (*(volatile uint32_t *)addr >> shift) & 0x1u;
}


void Gpio::gpiowrite(uint32_t pinset, bool value)
{
	unsigned int pin;

	uintptr_t addr;
	unsigned int shift;

	pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

	if (value == 0) {
		addr = (uintptr_t)_gpio_map + GPIO_GPCLR0_OFFSET + pin / 32;

	} else {
		addr = (uintptr_t)_gpio_map + GPIO_GPSET0_OFFSET + pin / 32;
	}

	shift = pin % 32;

	*(volatile uint32_t *)addr = 0x1u << shift;
}

static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PX4_INFO("usage: navio_gpio {start|stop|status}");
}
static Gpio *gpio = nullptr;

int navio_gpio_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		PX4_WARN("gpio should only be started directly for debugging");

		if (gpio != nullptr && gpio->isMapped()) {
			PX4_WARN("already mapped");
			return 0;
		}

		gpio = new Gpio();

		if (gpio == nullptr) {
			PX4_ERR("alloc failed");
			return 1;
		}

		int ret = gpio->start();

		if (ret != 0) {
			PX4_ERR("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {

		if (gpio == nullptr) {
			PX4_WARN("gpio not started from navio_gpio");
			return 0;
		}

		if (!gpio->isMapped()) {
			PX4_WARN("not mapped");
			return 1;
		}

		gpio->stop();

		delete gpio;
		gpio = nullptr;

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (gpio == nullptr) {
			PX4_WARN("gpio not started from navio_gpio");
			return 0;
		}

		if (gpio->isMapped()) {
			PX4_INFO("mapped");

		} else {
			PX4_INFO("not mapped\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;

}

} // navio_gpio
