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

#include "SyncObj.hpp"
#include <stdint.h>

#define GPIO_BLOCK_SIZE (4096)
#define GPIO_PHYS_ADDR  (0x3F200000)

#define GPIO_GPFSEL0_OFFSET    (0x0)
#define GPIO_GPLEV0_OFFSET     (0x34)
#define GPIO_GPSET0_OFFSET     (0x1C)
#define GPIO_GPCLR0_OFFSET     (0x28)


#define GPIO_CNF_SHIFT  6
#define GPIO_CNF_MASK   (3 << GPIO_CNF_SHIFT)
#define GPIO_CNF_INPUT  (0 << GPIO_CNF_SHIFT)
#define GPIO_CNF_OUTPUT (1 << GPIO_CNF_SHIFT)
#define GPIO_CNF_AF0    (4 << GPIO_CNF_SHIFT)
#define GPIO_CNF_AF1    (5 << GPIO_CNF_SHIFT)
#define GPIO_CNF_AF2    (6 << GPIO_CNF_SHIFT)
#define GPIO_CNF_AF3    (7 << GPIO_CNF_SHIFT)
#define GPIO_CNF_AF4    (3 << GPIO_CNF_SHIFT)
#define GPIO_CNF_AF5    (2 << GPIO_CNF_SHIFT)

#define GPIO_PIN_SHIFT  0
#define GPIO_PIN_MASK   (63 << GPIO_PIN_SHIFT)
#define GPIO_PIN0       (0 << GPIO_PIN_SHIFT)
#define GPIO_PIN1       (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2       (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3       (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4       (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5       (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6       (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7       (7 << GPIO_PIN_SHIFT)
#define GPIO_PIN8       (8 << GPIO_PIN_SHIFT)
#define GPIO_PIN9       (9 << GPIO_PIN_SHIFT)
#define GPIO_PIN10      (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11      (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12      (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13      (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14      (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15      (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16      (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17      (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18      (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19      (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20      (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21      (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22      (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23      (23 << GPIO_PIN_SHIFT)
#define GPIO_PIN24      (24 << GPIO_PIN_SHIFT)
#define GPIO_PIN25      (25 << GPIO_PIN_SHIFT)
#define GPIO_PIN26      (26 << GPIO_PIN_SHIFT)
#define GPIO_PIN27      (27 << GPIO_PIN_SHIFT)
#define GPIO_PIN28      (28 << GPIO_PIN_SHIFT)
#define GPIO_PIN29      (29 << GPIO_PIN_SHIFT)
#define GPIO_PIN30      (30 << GPIO_PIN_SHIFT)
#define GPIO_PIN31      (31 << GPIO_PIN_SHIFT)
#define GPIO_PIN32      (32 << GPIO_PIN_SHIFT)
#define GPIO_PIN33      (33 << GPIO_PIN_SHIFT)
#define GPIO_PIN34      (34 << GPIO_PIN_SHIFT)
#define GPIO_PIN35      (35 << GPIO_PIN_SHIFT)
#define GPIO_PIN36      (36 << GPIO_PIN_SHIFT)
#define GPIO_PIN37      (37 << GPIO_PIN_SHIFT)
#define GPIO_PIN38      (38 << GPIO_PIN_SHIFT)
#define GPIO_PIN39      (39 << GPIO_PIN_SHIFT)
#define GPIO_PIN40      (40 << GPIO_PIN_SHIFT)
#define GPIO_PIN41      (41 << GPIO_PIN_SHIFT)
#define GPIO_PIN42      (42 << GPIO_PIN_SHIFT)
#define GPIO_PIN43      (43 << GPIO_PIN_SHIFT)
#define GPIO_PIN44      (44 << GPIO_PIN_SHIFT)
#define GPIO_PIN45      (45 << GPIO_PIN_SHIFT)
#define GPIO_PIN46      (46 << GPIO_PIN_SHIFT)
#define GPIO_PIN47      (47 << GPIO_PIN_SHIFT)
#define GPIO_PIN48      (48 << GPIO_PIN_SHIFT)
#define GPIO_PIN49      (49 << GPIO_PIN_SHIFT)
#define GPIO_PIN50      (50 << GPIO_PIN_SHIFT)
#define GPIO_PIN51      (51 << GPIO_PIN_SHIFT)
#define GPIO_PIN52      (52 << GPIO_PIN_SHIFT)
#define GPIO_PIN53      (53 << GPIO_PIN_SHIFT)
#define GPIO_PIN54      (54 << GPIO_PIN_SHIFT)

using namespace DriverFramework;

namespace navio_gpio
{

class Gpio
{
public:
	Gpio()
	{ }
	~Gpio()
	{ }

	int start();
	int stop();

	int configgpio(uint32_t pinset);
	int unconfiggpio(uint32_t pinset);
	bool gpioread(uint32_t pinset);
	void gpiowrite(uint32_t pinset, bool value);

	bool isMapped() { return _gpio_map != nullptr; }

private:
	void atomic_modify(uintptr_t addr,
			   unsigned int shift,
			   unsigned int mask,
			   unsigned int value);

	void *_gpio_map;

	SyncObj m_lock;
};

} // namespace navio_gpio
