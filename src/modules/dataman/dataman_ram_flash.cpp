/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>

#include "dataman.h"
#include "dataman_internal.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>

#if defined(FLASH_BASED_DATAMAN)
#include <nuttx/clock.h>
#include <nuttx/progmem.h>
#endif

#if defined(FLASH_BASED_DATAMAN)

extern dm_operations_t dm_ram_operations;
extern dm_operations_data_t dm_operations_data;

extern px4_sem_t g_init_sema;

const dm_sector_descriptor_t *k_dataman_flash_sector = nullptr;

/* Private Ram_Flash based Operations */
#define RAM_FLASH_FLUSH_TIMEOUT_USEC USEC_PER_SEC

static void _ram_flash_update_flush_timeout()
{
	timespec &abstime = dm_operations_data.ram_flash.flush_timeout;

	if (clock_gettime(CLOCK_REALTIME, &abstime) == 0) {
		const unsigned billion = 1000 * 1000 * 1000;
		uint64_t nsecs = abstime.tv_nsec + (uint64_t)RAM_FLASH_FLUSH_TIMEOUT_USEC * 1000;
		abstime.tv_sec += nsecs / billion;
		nsecs -= (nsecs / billion) * billion;
		abstime.tv_nsec = nsecs;
	}
}

static ssize_t _ram_flash_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf,
				size_t count)
{
	ssize_t ret = dm_ram_operations.write(item, index, persistence, buf, count);

	if (ret < 1) {
		return ret;
	}

	if (persistence == DM_PERSIST_POWER_ON_RESET) {
		_ram_flash_update_flush_timeout();
	}

	return ret;
}

static ssize_t _ram_flash_read(dm_item_t item, unsigned index, void *buf, size_t count)
{
	return dm_ram_operations.read(item, index, buf, count);
}

static int _ram_flash_clear(dm_item_t item)
{
	ssize_t ret = dm_ram_operations.clear(item);

	if (ret < 0) {
		return ret;
	}

	_ram_flash_update_flush_timeout();
	return ret;
}

static int _ram_flash_restart(dm_reset_reason reason)
{
	return dm_ram_operations.restart(reason);
}

static int _ram_flash_initialize(unsigned max_offset)
{
	if (max_offset & 1) {
		/* STM32 flash API requires half-word(2 bytes) access */
		max_offset++;
	}

	if (max_offset > k_dataman_flash_sector->size) {
		PX4_WARN("Could not allocate %d bytes of flash memory", max_offset);
		return -1;
	}

	ssize_t ret = dm_ram_operations.initialize(max_offset);

	if (ret != 0) {
		return ret;
	}

	/* Copy flash to RAM */
	memcpy(dm_operations_data.ram_flash.data, (void *)k_dataman_flash_sector->address, max_offset);

	struct dataman_compat_s compat_state;
	ret = _ram_flash_read(DM_KEY_COMPAT, 0, &compat_state, sizeof(compat_state));

	if (ret != sizeof(compat_state) || compat_state.key != DM_COMPAT_KEY) {
		/* Not compatible: clear RAM and write DM_KEY_COMPAT(it will flush flash) */
		memset(dm_operations_data.ram_flash.data, 0, max_offset);

		compat_state.key = DM_COMPAT_KEY;
		ret = _ram_flash_write(DM_KEY_COMPAT, 0, DM_PERSIST_POWER_ON_RESET, &compat_state, sizeof(compat_state));
	}

	return ret > 0 ? 0 : -1;
}

static void _ram_flash_flush()
{
	/*
	 * reseting flush_timeout even in errors cases to avoid looping
	 * forever in case of flash failure.
	 */
	dm_operations_data.ram_flash.flush_timeout.tv_nsec = 0;
	dm_operations_data.ram_flash.flush_timeout.tv_sec = 0;

	ssize_t ret = up_progmem_getpage(k_dataman_flash_sector->address);
	ret = up_progmem_eraseblock(ret);

	if (ret < 0) {
		PX4_WARN("Error erasing flash sector %u", k_dataman_flash_sector->page);
		return;
	}

	const ssize_t len = (dm_operations_data.ram_flash.data_end - dm_operations_data.ram_flash.data) + 1;
	ret = up_progmem_write(k_dataman_flash_sector->address, dm_operations_data.ram_flash.data, len);

	if (ret < len) {
		PX4_WARN("Error writing to flash sector %u, error: %i", k_dataman_flash_sector->page, ret);
		return;
	}
}

static void _ram_flash_shutdown()
{
	if (dm_operations_data.ram_flash.flush_timeout.tv_sec) {
		_ram_flash_flush();
	}

	dm_ram_operations.shutdown();
}

static int _ram_flash_wait(px4_sem_t *sem)
{
	if (!dm_operations_data.ram_flash.flush_timeout.tv_sec) {
		px4_sem_wait(sem);
		return 0;
	}

	int ret;

	while ((ret = px4_sem_timedwait(sem, &dm_operations_data.ram_flash.flush_timeout)) == -1 && errno == EINTR);

	if (ret == 0) {
		/* a work was queued before timeout */
		return 0;
	}

	_ram_flash_flush();
	return 0;
}

dm_operations_t dm_ram_flash_operations = {
	.write   = _ram_flash_write,
	.read    = _ram_flash_read,
	.clear   = _ram_flash_clear,
	.restart = _ram_flash_restart,
	.initialize = _ram_flash_initialize,
	.shutdown = _ram_flash_shutdown,
	.wait = _ram_flash_wait,
};

#endif // FLASH_BASED_DATAMAN
