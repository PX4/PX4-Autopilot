/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#undef NULL  /* To please compiler */

#include <sbi/riscv_io.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hartmask.h>
#include <sbi/sbi_string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_DOMAIN_MAX_COUNT                4
#define MPFS_DOMAIN_REGION_MAX_COUNT        12

#define MPFS_SEL4_HART                1
#define MPFS_LINUX_HART               3

#define SEL4_BOOTADDRESS              CONFIG_MPFS_HART1_ENTRYPOINT
#define LINUX_BOOTADDRESS             CONFIG_MPFS_HART3_ENTRYPOINT

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sbi_domain mpfs_domains[MPFS_DOMAIN_MAX_COUNT];
static struct sbi_hartmask mpfs_masks[MPFS_DOMAIN_MAX_COUNT];
static struct sbi_domain_memregion
	mpfs_regions[MPFS_DOMAIN_REGION_MAX_COUNT + 1] = {
	0
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_domains_init
 *
 * Description:
 *   Initializes the domain structures. Hard coded values
 *   Domain 1: (SEL4)
 *        Hart 1, Access (RWX) to all memory
 *   Domain 2: (Linux)
 *        Hart 3 and 4, Access (RWX) to all memory
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

int board_domains_init(void)
{
	int err = -1;
	int i = 1;
	struct sbi_domain_memregion *reg;

	const char sel4_domain_name[] = "Sel4-Tee-Domain";
	const char linux_domain_name[] = "Linux-Ree-Domain";

	/* Hard code Sel4 domain for hart 1 */

	mpfs_domains[0].boot_hartid = MPFS_SEL4_HART;
	sbi_strncpy(mpfs_domains[0].name,
		    sel4_domain_name, sizeof(sel4_domain_name));
	mpfs_domains[0].next_addr = SEL4_BOOTADDRESS;
	mpfs_domains[0].next_mode = PRV_S;
	mpfs_domains[0].next_arg1 = 0;

	/* All memory, all access */

	sbi_domain_memregion_init(0, ~0UL,
				  (SBI_DOMAIN_MEMREGION_READABLE |
				   SBI_DOMAIN_MEMREGION_WRITEABLE |
				   SBI_DOMAIN_MEMREGION_EXECUTABLE),
				  &mpfs_regions[0]);

	mpfs_domains[0].regions = mpfs_regions;
	sbi_hartmask_set_hart(1, &mpfs_masks[0]);
	mpfs_domains[0].possible_harts = &mpfs_masks[0];
	mpfs_domains[0].system_reset_allowed = true;

	/* Linux Domain, hart 3,4 */

	mpfs_domains[1].boot_hartid = MPFS_LINUX_HART;
	sbi_strncpy(mpfs_domains[1].name,
		    linux_domain_name, sizeof(linux_domain_name));
	mpfs_domains[1].next_addr = LINUX_BOOTADDRESS;
	mpfs_domains[1].next_mode = PRV_S;
	mpfs_domains[1].next_arg1 = 0;

	mpfs_domains[1].regions = mpfs_regions;
	sbi_hartmask_set_hart(3, &mpfs_masks[1]);
	sbi_hartmask_set_hart(4, &mpfs_masks[1]);
	mpfs_domains[1].possible_harts = &mpfs_masks[1];

	sbi_domain_root_add_memregion(&mpfs_regions[0]);

	sbi_domain_for_each_memregion(&root, reg) {
		if ((reg->flags & SBI_DOMAIN_MEMREGION_READABLE) ||
		    (reg->flags & SBI_DOMAIN_MEMREGION_WRITEABLE) ||
		    (reg->flags & SBI_DOMAIN_MEMREGION_EXECUTABLE)) {
			continue;
		}

		if (MPFS_DOMAIN_REGION_MAX_COUNT <= i) {
			return SBI_EINVAL;
		}

		sbi_memcpy(&mpfs_regions[i++], reg, sizeof(*reg));
	}

	err = sbi_domain_register(&mpfs_domains[0], &mpfs_masks[0]);

	if (err) {
		sbi_printf("Sel4 Domain Register failed %d\n", err);
		return err;
	}

	err = sbi_domain_register(&mpfs_domains[1], &mpfs_masks[1]);

	if (err) {
		sbi_printf("Linux Domain Register failed %d\n", err);
		return err;
	}

	return 0;
}
