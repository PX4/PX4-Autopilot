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

struct mpfs_domain {
	const char *domain_name;
	const int *harts;
	const unsigned n_of_harts;
	const uintptr_t bootaddress;
	const bool reset_allowed;
	const bool domain_enabled;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sbi_domain mpfs_domains[MPFS_DOMAIN_MAX_COUNT];
static struct sbi_hartmask mpfs_masks[MPFS_DOMAIN_MAX_COUNT];
static struct sbi_domain_memregion
	mpfs_regions[MPFS_DOMAIN_REGION_MAX_COUNT + 1] = {
	0
};

static const int linux_harts[] = {1, 3, 4};
static const int px4_harts[] = {2};

static const struct mpfs_domain domains[] = {
	{
		.domain_name = "Linux-Ree-Domain",
		.harts = linux_harts,
		.n_of_harts = sizeof(linux_harts) / sizeof(linux_harts[0]),
		.bootaddress = CONFIG_MPFS_HART3_ENTRYPOINT,
		.reset_allowed = false,
		.domain_enabled = true,
	},
	{
		.domain_name = "PX4-Ree-Domain",
		.harts = px4_harts,
		.n_of_harts = sizeof(px4_harts) / sizeof(px4_harts[0]),
		.bootaddress = CONFIG_MPFS_HART2_ENTRYPOINT,
		.reset_allowed = true,
#ifdef CONFIG_BUILD_KERNEL
		.domain_enabled = true,
#else
		.domain_enabled = false,
#endif
	},
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_domains_init
 *
 * Description:
 *   Initializes the domain structures from the hard coded domains table
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
	unsigned i;
	unsigned j;
	struct sbi_domain_memregion *reg;

	/* All memory, all access */

	sbi_domain_memregion_init(0, ~0UL,
				  (SBI_DOMAIN_MEMREGION_READABLE |
				   SBI_DOMAIN_MEMREGION_WRITEABLE |
				   SBI_DOMAIN_MEMREGION_EXECUTABLE),
				  &mpfs_regions[0]);

	/* Add to root domain */

	sbi_domain_root_add_memregion(&mpfs_regions[0]);

	i = 1;
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

	/* Go through the constant configuration list */

	for (i = 0; i < sizeof(domains) / sizeof(domains[0]); i++) {

		if (!domains[i].domain_enabled) {
			continue;
		}

		/* Set first hart id in the list as boot hart */
		mpfs_domains[i].boot_hartid = domains[i].harts[0];
		sbi_strncpy(mpfs_domains[i].name,
			    domains[i].domain_name, sizeof(mpfs_domains[i].name));

		mpfs_domains[i].next_addr = domains[i].bootaddress;
		mpfs_domains[i].next_mode = PRV_S;
		mpfs_domains[i].next_arg1 = 0;
		mpfs_domains[i].regions = mpfs_regions;

		for (j = 0; j < domains[i].n_of_harts; j++) {
			sbi_hartmask_set_hart(domains[i].harts[j], &mpfs_masks[i]);
		}

		mpfs_domains[i].possible_harts = &mpfs_masks[i];
		mpfs_domains[i].system_reset_allowed = domains[i].reset_allowed;

		/* Register the domain */

		err = sbi_domain_register(&mpfs_domains[i], &mpfs_masks[i]);

		if (err) {
			sbi_printf("%s register failed %d\n", domains[i].domain_name, err);
			return err;
		}
	}

	return 0;
}
