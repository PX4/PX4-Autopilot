#include "fault.h"
#include "libc.h"
#include "rpmsg_remote.h"

volatile uint32_t cm4_fault_escalate;

/* Unmapped on the CM4 bus: an access ends in a bus fault. */
#define FAULT_BAD_ADDR 0xD0000000u

/* Transport and core faults. A layer or application adds its own kinds by
 * defining fault_inject_app() (lib/uorb_fault.c for the uORB bridge).
 */
static const char *const g_kinds[] = {
	"oversize", "badhdr", "hardfault", "busfault", "lockup", "hang", "badused",
};

int __attribute__((weak)) fault_inject_app(const char *kind, char *reply, size_t cap)
{
	(void)kind;
	(void)reply;
	(void)cap;
	return 0;
}

void fault_reply(char *reply, size_t cap, const char *kind, const char *text)
{
	reply[0] = '\0';
	str_append(reply, cap, kind);
	str_append(reply, cap, ": ");
	str_append(reply, cap, text);
}

void fault_inject(const char *kind, char *reply, size_t cap)
{
	int ret;

	if (strncmp(kind, "list", 5) == 0) {
		reply[0] = '\0';

		for (size_t i = 0; i < sizeof(g_kinds) / sizeof(g_kinds[0]); i++) {
			str_append(reply, cap, g_kinds[i]);
			str_append(reply, cap, " ");
		}

		fault_inject_app(kind, reply, cap);
		return;
	}

	if (fault_inject_app(kind, reply, cap)) {
		return;
	}

	if (strncmp(kind, "oversize", 9) == 0) {
		static uint8_t big[1200];
		ret = rp_send_service(rp_hello_service(), big, sizeof(big));
		fault_reply(reply, cap, kind, "rp_send of 1200 bytes returned ");
		str_append_u32(reply, cap, (uint32_t)ret);
		str_append(reply, cap, " (expect -2, ring untouched)");
		return;
	}

	if (strncmp(kind, "badhdr", 7) == 0) {
		rp_inject_hdr_len(60000);
		fault_reply(reply, cap, kind, "this reply claims 60000 bytes; a hardened host drops it");
		return;
	}

	if (strncmp(kind, "badused", 8) == 0) {
		rp_inject_used_len(0);
		fault_reply(reply, cap, kind, "next host TX buffer comes back with used->len 0");
		return;
	}

	/* Lethal from here on. */

	if (strncmp(kind, "hardfault", 10) == 0) {
		__asm volatile("udf #0");
	}

	if (strncmp(kind, "busfault", 9) == 0) {
		*(volatile uint32_t *)FAULT_BAD_ADDR = 1;
		__asm volatile("dsb");
		fault_reply(reply, cap, kind, "write to 0xd0000000 did not fault");
		return;
	}

	if (strncmp(kind, "lockup", 7) == 0) {
		cm4_fault_escalate = 1;
		__asm volatile("udf #0");
	}

	if (strncmp(kind, "hang", 5) == 0) {
		__asm volatile("cpsid i");

		for (;;) {
		}
	}

	fault_reply(reply, cap, kind, "unknown kind, try list");
}
