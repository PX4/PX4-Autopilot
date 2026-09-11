#pragma once

#include <stddef.h>
#include <stdint.h>

/* Minimal virtio-rpmsg remote (device side) matching the NuttX rptun master:
 * vring0 = remote TX (host pre-posts RX buffers), vring1 = remote RX (host
 * TX buffers). Both vrings, their buffers and the resource table live in the
 * .resource_table region; the host fills in vring addresses before releasing
 * this core.
 */

struct rpmsg_hdr {
	uint32_t src;
	uint32_t dst;
	uint32_t reserved;
	uint16_t len;
	uint16_t flags;
	uint8_t data[];
};

/* Local endpoints. The runtime provides rpmsg-ping and rpmsg-hello; an
 * application registers its own before rp_init(), which announces them all
 * to the host. Returns a service index for rp_send_service(), -1 when the
 * table is full.
 */
#define RP_MAX_SERVICES 6

typedef void (*rp_handler_t)(const struct rpmsg_hdr *hdr);

int rp_register_service(const char *name, rp_handler_t handler, void (*bound)(void));

/* Wait for the host to reach DRIVER_OK, attach vrings, announce services. */
void rp_init(void);

/* Drain vring1 and answer; call after a MU kick (or periodically). */
void rp_process(void);

/* Send payload from local endpoint src to remote endpoint dst. */
int rp_send(uint32_t src, uint32_t dst, const void *payload, uint16_t len);

/* Send from a local service to its bound host endpoint; -1 if not bound,
 * -2 if len does not fit one buffer.
 */
int rp_send_service(int svc, const void *payload, uint16_t len);

/* Service index of the hello endpoint (fault injection sends on it). */
int rp_hello_service(void);

#ifdef CM4_FAULT_INJECT
/* One-shot lies to the host (fault.c): the next rp_send writes this rpmsg
 * header length; the next consumed host TX descriptor is returned with this
 * used->len.
 */
void rp_inject_hdr_len(uint16_t len);
void rp_inject_used_len(uint32_t len);
#endif
