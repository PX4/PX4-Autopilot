#include "rpmsg_remote.h"
#include "libc.h"
#include "mu.h"
#include "status.h"

#ifdef CM4_FAULT_INJECT
#include "fault.h"
static uint16_t g_inject_hdr_len;
static uint32_t g_inject_used_len = UINT32_MAX;

void rp_inject_hdr_len(uint16_t len)
{
	g_inject_hdr_len = len;
}

void rp_inject_used_len(uint32_t len)
{
	g_inject_used_len = len;
}
#endif

/* ---- Resource table: layout of NuttX struct rptun_rsc_s ---------------- */

struct resource_table {
	uint32_t ver;
	uint32_t num;
	uint32_t reserved[2];
};

struct fw_rsc_trace {
	uint32_t type;
	uint32_t da;
	uint32_t len;
	uint32_t reserved;
	char name[32];
};

struct fw_rsc_vdev {
	uint32_t type;
	uint32_t id;
	uint32_t notifyid;
	uint32_t dfeatures;
	uint32_t gfeatures;
	uint32_t config_len;
	uint8_t status;
	uint8_t num_of_vrings;
	uint8_t reserved[2];
};

struct fw_rsc_vdev_vring {
	uint32_t da;
	uint32_t align;
	uint32_t num;
	uint32_t notifyid;
	uint32_t reserved;
};

struct fw_rsc_config {
	uint32_t h2r_buf_size;
	uint32_t r2h_buf_size;
	uint32_t reserved[14];
};

/* Shared memory the host may place vrings and buffers in. NuttX 12.12 rptun
 * carves them out after the table on its own; NuttX 13 rptun takes them
 * from this entry and refuses the device without it. Both stay inside the
 * window the runtime accepts descriptors from.
 */
struct fw_rsc_carveout {
	uint32_t type;
	uint32_t da;
	uint32_t pa;
	uint32_t len;
	uint32_t flags;
	uint32_t reserved;
	uint8_t name[32];
};

struct rptun_rsc_s {
	struct resource_table hdr;
	uint32_t offset[2];
	struct fw_rsc_trace log_trace;
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring rpmsg_vring0;
	struct fw_rsc_vdev_vring rpmsg_vring1;
	struct fw_rsc_config config;
	struct fw_rsc_carveout shm;
} __attribute__((aligned(8)));

_Static_assert(__builtin_offsetof(struct rptun_rsc_s, rpmsg_vring0) == 100, "vring0 offset");
_Static_assert(__builtin_offsetof(struct rptun_rsc_s, config) == 140, "config offset");
_Static_assert(__builtin_offsetof(struct rptun_rsc_s, shm) == 204, "carveout offset");
_Static_assert(sizeof(struct rptun_rsc_s) == 264, "rsc size");

#define RSC_CARVEOUT            0
#define RSC_VDEV                3
#define FW_RSC_U32_ADDR_ANY     0xFFFFFFFFu
#define VIRTIO_ID_RPMSG         7
#define VIRTIO_RPMSG_F_NS       (1u << 0)
#define VIRTIO_RPMSG_F_ACK      (1u << 1)
#define VIRTIO_RPMSG_F_BUFSZ    (1u << 2)
#define VIRTIO_STATUS_DRIVER_OK 0x04

#define VRING_NUM               16
#define VRING_ALIGN             8
#define RPMSG_BUF_SIZE          1024

#define VDEV_NOTIFYID           0
#define VRING0_NOTIFYID         1
#define VRING1_NOTIFYID         2

__attribute__((section(".resource_table"), used))
volatile struct rptun_rsc_s g_rsc = {
	.hdr = { .ver = 1, .num = 2 },
	.offset = { __builtin_offsetof(struct rptun_rsc_s, rpmsg_vdev), __builtin_offsetof(struct rptun_rsc_s, shm) },
	.rpmsg_vdev = {
		.type = RSC_VDEV,
		.id = VIRTIO_ID_RPMSG,
		.notifyid = VDEV_NOTIFYID,
		.dfeatures = VIRTIO_RPMSG_F_NS | VIRTIO_RPMSG_F_ACK | VIRTIO_RPMSG_F_BUFSZ,
		.config_len = sizeof(struct fw_rsc_config),
		.num_of_vrings = 2,
	},
	.rpmsg_vring0 = { .align = VRING_ALIGN, .num = VRING_NUM, .notifyid = VRING0_NOTIFYID },
	.rpmsg_vring1 = { .align = VRING_ALIGN, .num = VRING_NUM, .notifyid = VRING1_NOTIFYID },
	.config = { .h2r_buf_size = RPMSG_BUF_SIZE, .r2h_buf_size = RPMSG_BUF_SIZE },
	.shm = {
		.type = RSC_CARVEOUT,
		.da = CM4_SHM_DA + sizeof(struct rptun_rsc_s),
		.pa = FW_RSC_U32_ADDR_ANY,
		.len = CM4_SHM_USABLE - sizeof(struct rptun_rsc_s),
		.name = "vdev0buffer",
	},
};

/* ---- Address translation ----------------------------------------------- */

/* The host writes buffer addresses in the vring descriptors as CM7 physical
 * addresses (LMEM backdoor). Map them back into our TCM.
 */
static uintptr_t pa_to_va(uint64_t pa)
{
	uintptr_t a = (uintptr_t)pa;

	if (a - CM4_CODE_TCM_PA < CM4_TCM_SIZE) {
		return a - CM4_CODE_TCM_PA + CM4_CODE_TCM_DA;
	}

	return a;
}

/* Everything the host hands us must fall inside the usable part of the
 * shared window (below the application name and status block); anything
 * else means the host wrote its table somewhere we cannot see, and
 * following it would spray writes across the system bus.
 */
static int in_shm(uintptr_t va, size_t len)
{
	return va >= CM4_SHM_DA && va + len <= CM4_SHM_DA + CM4_SHM_USABLE;
}

/* ---- Vrings -------------------------------------------------------------- */

struct vring_desc {
	uint64_t addr;
	uint32_t len;
	uint16_t flags;
	uint16_t next;
};

struct vring_avail {
	uint16_t flags;
	uint16_t idx;
	uint16_t ring[];
};

struct vring_used_elem {
	uint32_t id;
	uint32_t len;
};

struct vring_used {
	uint16_t flags;
	uint16_t idx;
	struct vring_used_elem ring[];
};

struct vring {
	unsigned num;
	uint32_t notifyid;
	volatile struct vring_desc *desc;
	volatile struct vring_avail *avail;
	volatile struct vring_used *used;
	uint16_t last_avail;
};

static struct vring g_tx;	/* vring0: host RX */
static struct vring g_rx;	/* vring1: host TX */

#define dmb() __asm volatile("dmb" ::: "memory")

static void vring_attach(struct vring *vr, volatile const struct fw_rsc_vdev_vring *rsc)
{
	uintptr_t base = pa_to_va(rsc->da);
	uintptr_t avail;
	uintptr_t used;

	vr->num = rsc->num;
	vr->notifyid = rsc->notifyid;
	vr->desc = (volatile struct vring_desc *)base;
	avail = base + vr->num * sizeof(struct vring_desc);
	vr->avail = (volatile struct vring_avail *)avail;
	used = avail + sizeof(struct vring_avail) + vr->num * sizeof(uint16_t) + sizeof(uint16_t);
	used = (used + rsc->align - 1) & ~((uintptr_t)rsc->align - 1);
	vr->used = (volatile struct vring_used *)used;
	vr->last_avail = 0;
}

/* Next descriptor the host made available, without taking it; -1 if none. */
static int vring_peek_avail(struct vring *vr)
{
	dmb();

	if (vr->last_avail == vr->avail->idx) {
		return -1;
	}

	return vr->avail->ring[vr->last_avail % vr->num];
}

static int vring_get_avail(struct vring *vr)
{
	int head = vring_peek_avail(vr);

	if (head >= 0) {
		vr->last_avail++;
	}

	return head;
}

static void vring_put_used(struct vring *vr, uint16_t head, uint32_t len)
{
	uint16_t idx = vr->used->idx;

	vr->used->ring[idx % vr->num].id = head;
	vr->used->ring[idx % vr->num].len = len;
	dmb();
	vr->used->idx = idx + 1;
	dmb();
}

static void vring_kick(struct vring *vr)
{
	mu_send(vr->notifyid);
}

/* ---- rpmsg ----------------------------------------------------------------- */

struct rpmsg_ns_msg {
	char name[32];
	uint32_t addr;
	uint32_t flags;
};

#define RPMSG_NS_ADDR         53
#define RPMSG_NS_CREATE       0
#define RPMSG_NS_DESTROY      1
#define RPMSG_NS_CREATE_ACK   2

/* Local endpoint addresses: 0x400 + service index. Arbitrary, but they
 * must not collide with the host's dynamic range (>= 1024) or the NS
 * address.
 */
#define SERVICE_ADDR_BASE     0x400

struct service {
	const char *name;
	uint32_t addr;
	uint32_t remote;	/* host endpoint address once bound */
	rp_handler_t handler;
	void (*bound)(void);	/* called once the host endpoint is known */
};

static void handle_ping(const struct rpmsg_hdr *hdr);
static void handle_hello(const struct rpmsg_hdr *hdr);

/* Filled by rp_register_service(). */
#define NSERVICES RP_MAX_SERVICES
static struct service g_services[NSERVICES];
static int g_nservices;
static int g_hello_svc = -1;

int rp_register_service(const char *name, rp_handler_t handler, void (*bound)(void))
{
	if (g_nservices >= NSERVICES) {
		return -1;
	}

	struct service *s = &g_services[g_nservices];

	s->name = name;

	s->addr = SERVICE_ADDR_BASE + (uint32_t)g_nservices;

	s->remote = 0xFFFFFFFFu;

	s->handler = handler;

	s->bound = bound;

	return g_nservices++;
}

int rp_hello_service(void)
{
	return g_hello_svc;
}

static void rp_park(uint32_t state, uint32_t reason)
{
	cm4_status[0] = state;
	cm4_status[1] = reason;

	for (;;) {
		__asm volatile("wfi");
	}
}

int rp_send(uint32_t src, uint32_t dst, const void *payload, uint16_t len)
{
	int head = vring_peek_avail(&g_tx);
	volatile struct vring_desc *d;
	struct rpmsg_hdr *hdr;

	if (head < 0) {
		return -1;
	}

	d = &g_tx.desc[head];

	/* Refuse before taking the descriptor: a used entry with len 0 would
	 * make the host process a stale buffer.
	 */
	if ((uint32_t)len + sizeof(*hdr) > d->len) {
		return -2;
	}

	g_tx.last_avail++;
	hdr = (struct rpmsg_hdr *)pa_to_va(d->addr);

	if (!in_shm((uintptr_t)hdr, d->len)) {
		rp_park(CM4_STATE_BADRSC, 3);
	}

	hdr->src = src;
	hdr->dst = dst;
	hdr->reserved = 0;
	hdr->len = len;
	hdr->flags = 0;
	memcpy(hdr->data, payload, len);

#ifdef CM4_FAULT_INJECT

	if (g_inject_hdr_len) {
		hdr->len = g_inject_hdr_len;
		g_inject_hdr_len = 0;
	}

#endif

	vring_put_used(&g_tx, head, sizeof(*hdr) + len);
	vring_kick(&g_tx);
	cm4_status[3]++;
	return 0;
}

int rp_send_service(int svc, const void *payload, uint16_t len)
{
	if (svc < 0 || svc >= g_nservices || g_services[svc].remote == 0xFFFFFFFFu) {
		return -1;
	}

	return rp_send(g_services[svc].addr, g_services[svc].remote, payload, len);
}

static void ns_send(const struct service *svc, uint32_t flags)
{
	struct rpmsg_ns_msg msg;

	memset(&msg, 0, sizeof(msg));
	str_append(msg.name, sizeof(msg.name), svc->name);
	msg.addr = svc->addr;
	msg.flags = flags;
	rp_send(svc->addr, RPMSG_NS_ADDR, &msg, sizeof(msg));
}

static struct service *service_by_name(const char *name)
{
	for (int i = 0; i < g_nservices; i++) {
		if (strncmp(g_services[i].name, name, 32) == 0) {
			return &g_services[i];
		}
	}

	return 0;
}

static struct service *service_by_addr(uint32_t addr)
{
	for (int i = 0; i < g_nservices; i++) {
		if (g_services[i].addr == addr) {
			return &g_services[i];
		}
	}

	return 0;
}

static void handle_ns(const struct rpmsg_hdr *hdr)
{
	const struct rpmsg_ns_msg *ns = (const void *)hdr->data;
	struct service *svc;

	if (hdr->len != sizeof(*ns)) {
		return;
	}

	svc = service_by_name(ns->name);

	if (!svc) {
		return;
	}

	switch (ns->flags) {
	case RPMSG_NS_CREATE:
	case RPMSG_NS_CREATE_ACK: {
			/* The host announces its endpoint and acks ours; when both arrive
			 * for one binding the callback still fires once.
			 */
			int fresh = svc->remote != ns->addr;
			svc->remote = ns->addr;

			if (ns->flags == RPMSG_NS_CREATE) {
				ns_send(svc, RPMSG_NS_CREATE_ACK);
			}

			if (fresh && svc->bound) {
				svc->bound();
			}

			break;
		}

	case RPMSG_NS_DESTROY:
		svc->remote = 0xFFFFFFFFu;
		break;

	default:
		break;
	}
}

/* NuttX drivers/rpmsg/rpmsg_ping.c wire format and semantics. */
struct rpmsg_ping_msg {
	uint32_t cmd;
	uint32_t len;
	uint64_t cookie;
	uint8_t data[];
} __attribute__((packed));

#define PING_ACK_MASK   0x01u
#define PING_CMD_MASK   0xF0u
#define PING_CMD_REQ    0x00u
#define PING_CMD_RSP    0x20u

static void handle_ping(const struct rpmsg_hdr *hdr)
{
	uint8_t reply[RPMSG_BUF_SIZE];
	struct rpmsg_ping_msg *msg = (void *)reply;
	uint16_t len = hdr->len;

	if (len < sizeof(*msg) || len > sizeof(reply)) {
		return;
	}

	memcpy(reply, hdr->data, len);

	if ((msg->cmd & PING_CMD_MASK) == PING_CMD_RSP || (msg->cmd & PING_ACK_MASK) == 0) {
		return;
	}

	msg->len = (msg->cmd & PING_CMD_MASK) == PING_CMD_REQ ? len : sizeof(*msg);
	msg->cmd = PING_CMD_RSP;
	rp_send(hdr->dst, hdr->src, reply, (uint16_t)msg->len);
}

static void handle_hello(const struct rpmsg_hdr *hdr)
{
	static uint32_t count;
	char reply[RPMSG_BUF_SIZE - sizeof(struct rpmsg_hdr)];
	char text[64];
	uint16_t n = hdr->len < sizeof(text) - 1 ? hdr->len : sizeof(text) - 1;

	memcpy(text, hdr->data, n);
	text[n] = '\0';

	if (strncmp(text, "!fault ", 7) == 0) {
#ifdef CM4_FAULT_INJECT
		fault_inject(text + 7, reply, sizeof(reply));
#else
		reply[0] = '\0';
		str_append(reply, sizeof(reply), "fault injection not built (FAULT_INJECT=1)");
#endif

	} else {
		reply[0] = '\0';
		str_append(reply, sizeof(reply), "hello from cm4 #");
		str_append_u32(reply, sizeof(reply), ++count);
		str_append(reply, sizeof(reply), " (got: \"");
		str_append(reply, sizeof(reply), text);
		str_append(reply, sizeof(reply), "\")");
	}

	/* The TX ring may be full right after a burst; the host drains it
	 * concurrently, so a short bounded retry gets the reply out.
	 */
	for (int i = 0; i < 100000; i++) {
		if (rp_send(hdr->dst, hdr->src, reply, (uint16_t)(strlen(reply) + 1)) != -1) {
			break;
		}
	}
}

void rp_init(void)
{
	rp_register_service("rpmsg-ping", handle_ping, 0);
	g_hello_svc = rp_register_service("rpmsg-hello", handle_hello, 0);

	while ((g_rsc.rpmsg_vdev.status & VIRTIO_STATUS_DRIVER_OK) == 0) {
	}

	dmb();

	if (g_rsc.hdr.ver != 1 || g_rsc.rpmsg_vdev.id != VIRTIO_ID_RPMSG || g_rsc.rpmsg_vdev.num_of_vrings != 2) {
		rp_park(CM4_STATE_BADRSC, 1);
	}

	if (!in_shm(pa_to_va(g_rsc.rpmsg_vring0.da), 512) || !in_shm(pa_to_va(g_rsc.rpmsg_vring1.da), 512)) {
		rp_park(CM4_STATE_BADRSC, 2);
	}

	vring_attach(&g_tx, &g_rsc.rpmsg_vring0);
	vring_attach(&g_rx, &g_rsc.rpmsg_vring1);

	for (int i = 0; i < g_nservices; i++) {
		ns_send(&g_services[i], RPMSG_NS_CREATE);
	}
}

void rp_process(void)
{
	int head;
	int consumed = 0;

	while ((head = vring_get_avail(&g_rx)) >= 0) {
		volatile struct vring_desc *d = &g_rx.desc[head];
		const struct rpmsg_hdr *hdr = (const void *)pa_to_va(d->addr);

		if (!in_shm((uintptr_t)hdr, d->len)) {
			rp_park(CM4_STATE_BADRSC, 4);
		}

		if (sizeof(*hdr) + hdr->len <= d->len) {
			if (hdr->dst == RPMSG_NS_ADDR) {
				handle_ns(hdr);

			} else {
				struct service *svc = service_by_addr(hdr->dst);

				if (svc && svc->handler) {
					svc->handler(hdr);
				}
			}
		}

		/* The host reads the buffer size back from used->len when it reuses
		 * the descriptor, so return the full descriptor length like the
		 * OpenAMP remote does, not the bytes we consumed.
		 */
		uint32_t used_len = d->len;
#ifdef CM4_FAULT_INJECT

		if (g_inject_used_len != UINT32_MAX) {
			used_len = g_inject_used_len;
			g_inject_used_len = UINT32_MAX;
		}

#endif
		vring_put_used(&g_rx, head, used_len);
		consumed++;
	}

	if (consumed) {
		/* Let the host reclaim its TX buffers. */
		vring_kick(&g_rx);
	}
}
