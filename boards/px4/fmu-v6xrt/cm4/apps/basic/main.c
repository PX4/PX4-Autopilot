#include "mu.h"
#include "rpmsg_remote.h"
#include "status.h"

/* Runtime only: rpmsg-ping, rpmsg-hello and fault injection. The image
 * the rpmsg variant ships until a layer selects another application.
 */
int main(void)
{
	mu_init();
	cm4_status[0] = CM4_STATE_MU;

	rp_init();
	cm4_status[0] = CM4_STATE_READY;

	for (;;) {
		if (mu_rx_pending) {
			mu_rx_pending = 0;
			cm4_status[2]++;
			rp_process();

		} else {
			__asm volatile("wfi");
		}
	}
}
