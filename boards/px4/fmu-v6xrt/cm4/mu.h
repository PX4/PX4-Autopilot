#pragma once

#include <stdint.h>

/* MU-B, the CM4 side of the RT1176 Messaging Unit. Channel 0 carries
 * virtqueue kicks; the CM7 side is arch/arm/src/imxrt/imxrt_mu.c.
 */

void mu_init(void);
void mu_send(uint32_t msg);

/* Set from the MU interrupt when a kick arrived; cleared by the consumer. */
extern volatile uint32_t mu_rx_pending;
