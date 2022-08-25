/****************************************************************************
 * boards/arm/imxrt/imxrt1170-evk/src/imxrt_ethernet.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Force verbose debug on in this file only to support unit-level testing. */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  undef  CONFIG_DEBUG_INFO
#  define CONFIG_DEBUG_INFO 1
#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include "arm_internal.h"

#include "imxrt_gpio.h"
#include "imxrt_enet.h"

#include "board_config.h"

#ifdef CONFIG_IMXRT_ENET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_ENET_DEVNAME "eth0"

/* Debug ********************************************************************/

/* Extra, in-depth debug output that is only available if
 * CONFIG_NETDEV_PHY_DEBUG us defined.
 */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  define phyerr    _err
#  define phywarn   _warn
#  define phyinfo   _info
#else
#  define phyerr(x...)
#  define phywarn(x...)
#  define phyinfo(x...)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_enet_phy_enable
 ****************************************************************************/

#if 1
static void imxrt_enet_phy_enable(bool enable)
{
	phyinfo("IRQ%d: enable=%d\n", GPIO_ENET_INT, enable);

	if (enable) {
		up_enable_irq(GPIO_ENET_IRQ);

	} else {
		up_disable_irq(GPIO_ENET_IRQ);
	}
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imxrt_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.
 *   If CONFIG_IMXRT_ENET_PHYINIT is defined in the configuration then the
 *   board specific logic must provide imxrt_phyinitialize();
 *   The i.MX RT Ethernet driver will call this function one time before it
 *   first uses the PHY.
 *
 * Input Parameters:
 *   intf - Always zero for now.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int imxrt_phy_boardinitialize(int intf)
{
#ifdef CONFIG_IMXRT_GPIO1_0_15_IRQ
	/* Configure the PHY interrupt pin */

	phyinfo("Configuring interrupt: %08x\n", GPIO_ENET_INT);
	imxrt_config_gpio(GPIO_ENET_INT);
#endif

	/* Configure the PHY reset pin.
	 *
	 * The #RST uses inverted logic.  The initial value of zero will put the
	 * PHY into the reset state.
	 */

	phyinfo("Configuring reset: %08x\n", GPIO_ENET_RST);
	imxrt_config_gpio(GPIO_ENET_RST);
	imxrt_config_gpio(GPIO_ENET_INT);

	imxrt_gpio_write(GPIO_ENET_INT, true);
	imxrt_gpio_write(GPIO_ENET_RST, false);

	up_mdelay(10);

	/* Take the PHY out of reset. */

	imxrt_gpio_write(GPIO_ENET_RST, true);

	up_mdelay(1);

	/* Workaround for the IOMUX Daisy support */

	putreg32(0x1, IMXRT_INPUT_ENET_IPG_CLK_RMII);
	putreg32(0x1, IMXRT_INPUT_ENET_MDIO);
	putreg32(0x1, IMXRT_INPUT_ENET_RXDATA0);
	putreg32(0x1, IMXRT_INPUT_ENET_RXDATA1);
	putreg32(0x1, IMXRT_INPUT_ENET_RXEN);
	putreg32(0x1, IMXRT_INPUT_ENET_RXERR);

	return OK;
}

/****************************************************************************
 * Name: arch_phy_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a PHY interrupt occurs.  This function both attaches
 *   the interrupt handler and enables the interrupt if 'handler' is non-
 *   NULL.  If handler is NULL, then the interrupt is detached and disabled
 *   instead.
 *
 *   The PHY interrupt is always disabled upon return.  The caller must
 *   call back through the enable function point to control the state of
 *   the interrupt.
 *
 *   This interrupt may or may not be available on a given platform depending
 *   on how the network hardware architecture is implemented.  In a typical
 *   case, the PHY interrupt is provided to board-level logic as a GPIO
 *   interrupt (in which case this is a board-specific interface and really
 *   should be called board_phy_irq()); In other cases, the PHY interrupt
 *   may be cause by the chip's MAC logic (in which case arch_phy_irq()) is
 *   an appropriate name.  Other other boards, there may be no PHY interrupts
 *   available at all.  If client attachable PHY interrupts are available
 *   from the board or from the chip, then CONFIG_ARCH_PHY_INTERRUPT should
 *   be defined to indicate that fact.
 *
 *   Typical usage:
 *   a. OS service logic (not application logic*) attaches to the PHY
 *      PHY interrupt and enables the PHY interrupt.
 *   b. When the PHY interrupt occurs:  (1) the interrupt should be
 *      disabled and () work should be scheduled on the worker thread (or
 *      perhaps a dedicated application thread).
 *   c. That worker thread should use the SIOCGMIIPHY, SIOCGMIIREG,
 *      and SIOCSMIIREG ioctl calls** to communicate with the PHY,
 *      determine what network event took place (Link Up/Down?), and
 *      take the appropriate actions.
 *   d. It should then interact the PHY to clear any pending
 *      interrupts, then re-enable the PHY interrupt.
 *
 *    * This is an OS internal interface and should not be used from
 *      application space.  Rather applications should use the SIOCMIISIG
 *      ioctl to receive a signal when a PHY event occurs.
 *   ** This interrupt is really of no use if the Ethernet MAC driver
 *      does not support these ioctl calls.
 *
 * Input Parameters:
 *   intf    - Identifies the network interface.  For example "eth0".  Only
 *             useful on platforms that support multiple Ethernet interfaces
 *             and, hence, multiple PHYs and PHY interrupts.
 *   handler - The client interrupt handler to be invoked when the PHY
 *             asserts an interrupt.  Must reside in OS space, but can
 *             signal tasks in user space.  A value of NULL can be passed
 *             in order to detach and disable the PHY interrupt.
 *   arg     - The argument that will accompany the interrupt
 *   enable  - A function pointer that be unused to enable or disable the
 *             PHY interrupt.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#if 1
int arch_phy_irq(const char *intf, xcpt_t handler, void *arg,
		 phy_enable_t *enable)
{
	irqstate_t flags;
	phy_enable_t enabler;
	int irq;

	DEBUGASSERT(intf);

	ninfo("%s: handler=%p\n", intf, handler);
	phyinfo("EMAC: devname=%s\n", IMXRT_ENET_DEVNAME);

	if (strcmp(intf, IMXRT_ENET_DEVNAME) == 0) {
		irq     = GPIO_ENET_IRQ;
		enabler = imxrt_enet_phy_enable;

	} else {
		nerr("ERROR: Unsupported interface: %s\n", intf);
		return -EINVAL;
	}

	/* Disable interrupts until we are done.  This guarantees that the
	 * following operations are atomic.
	 */

	flags = spin_lock_irqsave(NULL);

	/* Configure the interrupt */

	if (handler) {
		/* The interrupt pin has already been configured as an interrupting
		 * input (by imxrt_phy_boardinitialize() above).
		 *
		 * Attach the new button handler.
		 */

		phyinfo("Attach IRQ%d\n", irq);
		irq_attach(irq, handler, arg);

	} else {
		phyinfo("Detach IRQ%d\n", irq);
		irq_detach(irq);
		enabler = NULL;
	}

	/* Return with the interrupt disabled in either case */

	up_disable_irq(GPIO_ENET_IRQ);

	/* Return the enabling function pointer */

	if (enable) {
		*enable = enabler;
	}

	/* Return the old handler (so that it can be restored) */

	spin_unlock_irqrestore(NULL, flags);
	return OK;
}
#endif /* CONFIG_IMXRT_GPIO1_0_15_IRQ */

#endif /* CONFIG_IMXRT_ENET */
