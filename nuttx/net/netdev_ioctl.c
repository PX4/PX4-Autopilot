/****************************************************************************
 * net/netdev_ioctl.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#include <nuttx/config.h>
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <nuttx/net/uip/uip-arch.h>
#include <nuttx/net/uip/uip.h>

#ifdef CONFIG_NET_IGMP
#  include "sys/sockio.h"
#  include "nuttx/net/uip/uip-igmp.h"
#endif

#include "net_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
# define AF_INETX AF_INET6
#else
# define AF_INETX AF_INET
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioctl_getipaddr
 *
 * Description:
 *   Copy IP addresses from device structure to user memory.
 *
 * Input Parameters:
 *   outaddr - Pointer to the user-provided memory to receive the address.
 *     Actual type may be either 'struct sockaddr' (IPv4 only) or type
 *     'struct sockaddr_storage' (both IPv4 and IPv6).
 *   inaddr - The source IP adress in the device structure.
 *
 ****************************************************************************/

static void ioctl_getipaddr(FAR void *outaddr, FAR const uip_ipaddr_t *inaddr)
{
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *dest = (FAR struct sockaddr_in6 *)outaddr;
  dest->sin_family              = AF_INET6;
  dest->sin_port                = 0;
  memcpy(dest->sin6_addr.in6_u.u6_addr8, inaddr, 16);
#else
  FAR struct sockaddr_in *dest  = (FAR struct sockaddr_in *)outaddr;
  dest->sin_family              = AF_INET;
  dest->sin_port                = 0;
  dest->sin_addr.s_addr         = *inaddr;
#endif
}

/****************************************************************************
 * Name: ioctl_setipaddr
 *
 * Description:
 *   Copy IP addresses from user memory into the device structure
 *
 * Input Parameters:
 *   outaddr - Pointer to the source IP address in the device structure.
 *   inaddr - Pointer to the user-provided memory to containing the new IP
 *     address.  Actual type may be either 'struct sockaddr' (IPv4 only) or
 *     type 'struct sockaddr_storage' (both IPv4 and IPv6).
 *
 ****************************************************************************/

static void ioctl_setipaddr(FAR uip_ipaddr_t *outaddr, FAR const void *inaddr)
{
#ifdef CONFIG_NET_IPv6
  FAR const struct sockaddr_in6 *src = (FAR const struct sockaddr_in6 *)inaddr;
  memcpy(outaddr, src->sin6_addr.in6_u.u6_addr8, 16);
#else
  FAR const struct sockaddr_in *src = (FAR const struct sockaddr_in *)inaddr;
  *outaddr = src->sin_addr.s_addr;
#endif
}

/****************************************************************************
 * Name: ioctl_ifup / ioctl_ifdown
 *
 * Description:
 *   Bring the interface up/down
 *
 ****************************************************************************/

static void ioctl_ifup(FAR struct uip_driver_s *dev)
{
  /* Make sure that the device supports the d_ifup() method */

  if (dev->d_ifup)
    {
      /* Is the interface already up? */

      if ((dev->d_flags & IFF_RUNNING) == 0)
        {
          /* No, bring the interface up now */

          if (dev->d_ifup(dev) == OK)
            {
              /* Mark the interface as up */

              dev->d_flags |= IFF_RUNNING;
            }
        }
    }
}

static void ioctl_ifdown(FAR struct uip_driver_s *dev)
{
  /* Make sure that the device supports the d_ifdown() method */

  if (dev->d_ifdown)
    {
      /* Is the interface already down? */

      if ((dev->d_flags & IFF_RUNNING) != 0)
        {
          /* No, take the interface down now */

          if (dev->d_ifdown(dev) == OK)
            {
              /* Mark the interface as down */

              dev->d_flags &= ~IFF_RUNNING;
            }
        }
    }
}

/****************************************************************************
 * Name: netdev_ioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Parameters:
 *   psock    Socket structure
 *   dev      Ethernet driver device structure
 *   cmd      The ioctl command
 *   req      The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

static int netdev_ifrioctl(FAR struct socket *psock, int cmd, struct ifreq *req)
{
  FAR struct uip_driver_s *dev;
  int ret = OK;

  nvdbg("cmd: %d\n", cmd);

  /* Find the network device associated with the device name
   * in the request data.
   */

  dev = netdev_findbyname(req->ifr_name);
  if (!dev)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Execute the command */

  switch (cmd)
    {
      case SIOCGIFADDR:  /* Get IP address */
        {
          ioctl_getipaddr(&req->ifr_addr, &dev->d_ipaddr);
        }
        break;

      case SIOCSIFADDR:  /* Set IP address */
        {
          ioctl_ifdown(dev);
          ioctl_setipaddr(&dev->d_ipaddr, &req->ifr_addr);
          ioctl_ifup(dev);
        }
        break;

      case SIOCGIFDSTADDR:  /* Get P-to-P address */
        {
          ioctl_getipaddr(&req->ifr_dstaddr, &dev->d_draddr);
        }
        break;

      case SIOCSIFDSTADDR:  /* Set P-to-P address */
        {
          ioctl_setipaddr(&dev->d_draddr, &req->ifr_dstaddr);
        }
        break;

      case SIOCGIFNETMASK:  /* Get network mask */
        {
          ioctl_getipaddr(&req->ifr_addr, &dev->d_netmask);
        }
        break;

      case SIOCSIFNETMASK:  /* Set network mask */
        {
          ioctl_setipaddr(&dev->d_netmask, &req->ifr_addr);
        }
        break;

      case SIOCGIFMTU:  /* Get MTU size */
        {
          req->ifr_mtu = CONFIG_NET_BUFSIZE;
        }
        break;

      case SIOCSIFFLAGS:  /* Sets the interface flags */
        {
          /* Is this a request to bring the interface up? */

          if (req->ifr_flags & IF_FLAG_IFUP)
            {
              /* Yes.. bring the interface up */

              ioctl_ifup(dev);
            }

          /* Is this a request to take the interface down? */

          else if (req->ifr_flags & IF_FLAG_IFDOWN)
            {
              /* Yes.. take the interface down */

              ioctl_ifdown(dev);
            }
        }
        break;

      case SIOCGIFFLAGS:  /* Gets the interface flags */
        {
          req->ifr_flags = 0;

          /* Is the interface running? */

          if (dev->d_flags & IFF_RUNNING)
            {
              /* Yes.. report interface up */

              req->ifr_flags |= IF_FLAG_IFUP;
            }
          else
            {
              /* No.. report interface down */

              req->ifr_flags |= IF_FLAG_IFDOWN;
            }
        }
        break;

      /* MAC address operations only make sense if Ethernet is supported */

#ifdef CONFIG_NET_ETHERNET
      case SIOCGIFHWADDR:  /* Get hardware address */
        {
          req->ifr_hwaddr.sa_family = AF_INETX;
          memcpy(req->ifr_hwaddr.sa_data, dev->d_mac.ether_addr_octet, IFHWADDRLEN);
        }
        break;

      case SIOCSIFHWADDR:  /* Set hardware address -- will not take effect until ifup */
        {
          req->ifr_hwaddr.sa_family = AF_INETX;
          memcpy(dev->d_mac.ether_addr_octet, req->ifr_hwaddr.sa_data, IFHWADDRLEN);
        }
        break;
#endif

      case SIOCDIFADDR:  /* Delete IP address */
        {
          ioctl_ifdown(dev);
          memset(&dev->d_ipaddr, 0, sizeof(uip_ipaddr_t));
        }
        break;

      case SIOCGIFCOUNT:  /* Get number of devices */
        {
          req->ifr_count = netdev_count();
          ret = -ENOSYS;
        }
        break;

      case SIOCGIFBRDADDR:  /* Get broadcast IP address */
      case SIOCSIFBRDADDR:  /* Set broadcast IP address */
        {
          ret = -ENOSYS;
        }
        break;

#ifdef CONFIG_NET_ARPIOCTLS
      case SIOCSARP:  /* Set a ARP mapping */
      case SIOCDARP:  /* Delete an ARP mapping */
      case SIOCGARP:  /* Get an ARP mapping */
# error "IOCTL Commands not implemented"
#endif

      default:
        {
          ret = -EINVAL;
        }
        break;;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: netdev_imsfioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Parameters:
 *   psock    Socket structure
 *   dev      Ethernet driver device structure
 *   cmd      The ioctl command
 *   imsf      The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int netdev_imsfioctl(FAR struct socket *psock, int cmd, struct ip_msfilter *imsf)
{
  FAR struct uip_driver_s *dev;
  int ret = OK;

  nvdbg("cmd: %d\n", cmd);

  /* Find the network device associated with the device name
   * in the request data.
   */

  dev = netdev_findbyname(imsf->imsf_name);
  if (!dev)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Execute the command */

  switch (cmd)
    {
      case SIOCSIPMSFILTER:  /* Set source filter content */
        {
          if (imsf->imsf_fmode == MCAST_INCLUDE)
            {
              ret = igmp_joingroup(dev, &imsf->imsf_multiaddr);
            }
          else
            {
              DEBUGASSERT(imsf->imsf_fmode == MCAST_EXCLUDE);
              ret = igmp_leavegroup(dev, &imsf->imsf_multiaddr);
            }
        }
        break;

      case SIOCGIPMSFILTER:  /* Retrieve source filter addresses */
      default:
        ret = -EINVAL;
        break;
    }

errout:
  return ret;
}
#endif

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_ioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Parameters:
 *   sockfd   Socket descriptor of device
 *   cmd      The ioctl command
 *   req      The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   On a failure, -1 is returned with errno set appropriately
 *
 *   EBADF
 *     'sockfd' is not a valid descriptor.
 *   EFAULT
 *     'req' references an inaccessible memory area.
 *   EINVAL
 *     'cmd' or 'req' is not valid.
 *   ENOTTY
 *     'sockfd' is not associated with a network device.
 *   ENOTTY
 *      The specified request does not apply to the kind of object that the
 *      descriptor 'sockfd' references.
 *
 ****************************************************************************/

int netdev_ioctl(int sockfd, int cmd, unsigned long arg)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  int ret;

  /* Check if this is a valid command.  In all cases, arg is a pointer that has
   * been cast to unsigned long.  Verify that the value of the to-be-pointer is
   * non-NULL.
   */

  if (!_SIOCVALID(cmd) || !arg)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      ret = -EBADF;
      goto errout;
    }

  /* Execute the command */

  ret = netdev_ifrioctl(psock, cmd, (FAR struct ifreq*)arg);
#ifdef CONFIG_NET_IGMP
  if (ret == -EINVAL)
    {
      ret = netdev_imsfioctl(psock, cmd, (FAR struct ip_msfilter*)arg);
    }
#endif

  /* Check for success or failure */

  if (ret >= 0)
    {
      return ret;
    }

/* On failure, set the errno and return -1 */

errout:
  errno = -ret;
  return ERROR;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
