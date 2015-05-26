/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "uORBDevices_posix.hpp"
#include "uORBUtils.hpp"
#include <stdlib.h>


uORB::DeviceNode::SubscriberData  *uORB::DeviceNode::filp_to_sd(device::file_t *filp)
{
  uORB::DeviceNode::SubscriberData *sd;
  if (filp) {
    sd = (uORB::DeviceNode::SubscriberData *)(filp->priv);
  }
  else {
    sd = 0;
  }
  return sd;
}

uORB::DeviceNode::DeviceNode(const struct orb_metadata *meta, const char *name, const char *path, int priority) :
  VDev(name, path),
  _meta(meta),
  _data(nullptr),
  _last_update(0),
  _generation(0),
  _publisher(0),
  _priority(priority)
{
  // enable debug() calls
  //_debug_enabled = true;
}

uORB::DeviceNode::~DeviceNode()
{
  if (_data != nullptr)
    delete[] _data;

}

int
uORB::DeviceNode::open(device::file_t *filp)
{
  int ret;

  /* is this a publisher? */
  if (filp->flags == PX4_F_WRONLY) {

    /* become the publisher if we can */
    lock();

    if (_publisher == 0) {
      _publisher = getpid();
      ret = PX4_OK;

    } else {
      ret = -EBUSY;
    }

    unlock();

    /* now complete the open */
    if (ret == PX4_OK) {
      ret = VDev::open(filp);

      /* open failed - not the publisher anymore */
      if (ret != PX4_OK)
        _publisher = 0;
    }

    return ret;
  }

  /* is this a new subscriber? */
  if (filp->flags == PX4_F_RDONLY) {

    /* allocate subscriber data */
    SubscriberData *sd = new SubscriberData;

    if (nullptr == sd)
      return -ENOMEM;

    memset(sd, 0, sizeof(*sd));

    /* default to no pending update */
    sd->generation = _generation;

    /* set priority */
    sd->priority = _priority;

    filp->priv = (void *)sd;

    ret = VDev::open(filp);

    if (ret != PX4_OK) {
      warnx("ERROR: VDev::open failed\n");
      delete sd;
    }

    //warnx("uORB::DeviceNode::Open: fd = %d flags = %d, priv = %p cdev = %p\n", filp->fd, filp->flags, filp->priv, filp->cdev);
    return ret;
  }

  /* can only be pub or sub, not both */
  return -EINVAL;
}

int
uORB::DeviceNode::close(device::file_t *filp)
{
  //warnx("uORB::DeviceNode::close fd = %d", filp->fd);
  /* is this the publisher closing? */
  if (getpid() == _publisher) {
    _publisher = 0;

  } else {
    SubscriberData *sd = filp_to_sd(filp);

    if (sd != nullptr) {
      hrt_cancel(&sd->update_call);
      delete sd;
    }
  }

  return VDev::close(filp);
}

ssize_t
uORB::DeviceNode::read(device::file_t *filp, char *buffer, size_t buflen)
{
  //warnx("uORB::DeviceNode::read fd = %d\n", filp->fd);
  SubscriberData *sd = (SubscriberData *)filp_to_sd(filp);

  /* if the object has not been written yet, return zero */
  if (_data == nullptr)
    return 0;

  /* if the caller's buffer is the wrong size, that's an error */
  if (buflen != _meta->o_size)
    return -EIO;

  /*
   * Perform an atomic copy & state update
   */
  lock();

  /* if the caller doesn't want the data, don't give it to them */
  if (nullptr != buffer)
    memcpy(buffer, _data, _meta->o_size);

  /* track the last generation that the file has seen */
  sd->generation = _generation;

  /* set priority */
  sd->priority = _priority;

  /*
   * Clear the flag that indicates that an update has been reported, as
   * we have just collected it.
   */
  sd->update_reported = false;

  unlock();

  return _meta->o_size;
}

ssize_t
uORB::DeviceNode::write(device::file_t *filp, const char *buffer, size_t buflen)
{
  //warnx("uORB::DeviceNode::write filp = %p (null is normal)", filp);
  /*
   * Writes are legal from interrupt context as long as the
   * object has already been initialised from thread context.
   *
   * Writes outside interrupt context will allocate the object
   * if it has not yet been allocated.
   *
   * Note that filp will usually be NULL.
   */
  if (nullptr == _data) {
    lock();

    /* re-check size */
    if (nullptr == _data)
      _data = new uint8_t[_meta->o_size];

    unlock();

    /* failed or could not allocate */
    if (nullptr == _data)
      return -ENOMEM;
  }

  /* If write size does not match, that is an error */
  if (_meta->o_size != buflen)
    return -EIO;

  /* Perform an atomic copy. */
  lock();
  memcpy(_data, buffer, _meta->o_size);
  unlock();

  /* update the timestamp and generation count */
  _last_update = hrt_absolute_time();
  _generation++;

  /* notify any poll waiters */
  poll_notify(POLLIN);

  return _meta->o_size;
}

int
uORB::DeviceNode::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
  //warnx("uORB::DeviceNode::ioctl fd = %d cmd = %d", filp->fd, cmd);
  SubscriberData *sd = filp_to_sd(filp);

  switch (cmd) {
  case ORBIOCLASTUPDATE:
    *(hrt_abstime *)arg = _last_update;
    return PX4_OK;

  case ORBIOCUPDATED:
    *(bool *)arg = appears_updated(sd);
    return PX4_OK;

  case ORBIOCSETINTERVAL:
    sd->update_interval = arg;
    return PX4_OK;

  case ORBIOCGADVERTISER:
    *(uintptr_t *)arg = (uintptr_t)this;
    return PX4_OK;

  case ORBIOCGPRIORITY:
    *(int *)arg = sd->priority;
    return PX4_OK;

  default:
    /* give it to the superclass */
    return VDev::ioctl(filp, cmd, arg);
  }
}

ssize_t
uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t handle, const void *data)
{
  //warnx("uORB::DeviceNode::publish meta = %p", meta);

  if (handle == nullptr) {
    warnx("uORB::DeviceNode::publish called with invalid handle");
    errno = EINVAL;
    return ERROR;
  }

  uORB::DeviceNode *devnode = (uORB::DeviceNode *)handle;
  int ret;

  /* this is a bit risky, since we are trusting the handle in order to deref it */
  if (devnode->_meta != meta) {
    errno = EINVAL;
    return ERROR;
  }

  /* call the devnode write method with no file pointer */
  ret = devnode->write(nullptr, (const char *)data, meta->o_size);

  if (ret < 0)
    return ERROR;

  if (ret != (int)meta->o_size) {
    errno = EIO;
    return ERROR;
  }

  return PX4_OK;
}

pollevent_t
uORB::DeviceNode::poll_state(device::file_t *filp)
{
  //warnx("uORB::DeviceNode::poll_state fd = %d", filp->fd);
  SubscriberData *sd = filp_to_sd(filp);

  /*
   * If the topic appears updated to the subscriber, say so.
   */
  if (appears_updated(sd))
    return POLLIN;

  return 0;
}

void
uORB::DeviceNode::poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events)
{
  //warnx("uORB::DeviceNode::poll_notify_one fds = %p fds->priv = %p", fds, fds->priv);
  SubscriberData *sd = filp_to_sd((device::file_t *)fds->priv);

  /*
   * If the topic looks updated to the subscriber, go ahead and notify them.
   */
  if (appears_updated(sd))
    VDev::poll_notify_one(fds, events);
}

bool
uORB::DeviceNode::appears_updated(SubscriberData *sd)
{
  //warnx("uORB::DeviceNode::appears_updated sd = %p", sd);
  /* assume it doesn't look updated */
  bool ret = false;

  /* check if this topic has been published yet, if not bail out */
  if (_data == nullptr) {
    ret = false;
    goto out;
  }

  /*
   * If the subscriber's generation count matches the update generation
   * count, there has been no update from their perspective; if they
   * don't match then we might have a visible update.
   */
  while (sd->generation != _generation) {

    /*
     * Handle non-rate-limited subscribers.
     */
    if (sd->update_interval == 0) {
      ret = true;
      break;
    }

    /*
     * If we have previously told the subscriber that there is data,
     * and they have not yet collected it, continue to tell them
     * that there has been an update.  This mimics the non-rate-limited
     * behaviour where checking / polling continues to report an update
     * until the topic is read.
     */
    if (sd->update_reported) {
      ret = true;
      break;
    }

// FIXME - the calls to hrt_called and hrt_call_after seem not to work in the
//         POSIX build
#ifndef __PX4_POSIX
    /*
     * If the interval timer is still running, the topic should not
     * appear updated, even though at this point we know that it has.
     * We have previously been through here, so the subscriber
     * must have collected the update we reported, otherwise
     * update_reported would still be true.
     */
    if (!hrt_called(&sd->update_call))
      break;

    /*
     * Make sure that we don't consider the topic to be updated again
     * until the interval has passed once more by restarting the interval
     * timer and thereby re-scheduling a poll notification at that time.
     */
    hrt_call_after(&sd->update_call,
             sd->update_interval,
             &uORB::DeviceNode::update_deferred_trampoline,
             (void *)this);
#endif

    /*
     * Remember that we have told the subscriber that there is data.
     */
    sd->update_reported = true;
    ret = true;

    break;
  }

out:
  /* consider it updated */
  return ret;
}

void
uORB::DeviceNode::update_deferred()
{
  /*
   * Instigate a poll notification; any subscribers whose intervals have
   * expired will be woken.
   */
  poll_notify(POLLIN);
}

void
uORB::DeviceNode::update_deferred_trampoline(void *arg)
{
  uORB::DeviceNode *node = (uORB::DeviceNode *)arg;

  node->update_deferred();
}

uORB::DeviceMaster::DeviceMaster(Flavor f) :
  VDev((f == PUBSUB) ? "obj_master" : "param_master",
       (f == PUBSUB) ? TOPIC_MASTER_DEVICE_PATH : PARAM_MASTER_DEVICE_PATH),
  _flavor(f)
{
  // enable debug() calls
  //_debug_enabled = true;

}

uORB::DeviceMaster::~DeviceMaster()
{
}

int
uORB::DeviceMaster::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
  int ret;

  switch (cmd) {
  case ORBIOCADVERTISE: {
      const struct orb_advertdata *adv = (const struct orb_advertdata *)arg;
      const struct orb_metadata *meta = adv->meta;
      const char *objname;
      const char *devpath;
      char nodepath[orb_maxpath];
      uORB::DeviceNode *node;

      /* set instance to zero - we could allow selective multi-pubs later based on value */
      if (adv->instance != nullptr) {
        *(adv->instance) = 0;
      }

      /* construct a path to the node - this also checks the node name */
      ret = uORB::Utils::node_mkpath(nodepath, _flavor, meta, adv->instance);

      if (ret != PX4_OK) {
        return ret;
      }

      /* ensure that only one advertiser runs through this critical section */
      lock();

      ret = ERROR;

      /* try for topic groups */
      const unsigned max_group_tries = (adv->instance != nullptr) ? ORB_MULTI_MAX_INSTANCES : 1;
      unsigned group_tries = 0;
      do {
        /* if path is modifyable change try index */
        if (adv->instance != nullptr) {
          /* replace the number at the end of the string */
          nodepath[strlen(nodepath) - 1] = '0' + group_tries;
          *(adv->instance) = group_tries;
        }

        /* driver wants a permanent copy of the node name, so make one here */
        objname = strdup(meta->o_name);

        if (objname == nullptr) {
          return -ENOMEM;
        }

        /* driver wants a permanent copy of the path, so make one here */
        devpath = strdup(nodepath);

        if (devpath == nullptr) {
          // FIXME - looks like we leaked memory here for objname
          return -ENOMEM;
        }

        /* construct the new node */
        node = new uORB::DeviceNode(meta, objname, devpath, adv->priority);

        /* if we didn't get a device, that's bad */
        if (node == nullptr) {
          unlock();

          // FIXME - looks like we leaked memory here for devpath and objname
          return -ENOMEM;
        }

        /* initialise the node - this may fail if e.g. a node with this name already exists */
        ret = node->init();

        /* if init failed, discard the node and its name */
        if (ret != PX4_OK) {
          delete node;
          free((void *)objname);
          free((void *)devpath);
        }

        group_tries++;

      } while (ret != PX4_OK && (group_tries < max_group_tries));

      if (group_tries > max_group_tries) {
        ret = -ENOMEM;
      }

      /* the file handle for the driver has been created, unlock */
      unlock();

      return ret;
    }

  default:
    /* give it to the superclass */
    return VDev::ioctl(filp, cmd, arg);
  }
}

