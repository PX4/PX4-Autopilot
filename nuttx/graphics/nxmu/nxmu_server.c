/****************************************************************************
 * graphics/nxmu/nxmu_server.c
 *
 *   Copyright (C) 2008-2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include "nxfe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_disconnect
 ****************************************************************************/

static inline void nxmu_disconnect(FAR struct nxfe_conn_s *conn)
{
  struct nxclimsg_disconnected_s outmsg;
  int ret;

  /* Send the handshake message back to the client */

  outmsg.msgid = NX_CLIMSG_DISCONNECTED;

  ret = nxmu_sendclient(conn, &outmsg, sizeof(struct nxclimsg_disconnected_s));
  if (ret < 0)
    {
      gdbg("nxmu_sendclient failed: %d\n", errno);
    }

  /* Close the outgoing client message queue */

  (void)mq_close(conn->swrmq);
}

/****************************************************************************
 * Name: nxmu_connect
 ****************************************************************************/

static inline void nxmu_connect(FAR struct nxfe_conn_s *conn)
{
  char mqname[NX_CLIENT_MXNAMELEN];
  struct nxclimsg_connected_s outmsg;
  int ret;

  /* Create the client MQ name */

  sprintf(mqname, NX_CLIENT_MQNAMEFMT, conn->cid);

  /* Open the client MQ -- this should have already been created by the client */

  conn->swrmq  = mq_open(mqname, O_WRONLY);
  if (conn->swrmq == (mqd_t)-1)
    {
      gdbg("mq_open(%s) failed: %d\n", mqname, errno);
      outmsg.msgid = NX_CLIMSG_DISCONNECTED;
    }

  /* Send the handshake message back to the client */

  outmsg.msgid = NX_CLIMSG_CONNECTED;
  ret = nxmu_sendclient(conn, &outmsg, sizeof(struct nxclimsg_connected_s));
  if (ret < 0)
    {
      gdbg("nxmu_sendclient failed: %d\n", errno);
    }
}

/****************************************************************************
 * Name: nxmu_shutdown
 ****************************************************************************/

static inline void nxmu_shutdown(FAR struct nxfe_state_s *fe)
{
  FAR struct nxbe_window_s *wnd;

  /* Inform all of the clients in the window list that the server is
   * exit-ting.  Notes: (1) that the following loop will probably attempt to
   * disconnect clients multiple times because one client may have multiple
   * windows:  The first disconnect will fail; subsequent will return errors
   * that are ignored.  (2) The final window to be disconnected will be the
   * background window, thus close all of the servers message queues.
   */

  for (wnd = fe->be.topwnd; wnd; wnd = wnd->below)
    {
       (void)nxmu_disconnect(wnd->conn);
    }
}

/****************************************************************************
 * Name: nxmu_blocked
 ****************************************************************************/

static inline void nxmu_blocked(FAR struct nxbe_window_s *wnd, FAR void *arg)
{
  struct nxclimsg_blocked_s outmsg;
  int ret;

  outmsg.msgid = NX_CLIMSG_BLOCKED;
  outmsg.wnd   = wnd;
  outmsg.arg   = arg;

  ret = nxmu_sendclient(wnd->conn, &outmsg, sizeof(struct nxclimsg_blocked_s));
  if (ret < 0)
    {
      gdbg("nxmu_sendclient failed: %d\n", errno);
    }
}

/****************************************************************************
 * Name: nxmu_setup
 ****************************************************************************/

static inline int nxmu_setup(FAR const char *mqname, FAR NX_DRIVERTYPE *dev,
                             FAR struct nxfe_state_s *fe)
{
  struct mq_attr attr;
  int            ret;

  memset(fe, 0, sizeof(struct nxfe_state_s));

  /* Configure the framebuffer/LCD device */

  ret = nxbe_configure(dev, &fe->be);
  if (ret < 0)
    {
      gdbg("nxbe_configure failed: %d\n", -ret);
      errno = -ret;
      return ERROR;
    }

#if CONFIG_FB_CMAP
  ret = nxbe_colormap(dev);
  if (ret < 0)
    {
      gdbg("nxbe_colormap failed: %d\n", -ret);
      errno = -ret;
      return ERROR;
    }
#endif /* CONFIG_FB_CMAP */

  /* Initialize the non-NULL elements of the server connection structure.
   * Oddly, this strcture represents the connection between the server and
   * itself.
   *
   * Open the incoming server MQ.  The server receives messages on the
   * background window's incoming message queue.
   */

  attr.mq_maxmsg  = CONFIG_NX_MXSERVERMSGS;
  attr.mq_msgsize = NX_MXSVRMSGLEN;
  attr.mq_flags   = 0;

  fe->conn.crdmq = mq_open(mqname, O_RDONLY|O_CREAT, 0666, &attr);
  if (fe->conn.crdmq == (mqd_t)-1)
    {
      gdbg("mq_open(%s) failed: %d\n", mqname, errno);
      return ERROR; /* mq_open sets errno */
    }

  /* NOTE that the outgoing client MQ (cwrmq) is not initialized.  The
   * background window never initiates messages.
   */

  /* Open the outgoing server MQ.  This is used to send messages to the
   * background window which will, of course, be received and handled by
   * the server message loop.
   */

  fe->conn.swrmq = mq_open(mqname, O_WRONLY);
  if (fe->conn.swrmq == (mqd_t)-1)
    {
      gdbg("mq_open(%s) failed: %d\n", mqname, errno);
      mq_close(fe->conn.crdmq);
      return ERROR; /* mq_open sets errno */
    }

  /* The server is now "connected" to itself via the background window */

  fe->conn.state = NX_CLISTATE_CONNECTED;

  /* Initialize the non-NULL elements of the background window */

  fe->be.bkgd.conn = &fe->conn;
  fe->be.bkgd.be   = (FAR struct nxbe_state_s*)fe;

  fe->be.bkgd.bounds.pt2.x = fe->be.vinfo.xres - 1;
  fe->be.bkgd.bounds.pt2.y = fe->be.vinfo.yres - 1;

  /* Complete initialization of the server state structure.  The
   * window list contains only one element:  The background window
   * with nothing else above or below it
   */

  fe->be.topwnd = &fe->be.bkgd;

  /* Initialize the mouse position */

#ifdef CONFIG_NX_MOUSE
  nxmu_mouseinit(fe->be.vinfo.xres, fe->be.vinfo.yres);
#endif
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_runinstance
 *
 * Description:
 *   This is the server entry point.  It does not return; the calling thread
 *   is dedicated to supporting NX server.
 *
 *   NOTE that multiple instances of the NX server may run at the same time,
 *   each with different callback and message queue names.
 *
 * Input Parameters:
 *   mqname - The name for the server incoming message queue
 *   dev    - Vtable "object" of the framebuffer/LCD "driver" to use
 *
 * Return:
 *   This function usually does not return.  If it does return, it will
 *   return ERROR and errno will be set appropriately.
 *
 ****************************************************************************/

int nx_runinstance(FAR const char *mqname, FAR NX_DRIVERTYPE *dev)
{
  struct nxfe_state_s     fe;
  FAR struct nxsvrmsg_s *msg;
  uint8_t                buffer[NX_MXSVRMSGLEN];
  int                    nbytes;
  int                    ret;

  /* Initialization *********************************************************/

  /* Sanity checking */

#ifdef CONFIG_DEBUG
  if (!mqname || !dev)
    {
      errno = EINVAL;
      return ERROR;
    }
#endif

  /* Initialize and configure the server */

  ret = nxmu_setup(mqname, dev, &fe);
  if (ret < 0)
    {
      return ret; /* nxmu_setup sets errno */
    }

  /* Produce the initial, background display */

  nxbe_redraw(&fe.be, &fe.be.bkgd, &fe.be.bkgd.bounds);

  /* Message Loop ***********************************************************/

  /* Then loop forever processing incoming messages */

  for (;;)
    {
       /* Receive the next server message */

       nbytes = mq_receive(fe.conn.crdmq, buffer, NX_MXSVRMSGLEN, 0);
       if (nbytes < 0)
         {
           if (errno != EINTR)
             {
               gdbg("mq_receive failed: %d\n", errno);
               goto errout; /* mq_receive sets errno */
             }
           continue;
         }

       /* Dispatch the message appropriately */

       DEBUGASSERT(nbytes >= sizeof(struct nxsvrmsg_s));
       msg = (FAR struct nxsvrmsg_s *)buffer;

       gvdbg("Received opcode=%d nbytes=%d\n", msg->msgid, nbytes);
       switch (msg->msgid)
         {
         /* Messages sent from clients to the NX server *********************/

         case NX_SVRMSG_CONNECT: /* Establish connection with new NX server client */
           {
             FAR struct nxsvrmsg_s *connmsg = (FAR struct nxsvrmsg_s *)buffer;
             nxmu_connect(connmsg->conn);
           }
           break;

         case NX_SVRMSG_DISCONNECT: /* Tear down connection with terminating client */
           {
             FAR struct nxsvrmsg_s *disconnmsg = (FAR struct nxsvrmsg_s *)buffer;
             nxmu_disconnect(disconnmsg->conn);
           }
           break;

         case NX_SVRMSG_OPENWINDOW: /* Create a new window */
           {
             FAR struct nxsvrmsg_openwindow_s *openmsg = (FAR struct nxsvrmsg_openwindow_s *)buffer;
             nxmu_openwindow(&fe.be, openmsg->wnd);
           }
           break;

         case NX_SVRMSG_CLOSEWINDOW: /* Close an existing window */
           {
             FAR struct nxsvrmsg_closewindow_s *closemsg = (FAR struct nxsvrmsg_closewindow_s *)buffer;
             nxbe_closewindow(closemsg->wnd);
           }
           break;

         case NX_SVRMSG_BLOCKED: /* Block messsages to a window */
           {
             FAR struct nxsvrmsg_blocked_s *blocked = (FAR struct nxsvrmsg_blocked_s *)buffer;
             nxmu_blocked(blocked->wnd, blocked->arg);
           }
           break;

         case NX_SVRMSG_REQUESTBKGD: /* Give access to the background window */
           {
             FAR struct nxsvrmsg_requestbkgd_s *rqbgmsg = (FAR struct nxsvrmsg_requestbkgd_s *)buffer;
             nxmu_requestbkgd(rqbgmsg->conn, &fe.be, rqbgmsg->cb, rqbgmsg->arg);
           }
           break;

         case NX_SVRMSG_RELEASEBKGD: /* End access to the background window */
           {
             nxmu_releasebkgd(&fe);
           }
           break;

         case NX_SVRMSG_SETPOSITION: /* Change window position */
           {
             FAR struct nxsvrmsg_setposition_s *setposmsg = (FAR struct nxsvrmsg_setposition_s *)buffer;
             nxbe_setposition(setposmsg->wnd, &setposmsg->pos);
           }
           break;

         case NX_SVRMSG_SETSIZE: /* Change window size */
           {
             FAR struct nxsvrmsg_setsize_s *setsizemsg = (FAR struct nxsvrmsg_setsize_s *)buffer;
             nxbe_setsize(setsizemsg->wnd, &setsizemsg->size);
           }
           break;

         case NX_SVRMSG_GETPOSITION: /* Get the window size/position */
           {
             FAR struct nxsvrmsg_getposition_s *getposmsg = (FAR struct nxsvrmsg_getposition_s *)buffer;
             nxfe_reportposition(getposmsg->wnd);
           }
           break;

         case NX_SVRMSG_RAISE: /* Move the window to the top of the display */
           {
             FAR struct nxsvrmsg_raise_s *raisemsg = (FAR struct nxsvrmsg_raise_s *)buffer;
             nxbe_raise(raisemsg->wnd);
           }
           break;

         case NX_SVRMSG_LOWER: /* Lower the window to the bottom of the display */
           {
             FAR struct nxsvrmsg_lower_s *lowermsg = (FAR struct nxsvrmsg_lower_s *)buffer;
             nxbe_lower(lowermsg->wnd);
           }
           break;

         case NX_SVRMSG_SETPIXEL: /* Set a single pixel in the window with a color */
           {
             FAR struct nxsvrmsg_setpixel_s *setmsg = (FAR struct nxsvrmsg_setpixel_s *)buffer;
             nxbe_setpixel(setmsg->wnd, &setmsg->pos, setmsg->color);
           }
           break;

         case NX_SVRMSG_FILL: /* Fill a rectangular region in the window with a color */
           {
             FAR struct nxsvrmsg_fill_s *fillmsg = (FAR struct nxsvrmsg_fill_s *)buffer;
             nxbe_fill(fillmsg->wnd, &fillmsg->rect, fillmsg->color);
           }
           break;

         case NX_SVRMSG_GETRECTANGLE: /* Get a rectangular region from the window */
           {
             FAR struct nxsvrmsg_getrectangle_s *getmsg = (FAR struct nxsvrmsg_getrectangle_s *)buffer;
             nxbe_getrectangle(getmsg->wnd, &getmsg->rect, getmsg->plane, getmsg->dest, getmsg->deststride);
             
             if (getmsg->sem_done)
              {
                sem_post(getmsg->sem_done);
              }
           }
           break;

         case NX_SVRMSG_FILLTRAP: /* Fill a trapezoidal region in the window with a color */
           {
             FAR struct nxsvrmsg_filltrapezoid_s *trapmsg = (FAR struct nxsvrmsg_filltrapezoid_s *)buffer;
             nxbe_filltrapezoid(trapmsg->wnd, &trapmsg->clip, &trapmsg->trap, trapmsg->color);
           }
           break;
         case NX_SVRMSG_MOVE: /* Move a rectangular region within the window */
           {
             FAR struct nxsvrmsg_move_s *movemsg = (FAR struct nxsvrmsg_move_s *)buffer;
             nxbe_move(movemsg->wnd, &movemsg->rect, &movemsg->offset);
           }
           break;

         case NX_SVRMSG_BITMAP: /* Copy a rectangular bitmap into the window */
           {
             FAR struct nxsvrmsg_bitmap_s *bmpmsg = (FAR struct nxsvrmsg_bitmap_s *)buffer;
             nxbe_bitmap(bmpmsg->wnd, &bmpmsg->dest, bmpmsg->src, &bmpmsg->origin, bmpmsg->stride);
             
             if (bmpmsg->sem_done)
              {
                sem_post(bmpmsg->sem_done);
              }
           }
           break;

         case NX_SVRMSG_SETBGCOLOR: /* Set the color of the background */
           {
             FAR struct nxsvrmsg_setbgcolor_s *bgcolormsg = (FAR struct nxsvrmsg_setbgcolor_s *)buffer;
             nxgl_colorcopy(fe.be.bgcolor, bgcolormsg->color);
             nxbe_fill(&fe.be.bkgd, &fe.be.bkgd.bounds, bgcolormsg->color);
           }
           break;

#ifdef CONFIG_NX_MOUSE
         case NX_SVRMSG_MOUSEIN: /* New mouse report from mouse client */
           {
             FAR struct nxsvrmsg_mousein_s *mousemsg = (FAR struct nxsvrmsg_mousein_s *)buffer;
             nxmu_mousein(&fe, &mousemsg->pt, mousemsg->buttons);
           }
           break;
#endif
#ifdef CONFIG_NX_KBD
         case NX_SVRMSG_KBDIN: /* New keyboard report from keyboard client */
           {
             FAR struct nxsvrmsg_kbdin_s *kbdmsg = (FAR struct nxsvrmsg_kbdin_s *)buffer;
             nxmu_kbdin(&fe, kbdmsg->nch, kbdmsg->ch);
           }
           break;
#endif

         /* Messages sent to the backgound window ***************************/

         case NX_CLIMSG_REDRAW: /* Re-draw the background window */
            {
              FAR struct nxclimsg_redraw_s *redraw = (FAR struct nxclimsg_redraw_s *)buffer;
              DEBUGASSERT(redraw->wnd == &fe.be.bkgd);
              gvdbg("Re-draw background rect={(%d,%d),(%d,%d)}\n",
                    redraw->rect.pt1.x, redraw->rect.pt1.y,
                    redraw->rect.pt2.x, redraw->rect.pt2.y);
              nxbe_fill(&fe.be.bkgd, &redraw->rect, fe.be.bgcolor);
            }
          break;

         case NX_CLIMSG_MOUSEIN:      /* Ignored */
         case NX_CLIMSG_KBDIN:
           break;

         case NX_CLIMSG_CONNECTED:    /* Shouldn't happen */
         case NX_CLIMSG_DISCONNECTED:
         default:
           gdbg("Unrecognized command: %d\n", msg->msgid);
           break;
         }
    }

errout:
  nxmu_shutdown(&fe);
  return OK;
}
