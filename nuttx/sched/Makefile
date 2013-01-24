############################################################################
# sched/Makefile
#
#   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

ASRCS = 
AOBJS = $(ASRCS:.S=$(OBJEXT))

MISC_SRCS  = os_start.c os_bringup.c errno_getptr.c errno_get.c errno_set.c
MISC_SRCS += sched_garbage.c sched_setupstreams.c sched_getfiles.c sched_getsockets.c
MISC_SRCS += sched_getstreams.c sched_setupidlefiles.c sched_setuptaskfiles.c
MISC_SRCS += sched_setuppthreadfiles.c sched_releasefiles.c

TSK_SRCS = prctl.c task_create.c task_init.c task_setup.c task_activate.c
TSK_SRCS += task_start.c task_delete.c task_deletecurrent.c task_exithook.c
TSK_SRCS += task_restart.c task_vfork.c exit.c getpid.c sched_addreadytorun.c
TSK_SRCS += sched_removereadytorun.c sched_addprioritized.c sched_mergepending.c
TSK_SRCS += sched_addblocked.c sched_removeblocked.c sched_free.c sched_gettcb.c
TSK_SRCS += sched_verifytcb.c sched_releasetcb.c

ifneq ($(CONFIG_BINFMT_DISABLE),y)
ifeq ($(CONFIG_LIBC_EXECFUNCS),y)
TSK_SRCS += task_posixspawn.c
endif
endif

SCHED_SRCS = sched_setparam.c sched_setpriority.c sched_getparam.c
SCHED_SRCS += sched_setscheduler.c sched_getscheduler.c
SCHED_SRCS += sched_yield.c sched_rrgetinterval.c sched_foreach.c
SCHED_SRCS += sched_lock.c sched_unlock.c sched_lockcount.c sched_self.c

ifeq ($(CONFIG_SCHED_ATEXIT),y)
SCHED_SRCS += atexit.c 
endif

ifeq ($(CONFIG_SCHED_ONEXIT),y)
SCHED_SRCS += on_exit.c 
endif

ifeq ($(CONFIG_PRIORITY_INHERITANCE),y)
SCHED_SRCS += sched_reprioritize.c 
endif

ifeq ($(CONFIG_SCHED_HAVE_PARENT),y)
SCHED_SRCS += task_reparent.c
ifeq ($(CONFIG_SCHED_CHILD_STATUS),y)
SCHED_SRCS += task_childstatus.c 
endif
endif

ifeq ($(CONFIG_SCHED_WAITPID),y)
SCHED_SRCS += sched_waitpid.c
ifeq ($(CONFIG_SCHED_HAVE_PARENT),y)
SCHED_SRCS += sched_waitid.c sched_wait.c
endif
endif

ENV_SRCS  = env_getenvironptr.c env_dup.c env_share.c env_release.c
ENV_SRCS += env_findvar.c env_removevar.c
ENV_SRCS += env_clearenv.c env_getenv.c env_putenv.c env_setenv.c env_unsetenv.c

WDOG_SRCS = wd_initialize.c wd_create.c wd_start.c wd_cancel.c wd_delete.c
WDOG_SRCS += wd_gettime.c

TIME_SRCS = sched_processtimer.c

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
TIME_SRCS += sleep.c usleep.c
endif

CLOCK_SRCS  = clock_initialize.c clock_settime.c clock_gettime.c clock_getres.c
CLOCK_SRCS += clock_time2ticks.c clock_abstime2ticks.c clock_ticks2time.c
CLOCK_SRCS += clock_gettimeofday.c clock_systimer.c

SIGNAL_SRCS  = sig_initialize.c
SIGNAL_SRCS += sig_action.c sig_procmask.c sig_pending.c sig_suspend.c
SIGNAL_SRCS += sig_kill.c sig_queue.c sig_waitinfo.c sig_timedwait.c
SIGNAL_SRCS += sig_findaction.c sig_allocatependingsigaction.c
SIGNAL_SRCS += sig_releasependingsigaction.c sig_unmaskpendingsignal.c
SIGNAL_SRCS += sig_removependingsignal.c sig_releasependingsignal.c sig_lowest.c
SIGNAL_SRCS += sig_mqnotempty.c sig_cleanup.c sig_received.c sig_deliver.c pause.c

MQUEUE_SRCS  = mq_open.c mq_close.c mq_unlink.c mq_send.c mq_timedsend.c
MQUEUE_SRCS += mq_sndinternal.c mq_receive.c mq_timedreceive.c mq_rcvinternal.c
MQUEUE_SRCS += mq_initialize.c mq_descreate.c mq_findnamed.c mq_msgfree.c mq_msgqfree.c

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
MQUEUE_SRCS += mq_waitirq.c
endif

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
MQUEUE_SRCS += mq_notify.c
endif

PTHREAD_SRCS  = pthread_create.c pthread_exit.c pthread_join.c pthread_detach.c
PTHREAD_SRCS += pthread_yield.c pthread_getschedparam.c pthread_setschedparam.c
PTHREAD_SRCS += pthread_mutexinit.c pthread_mutexdestroy.c
PTHREAD_SRCS += pthread_mutexlock.c pthread_mutextrylock.c pthread_mutexunlock.c
PTHREAD_SRCS += pthread_condinit.c pthread_conddestroy.c
PTHREAD_SRCS += pthread_condwait.c pthread_condsignal.c pthread_condbroadcast.c
PTHREAD_SRCS += pthread_barrierinit.c pthread_barrierdestroy.c pthread_barrierwait.c
PTHREAD_SRCS += pthread_cancel.c pthread_setcancelstate.c
PTHREAD_SRCS += pthread_keycreate.c pthread_setspecific.c pthread_getspecific.c pthread_keydelete.c
PTHREAD_SRCS += pthread_initialize.c pthread_completejoin.c pthread_findjoininfo.c
PTHREAD_SRCS += pthread_removejoininfo.c pthread_once.c pthread_setschedprio.c

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
PTHREAD_SRCS += pthread_condtimedwait.c pthread_kill.c pthread_sigmask.c
endif

SEM_SRCS  = sem_initialize.c sem_destroy.c sem_open.c sem_close.c sem_unlink.c
SEM_SRCS += sem_wait.c sem_trywait.c sem_timedwait.c sem_post.c sem_findnamed.c

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
SEM_SRCS += sem_waitirq.c
endif

ifeq ($(CONFIG_PRIORITY_INHERITANCE),y)
SEM_SRCS += sem_holder.c
endif

ifneq ($(CONFIG_DISABLE_POSIX_TIMERS),y)
TIMER_SRCS += timer_initialize.c timer_create.c timer_delete.c timer_getoverrun.c
TIMER_SRCS += timer_gettime.c timer_settime.c timer_release.c
endif

ifeq ($(CONFIG_SCHED_WORKQUEUE),y)
WORK_SRCS = work_thread.c work_queue.c work_cancel.c work_signal.c
endif

ifeq ($(CONFIG_PAGING),y)
PGFILL_SRCS = pg_miss.c pg_worker.c
endif

IRQ_SRCS = irq_initialize.c irq_attach.c irq_dispatch.c irq_unexpectedisr.c

KMM_SRCS = kmm_initialize.c kmm_addregion.c kmm_semaphore.c
KMM_SRCS = kmm_kmalloc.c kmm_kzalloc.c kmm_krealloc.c kmm_kfree.c

CSRCS = $(MISC_SRCS) $(TSK_SRCS) $(SCHED_SRCS) $(WDOG_SRCS) $(TIME_SRCS) \
	    $(SEM_SRCS) $(TIMER_SRCS) $(WORK_SRCS) $(PGFILL_SRCS) $(IRQ_SRCS)

ifneq ($(CONFIG_DISABLE_CLOCK),y)
CSRCS += $(CLOCK_SRCS)
endif

ifneq ($(CONFIG_DISABLE_PTHREAD),y)
CSRCS += $(PTHREAD_SRCS)
endif

ifneq ($(CONFIG_DISABLE_SIGNALS),y)
CSRCS += $(SIGNAL_SRCS)
endif

ifneq ($(CONFIG_DISABLE_MQUEUE),y)
CSRCS += $(MQUEUE_SRCS)
endif

ifneq ($(CONFIG_DISABLE_ENVIRON),y)
CSRCS += $(ENV_SRCS)
endif

ifeq ($(CONFIG_NUTTX_KERNEL),y)
CSRCS += $(KMM_SRCS)
endif

COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libsched$(LIBEXT)

all: $(BIN)
.PHONY: context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
