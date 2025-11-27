#ifndef BIONIC_PTHREAD_CANCEL_H
#define BIONIC_PTHREAD_CANCEL_H

#ifdef __ANDROID__

#include <pthread.h>

/* The signal used for asynchronous cancelation.  */
#define SIGCANCEL       __SIGRTMIN

namespace
{
static void handler_pthread_cancel(int sig)
{
	pthread_exit(0);
}
int pthread_cancel(pthread_t thread)
{
	return pthread_kill(thread, SIGCANCEL);
}
void register_handler_pthread_cancel()
{
	struct sigaction actions;
	memset(&actions, 0, sizeof(actions));
	sigemptyset(&actions.sa_mask);
	actions.sa_flags = 0;
	actions.sa_handler = handler_pthread_cancel;
	sigaction(SIGCANCEL, &actions, NULL);
}
}

#endif

#endif // BIONIC_PTHREAD_CANCEL_H
