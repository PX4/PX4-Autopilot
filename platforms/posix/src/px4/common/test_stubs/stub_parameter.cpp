#include "stub_parameter.h"



extern "C" {
	/* This function blocks forever in tests, so override it with a version that can be customized */
	int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex)
	{
		return stub_pthread_cond_wait_callback(cond, mutex);
	}
}
