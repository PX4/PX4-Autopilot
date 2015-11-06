#include <semaphore.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>

int sem_init(sem_t *sem, int pshared, unsigned int value)
{
	return 1;
}

int sem_wait(sem_t *sem)
{
	return 1;
}

int sem_destroy(sem_t *sem)
{
	return 1;
}

int sem_post(sem_t *sem)
{
	return 1;
}

int sem_getvalue(sem_t *sem, int *sval)
{
	return 1;
}

int usleep(useconds_t usec)
{
	return 0;
}

pthread_t pthread_self(void)
{
	pthread_t x = 0;
	return x;
}


int pthread_kill(pthread_t thread, int sig)
{
	return 1;
}

void pthread_exit(void *retval)
{
}

int pthread_join(pthread_t thread, void **retval)
{
	return 1;
}

int pthread_cancel(pthread_t thread)
{
	return 1;
}
int pthread_attr_init(pthread_attr_t *attr)
{
	return 1;
}

int pthread_attr_setstacksize(pthread_attr_t *attr, size_t stacksize)
{
	return 1;
}

int pthread_attr_getstacksize(const pthread_attr_t *attr, size_t *stacksize)
{
	return 1;
}

int pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param)
{
	return 1;
}

int pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine)(void *), void *arg)
{
	return 1;
}
int pthread_attr_getschedparam(const pthread_attr_t *attr, struct sched_param *param)
{
	return 1;
}

int pthread_attr_destroy(pthread_attr_t *attr)
{
	return 1;
}

int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	return 1;
}
