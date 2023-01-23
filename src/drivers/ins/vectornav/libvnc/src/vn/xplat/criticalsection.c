#include "vn/xplat/criticalsection.h"

VnError VnCriticalSection_initialize(VnCriticalSection *cs)
{
	#if _WIN32

	InitializeCriticalSection(&cs->handle);

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || defined __NUTTX__

	if (pthread_mutex_init(&cs->handle, NULL))
		return E_UNKNOWN;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnCriticalSection_deinitialize(VnCriticalSection *cs)
{
	#if _WIN32

	DeleteCriticalSection(&cs->handle);

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || defined __NUTTX__

	if (pthread_mutex_destroy(&cs->handle))
		return E_UNKNOWN;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnCriticalSection_enter(VnCriticalSection *cs)
{
	#if _WIN32

	EnterCriticalSection(&cs->handle);

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || defined __NUTTX__

	if (pthread_mutex_lock(&cs->handle))
		return E_UNKNOWN;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnCriticalSection_leave(VnCriticalSection *cs)
{
	#if _WIN32

	LeaveCriticalSection(&cs->handle);

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || defined __NUTTX__

	if (pthread_mutex_unlock(&cs->handle))
		return E_UNKNOWN;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}
