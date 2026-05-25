/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
/**
 * @file pthread.cpp
 *
 * Small pthread compatibility layer for native MSVC builds.
 *
 * MinGW uses winpthreads and does not compile this file. MSVC has no pthread
 * library, so this implements the subset used by PX4 SITL with Win32 threads,
 * critical sections, condition variables, TLS, and init-once. It is not a
 * general-purpose pthreads replacement; add POSIX surface here only when PX4
 * actually needs it.
 */

#include "px4_windows_internal.h"

#if defined(_MSC_VER)

#include <atomic>
#include <process.h>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace
{

thread_local pthread_t tls_self = 0;

struct PX4ThreadStart {
	void *(*start_routine)(void *);
	void *arg;
	pthread_t self;
};

/* POSIX requires pthread_key_create() destructors to run on thread exit. The
 * Win32 TLS API has no such hook, so we keep our own registry of keys with
 * non-null destructors and walk it from the thread trampoline tail. Without
 * this, every per-thread allocation registered via pthread_setspecific() leaks
 * on the MSVC SITL build (e.g. CmdThreadSpecificData in px4_daemon::Server). */
std::mutex &tls_destructor_mutex()
{
	static std::mutex m;
	return m;
}

std::unordered_map<pthread_key_t, void (*)(void *)> &tls_destructors()
{
	static std::unordered_map<pthread_key_t, void (*)(void *)> map;
	return map;
}

std::atomic<px4_pthread_cond_notify_callback_t> &cond_notify_callback()
{
	static std::atomic<px4_pthread_cond_notify_callback_t> callback{nullptr};
	return callback;
}

void run_tls_destructors_on_exit()
{
	/* POSIX allows up to PTHREAD_DESTRUCTOR_ITERATIONS (typically 4) passes
	 * because a destructor may install new TLS values. Snapshot under the
	 * mutex, run unlocked so destructors can call pthread_key_delete()/
	 * pthread_setspecific() without deadlocking. */
	for (int pass = 0; pass < 4; ++pass) {
		struct Pending {
			pthread_key_t key;
			void (*destructor)(void *);
			void *value;
		};
		std::vector<Pending> pending;
		{
			std::lock_guard<std::mutex> guard(tls_destructor_mutex());
			pending.reserve(tls_destructors().size());

			for (const auto &entry : tls_destructors()) {
				if (entry.second == nullptr) {
					continue;
				}

				void *value = TlsGetValue(entry.first);

				if (value == nullptr) {
					continue;
				}

				pending.push_back({entry.first, entry.second, value});
			}
		}

		if (pending.empty()) {
			return;
		}

		for (const auto &p : pending) {
			TlsSetValue(p.key, nullptr);
			p.destructor(p.value);
		}
	}
}

BOOL CALLBACK init_mutex_once(PINIT_ONCE, PVOID parameter, PVOID *)
{
	/* Static pthread mutex initializers cannot run a constructor. INIT_ONCE lets
	 * the first lock/trylock/destroy lazily initialize the CRITICAL_SECTION. */
	pthread_mutex_t *mutex = static_cast<pthread_mutex_t *>(parameter);
	InitializeCriticalSection(&mutex->critical_section);
	return TRUE;
}

BOOL CALLBACK pthread_once_callback(PINIT_ONCE, PVOID parameter, PVOID *)
{
	reinterpret_cast<void (*)(void)>(parameter)();
	return TRUE;
}

static int ensure_mutex(pthread_mutex_t *mutex)
{
	if (!mutex) {
		return EINVAL;
	}

	return InitOnceExecuteOnce(&mutex->once, init_mutex_once, mutex, nullptr) ? 0 : EINVAL;
}

static HANDLE handle_for_pthread(pthread_t thread)
{
	/* pthread_self() for externally-created threads falls back to the Win32
	 * thread id. Thread-description APIs require a pseudo handle for the current
	 * thread in that case. */
	if (thread == 0 || thread == static_cast<pthread_t>(GetCurrentThreadId())) {
		return GetCurrentThread();
	}

	return reinterpret_cast<HANDLE>(thread);
}

static int windows_thread_priority(int priority)
{
	if (priority >= THREAD_PRIORITY_TIME_CRITICAL) {
		return THREAD_PRIORITY_TIME_CRITICAL;

	} else if (priority >= THREAD_PRIORITY_HIGHEST) {
		return THREAD_PRIORITY_HIGHEST;

	} else if (priority >= THREAD_PRIORITY_ABOVE_NORMAL) {
		return THREAD_PRIORITY_ABOVE_NORMAL;

	} else if (priority == THREAD_PRIORITY_NORMAL) {
		return THREAD_PRIORITY_NORMAL;

	} else if (priority <= THREAD_PRIORITY_IDLE) {
		return THREAD_PRIORITY_IDLE;

	} else if (priority <= THREAD_PRIORITY_LOWEST) {
		return THREAD_PRIORITY_LOWEST;
	}

	return THREAD_PRIORITY_BELOW_NORMAL;
}

static DWORD abstime_to_timeout_ms(const timespec *abstime)
{
	if (!abstime) {
		return INFINITE;
	}

	timespec now {};
	clock_gettime(CLOCK_REALTIME, &now);

	/* POSIX timed waits use an absolute timestamp; Win32 condition variables
	 * take a relative millisecond timeout. */
	const int64_t now_ms = (static_cast<int64_t>(now.tv_sec) * 1000) + (now.tv_nsec / 1000000);
	const int64_t target_ms = (static_cast<int64_t>(abstime->tv_sec) * 1000) + (abstime->tv_nsec / 1000000);

	if (target_ms <= now_ms) {
		return 0;
	}

	const int64_t delta = target_ms - now_ms;
	return (delta > static_cast<int64_t>(MAXDWORD)) ? INFINITE : static_cast<DWORD>(delta);
}

unsigned __stdcall thread_trampoline(void *arg)
{
	PX4ThreadStart *start = static_cast<PX4ThreadStart *>(arg);
	/* Store the handle-valued pthread id before entering PX4 code so
	 * pthread_self() returns something joinable/comparable inside the thread. */
	tls_self = start->self;
	void *(*entry)(void *) = start->start_routine;
	void *entry_arg = start->arg;
	delete start;

	const uintptr_t result = reinterpret_cast<uintptr_t>(entry(entry_arg));
	run_tls_destructors_on_exit();
	_endthreadex(static_cast<unsigned>(result));
	return static_cast<unsigned>(result);
}

}

extern "C" {

	int pthread_attr_init(pthread_attr_t *attr)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->stack_size = 0;
		attr->stack_addr = nullptr;
		attr->detach_state = PTHREAD_CREATE_JOINABLE;
		attr->sched_policy = SCHED_OTHER;
		attr->inherit_sched = PTHREAD_INHERIT_SCHED;
		attr->scope = PTHREAD_SCOPE_SYSTEM;
		attr->sched.sched_priority = 0;
		return 0;
	}

	int pthread_attr_destroy(pthread_attr_t *attr)
	{
		(void)attr;
		return 0;
	}

	int pthread_attr_setstacksize(pthread_attr_t *attr, size_t stack_size)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->stack_size = stack_size;
		return 0;
	}

	int pthread_attr_getstacksize(const pthread_attr_t *attr, size_t *stack_size)
	{
		if (!attr || !stack_size) {
			return EINVAL;
		}

		*stack_size = attr->stack_size;
		return 0;
	}

	int pthread_attr_setstack(pthread_attr_t *attr, void *stack_addr, size_t stack_size)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->stack_addr = stack_addr;
		attr->stack_size = stack_size;
		return 0;
	}

	int pthread_attr_getstack(const pthread_attr_t *attr, void **stack_addr, size_t *stack_size)
	{
		if (!attr || !stack_addr || !stack_size) {
			return EINVAL;
		}

		*stack_addr = attr->stack_addr;
		*stack_size = attr->stack_size;
		return 0;
	}

	int pthread_attr_getschedparam(const pthread_attr_t *attr, struct sched_param *param)
	{
		if (!attr || !param) {
			return EINVAL;
		}

		*param = attr->sched;
		return 0;
	}

	int pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param)
	{
		if (!attr || !param) {
			return EINVAL;
		}

		attr->sched = *param;
		return 0;
	}

	int pthread_attr_getschedpolicy(const pthread_attr_t *attr, int *policy)
	{
		if (!attr || !policy) {
			return EINVAL;
		}

		*policy = attr->sched_policy;
		return 0;
	}

	int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->sched_policy = policy;
		return 0;
	}

	int pthread_attr_getinheritsched(const pthread_attr_t *attr, int *inheritsched)
	{
		if (!attr || !inheritsched) {
			return EINVAL;
		}

		*inheritsched = attr->inherit_sched;
		return 0;
	}

	int pthread_attr_setinheritsched(pthread_attr_t *attr, int inheritsched)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->inherit_sched = inheritsched;
		return 0;
	}

	int pthread_attr_getdetachstate(const pthread_attr_t *attr, int *detachstate)
	{
		if (!attr || !detachstate) {
			return EINVAL;
		}

		*detachstate = attr->detach_state;
		return 0;
	}

	int pthread_attr_setdetachstate(pthread_attr_t *attr, int detachstate)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->detach_state = detachstate;
		return 0;
	}

	int pthread_attr_getscope(const pthread_attr_t *attr, int *scope)
	{
		if (!attr || !scope) {
			return EINVAL;
		}

		*scope = attr->scope;
		return 0;
	}

	int pthread_attr_setscope(pthread_attr_t *attr, int scope)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->scope = scope;
		return 0;
	}

	int pthread_mutexattr_init(pthread_mutexattr_t *attr)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->type = PTHREAD_MUTEX_NORMAL;
		return 0;
	}

	int pthread_mutexattr_destroy(pthread_mutexattr_t *attr)
	{
		(void)attr;
		return 0;
	}

	int pthread_mutexattr_gettype(const pthread_mutexattr_t *attr, int *type)
	{
		if (!attr || !type) {
			return EINVAL;
		}

		*type = attr->type;
		return 0;
	}

	int pthread_mutexattr_settype(pthread_mutexattr_t *attr, int type)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->type = type;
		return 0;
	}

	int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr)
	{
		if (!mutex) {
			return EINVAL;
		}

		mutex->once = INIT_ONCE_STATIC_INIT;
		mutex->type = attr ? attr->type : PTHREAD_MUTEX_NORMAL;
		return ensure_mutex(mutex);
	}

	int pthread_mutex_destroy(pthread_mutex_t *mutex)
	{
		if (ensure_mutex(mutex) != 0) {
			return EINVAL;
		}

		DeleteCriticalSection(&mutex->critical_section);
		mutex->once = INIT_ONCE_STATIC_INIT;
		return 0;
	}

	int pthread_mutex_lock(pthread_mutex_t *mutex)
	{
		if (ensure_mutex(mutex) != 0) {
			return EINVAL;
		}

		EnterCriticalSection(&mutex->critical_section);
		return 0;
	}

	int pthread_mutex_trylock(pthread_mutex_t *mutex)
	{
		if (ensure_mutex(mutex) != 0) {
			return EINVAL;
		}

		return TryEnterCriticalSection(&mutex->critical_section) ? 0 : EBUSY;
	}

	int pthread_mutex_unlock(pthread_mutex_t *mutex)
	{
		if (ensure_mutex(mutex) != 0) {
			return EINVAL;
		}

		LeaveCriticalSection(&mutex->critical_section);
		return 0;
	}

	int pthread_condattr_init(pthread_condattr_t *attr)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->clock_id = CLOCK_REALTIME;
		return 0;
	}

	int pthread_condattr_destroy(pthread_condattr_t *attr)
	{
		(void)attr;
		return 0;
	}

	int pthread_condattr_setclock(pthread_condattr_t *attr, clockid_t clock_id)
	{
		if (!attr) {
			return EINVAL;
		}

		attr->clock_id = clock_id;
		return 0;
	}

	int pthread_cond_init(pthread_cond_t *cond, const pthread_condattr_t *attr)
	{
		(void)attr;

		if (!cond) {
			return EINVAL;
		}

		InitializeConditionVariable(cond);
		return 0;
	}

	int pthread_cond_destroy(pthread_cond_t *cond)
	{
		(void)cond;
		return 0;
	}

	int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex)
	{
		if (!cond || ensure_mutex(mutex) != 0) {
			return EINVAL;
		}

		return SleepConditionVariableCS(cond, &mutex->critical_section, INFINITE) ? 0 : EINVAL;
	}

	int pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex, const struct timespec *abstime)
	{
		if (!cond || ensure_mutex(mutex) != 0) {
			return EINVAL;
		}

		if (SleepConditionVariableCS(cond, &mutex->critical_section, abstime_to_timeout_ms(abstime))) {
			return 0;
		}

		return (GetLastError() == ERROR_TIMEOUT) ? ETIMEDOUT : EINVAL;
	}

	int pthread_cond_signal(pthread_cond_t *cond)
	{
		if (!cond) {
			return EINVAL;
		}

		if (px4_pthread_cond_notify_callback_t callback = cond_notify_callback().load(std::memory_order_acquire)) {
			callback(cond, 0);
		}

		WakeConditionVariable(cond);
		return 0;
	}

	int pthread_cond_broadcast(pthread_cond_t *cond)
	{
		if (!cond) {
			return EINVAL;
		}

		if (px4_pthread_cond_notify_callback_t callback = cond_notify_callback().load(std::memory_order_acquire)) {
			callback(cond, 1);
		}

		WakeAllConditionVariable(cond);
		return 0;
	}

	int px4_pthread_cond_set_notify_callback(px4_pthread_cond_notify_callback_t callback)
	{
		cond_notify_callback().store(callback, std::memory_order_release);
		return 0;
	}

	int pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine)(void *), void *arg)
	{
		if (!thread || !start_routine) {
			return EINVAL;
		}

		PX4ThreadStart *start = new PX4ThreadStart{start_routine, arg, 0};
		unsigned thread_id = 0;
		/* Create suspended so the parent can publish the final HANDLE-valued
		 * pthread_t before the child calls pthread_self(). */
		const uintptr_t handle = _beginthreadex(nullptr,
							attr ? static_cast<unsigned>(attr->stack_size) : 0,
							thread_trampoline,
							start,
							CREATE_SUSPENDED,
							&thread_id);

		if (handle == 0) {
			delete start;
			return EAGAIN;
		}

		start->self = static_cast<pthread_t>(handle);
		*thread = static_cast<pthread_t>(handle);

		if (attr && attr->sched.sched_priority != 0) {
			SetThreadPriority(reinterpret_cast<HANDLE>(handle), windows_thread_priority(attr->sched.sched_priority));
		}

		ResumeThread(reinterpret_cast<HANDLE>(handle));

		if (attr && attr->detach_state == PTHREAD_CREATE_DETACHED) {
			/* Closing the handle detaches ownership from the creator. The Windows
			 * thread itself keeps running until its entry function returns. */
			CloseHandle(reinterpret_cast<HANDLE>(handle));
		}

		return 0;
	}

	int pthread_join(pthread_t thread, void **value_ptr)
	{
		if (thread == 0) {
			return ESRCH;
		}

		HANDLE handle = reinterpret_cast<HANDLE>(thread);

		if (WaitForSingleObject(handle, INFINITE) == WAIT_FAILED) {
			/* Even on failure the caller has handed ownership of the handle
			 * to pthread_join() per POSIX semantics; close it so we don't
			 * leak the Win32 thread object. */
			CloseHandle(handle);
			return ESRCH;
		}

		if (value_ptr) {
			DWORD exit_code = 0;
			GetExitCodeThread(handle, &exit_code);
			*value_ptr = reinterpret_cast<void *>(static_cast<uintptr_t>(exit_code));
		}

		CloseHandle(handle);
		return 0;
	}

	int pthread_detach(pthread_t thread)
	{
		if (thread == 0) {
			return ESRCH;
		}

		CloseHandle(reinterpret_cast<HANDLE>(thread));
		return 0;
	}

	void pthread_exit(void *value_ptr)
	{
		run_tls_destructors_on_exit();
		_endthreadex(static_cast<unsigned>(reinterpret_cast<uintptr_t>(value_ptr)));
	}

	pthread_t pthread_self(void)
	{
		if (tls_self != 0) {
			return tls_self;
		}

		return static_cast<pthread_t>(GetCurrentThreadId());
	}

	int pthread_equal(pthread_t t1, pthread_t t2)
	{
		return t1 == t2;
	}

	int pthread_getschedparam(pthread_t thread, int *policy, struct sched_param *param)
	{
		if (!policy || !param) {
			return EINVAL;
		}

		const int priority = GetThreadPriority(handle_for_pthread(thread));

		if (priority == THREAD_PRIORITY_ERROR_RETURN && GetLastError() != ERROR_SUCCESS) {
			return ESRCH;
		}

		*policy = SCHED_OTHER;
		param->sched_priority = priority;
		return 0;
	}

	int pthread_setschedparam(pthread_t thread, int policy, const struct sched_param *param)
	{
		(void)policy;

		if (!param) {
			return EINVAL;
		}

		return SetThreadPriority(handle_for_pthread(thread), windows_thread_priority(param->sched_priority)) ? 0 : ESRCH;
	}

	int pthread_cancel(pthread_t thread)
	{
		if (thread == 0) {
			return ESRCH;
		}

		/* POSIX cancellation is cooperative; TerminateThread is not. PX4 does not
		 * depend on cancellation cleanup handlers in SITL, so this is a last-resort
		 * compatibility hook for code that expects pthread_cancel to exist. */
		return TerminateThread(reinterpret_cast<HANDLE>(thread), 0) ? 0 : ESRCH;
	}

	int pthread_kill(pthread_t thread, int sig)
	{
		(void)sig;
		return thread == 0 ? ESRCH : 0;
	}

	int pthread_key_create(pthread_key_t *key, void (*destructor)(void *))
	{
		if (!key) {
			return EINVAL;
		}

		const DWORD index = TlsAlloc();

		if (index == TLS_OUT_OF_INDEXES) {
			return EAGAIN;
		}

		if (destructor) {
			std::lock_guard<std::mutex> guard(tls_destructor_mutex());
			tls_destructors()[index] = destructor;
		}

		*key = index;
		return 0;
	}

	int pthread_key_delete(pthread_key_t key)
	{
		{
			std::lock_guard<std::mutex> guard(tls_destructor_mutex());
			tls_destructors().erase(key);
		}
		return TlsFree(key) ? 0 : EINVAL;
	}

	int pthread_setspecific(pthread_key_t key, const void *value)
	{
		return TlsSetValue(key, const_cast<void *>(value)) ? 0 : EINVAL;
	}

	void *pthread_getspecific(pthread_key_t key)
	{
		return TlsGetValue(key);
	}

	int pthread_once(pthread_once_t *once_control, void (*init_routine)(void))
	{
		if (!once_control || !init_routine) {
			return EINVAL;
		}

		return InitOnceExecuteOnce(once_control, pthread_once_callback, reinterpret_cast<PVOID>(init_routine), nullptr) ? 0 : EINVAL;
	}

	int px4_pthread_setname_np(pthread_t thread, const char *name)
	{
		if (!name) {
			return EINVAL;
		}

		HANDLE handle = handle_for_pthread(thread);

		wchar_t wide_name[64] {};
		MultiByteToWideChar(CP_UTF8, 0, name, -1, wide_name, static_cast<int>(sizeof(wide_name) / sizeof(wide_name[0])));

		typedef HRESULT(WINAPI * SetThreadDescriptionFn)(HANDLE, PCWSTR);
		/* Thread descriptions exist on modern Windows. Resolve dynamically so the
		 * binary still starts on older hosts or Wine versions lacking the export. */
		static SetThreadDescriptionFn set_thread_description =
			reinterpret_cast<SetThreadDescriptionFn>(GetProcAddress(GetModuleHandleA("Kernel32.dll"), "SetThreadDescription"));

		if (set_thread_description) {
			set_thread_description(handle, wide_name);
		}

		return 0;
	}

	int px4_pthread_setname_current_np(const char *name)
	{
		return px4_pthread_setname_np(0, name);
	}

	int pthread_getname_np(pthread_t thread, char *name, size_t len)
	{
		if (!name || len == 0) {
			return EINVAL;
		}

		name[0] = '\0';

		HANDLE handle = handle_for_pthread(thread);

		typedef HRESULT(WINAPI * GetThreadDescriptionFn)(HANDLE, PWSTR *);
		static GetThreadDescriptionFn get_thread_description =
			reinterpret_cast<GetThreadDescriptionFn>(GetProcAddress(GetModuleHandleA("Kernel32.dll"), "GetThreadDescription"));

		if (!get_thread_description) {
			return 0;
		}

		PWSTR wide_name = nullptr;

		if (FAILED(get_thread_description(handle, &wide_name)) || !wide_name) {
			return 0;
		}

		WideCharToMultiByte(CP_UTF8, 0, wide_name, -1, name, static_cast<int>(len), nullptr, nullptr);
		LocalFree(wide_name);
		return 0;
	}

} // extern "C"

#endif // defined(_MSC_VER)
