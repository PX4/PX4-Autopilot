// Minimal stubs satisfying TECS.cpp link requirements in unit test builds.
// PX4_WARN calls in TECS.cpp are only reached on invalid dt; they are never
// triggered by the closed-loop tests which always pass dt = 0.02 s.

#include <cstdarg>
#include <cstdio>
#include <px4_platform_common/tasks.h>

extern "C" __attribute__((visibility("default")))
void px4_log_modulename(int /*level*/, const char * /*module*/, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
}

// Stubs for work_queue internals pulled in transitively via px4_layer.
// The HRT work queue thread is never started in unit tests.
px4_task_t px4_task_spawn_cmd(const char * /*name*/, int /*scheduler*/, int /*priority*/,
			      int /*stack_size*/, px4_main_t /*entry*/, char *const /*argv*/[])
{
	return -1;
}

int px4_task_kill(px4_task_t /*id*/, int /*sig*/) { return 0; }

extern "C" px4_task_t px4_getpid() { return -1; }
