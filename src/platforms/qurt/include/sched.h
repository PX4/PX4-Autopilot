#pragma once

#define SCHED_FIFO     1
#define SCHED_RR       2

struct sched_param {
	int sched_priority;
};

int    sched_get_priority_max(int policy);
int    sched_get_priority_min(int policy);

