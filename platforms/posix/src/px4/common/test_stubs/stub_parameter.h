#pragma once

#include <pthread.h>

#include <functional>

std::function<int(pthread_cond_t *, pthread_mutex_t *)> stub_pthread_cond_wait_callback =
[](pthread_cond_t *, pthread_mutex_t *) {return 0;};
