#pragma once

#include <cstdint>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>
#include <pthread.h>

class LockstepScheduler {
public:
    void set_absolute_time(uint64_t time_us);
    uint64_t get_absolute_time() const;
    int cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *lock, uint64_t time_us);
    int usleep_until(uint64_t timed_us);

private:
    std::atomic<uint64_t> time_us_{0};

    struct TimedWait {
        pthread_cond_t *passed_cond{nullptr};
        pthread_mutex_t *passed_lock{nullptr};
        uint64_t time_us{0};
        bool timeout{false};
        bool done{false};
    };
    std::vector<std::shared_ptr<TimedWait>> timed_waits_{};
    std::mutex timed_waits_mutex_{};
    bool timed_waits_iterator_invalidated_{false};
};
