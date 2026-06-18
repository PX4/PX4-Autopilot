/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/
#ifdef EN_ION_BUF

#include "buffers.h"

static bool g_allocator_initialized = false;

static int64_t _time_monotonic_ns(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts))
    {
        fprintf(stderr, "ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec * 1000000000 + (int64_t)ts.tv_nsec;
}

int mpa_ion_buf_pool_alloc_bufs(mpa_ion_buf_pool_t *pool_ptr, uint32_t n_bufs, uint32_t width,
                                uint32_t height, uint32_t hal3_format, uint32_t gralloc_flags)
{
    if (!g_allocator_initialized)
    {
        init_buffer_allocator(); // asserts on failure
        g_allocator_initialized = true;
    }

    if (n_bufs > MPA_ION_BUF_POOL_MAX_SIZE)
    {
        n_bufs = MPA_ION_BUF_POOL_MAX_SIZE;
    }

    pthread_mutex_init(&pool_ptr->pool_mtx, NULL);
    for (uint32_t i = 0; i < n_bufs; i++)
    {

        pool_ptr->bufs[i] = (mpa_ion_buf_t)MPA_ION_BUF_INITIALIZER;
        if (allocate_one_buffer(&pool_ptr->bufs[i], width, height, hal3_format, gralloc_flags))
        {
            printf("buffer allocation failed at index: %d\n", i);
            return -1;
        };

        pool_ptr->bufs[i].buffer_id = i;
        pool_ptr->in_use[i] = 0;
        pool_ptr->n_client_references[i] = 0;
        pool_ptr->time_sent_to_clients_ns[i] = 0;
        pool_ptr->producer_active[i] = 0;
        pool_ptr->client_mask[i] = 0;
        pool_ptr->generation[i] = 0;
        pool_ptr->bufs[i].generation = pool_ptr->generation[i];
    }

    pool_ptr->n_bufs = n_bufs;
    pool_ptr->initialized = 1;

    return 0;
}

int mpa_ion_buf_pool_delete_bufs(mpa_ion_buf_pool_t *pool_ptr)
{
    for (uint32_t i = 0; i < pool_ptr->n_bufs; i++)
    {
        if (delete_one_buffer(&pool_ptr->bufs[i]))
        {
            printf("buffer deletion failed at index: %d\n", i);
            return -1;
        };

        pool_ptr->bufs[i] = (mpa_ion_buf_t)MPA_ION_BUF_INITIALIZER;
        pool_ptr->bufs[i].buffer_id = i;
        pool_ptr->in_use[i] = 0;
        pool_ptr->n_client_references[i] = 0;
        pool_ptr->time_sent_to_clients_ns[i] = 0;
        pool_ptr->producer_active[i] = 0;
        pool_ptr->client_mask[i] = 0;
        pool_ptr->generation[i] = 0;
        pool_ptr->bufs[i].generation = pool_ptr->generation[i];
    }

    pool_ptr->n_bufs = 0;
    pool_ptr->initialized = 0;

    shutdown_buffer_allocator();
    g_allocator_initialized = false;

    return 0;
}

mpa_ion_buf_t *mpa_ion_buf_pool_get_buf_info_by_id(mpa_ion_buf_pool_t *pool_ptr, int id)
{
    pthread_mutex_lock(&pool_ptr->pool_mtx);

    for (uint32_t i = 0; i < pool_ptr->n_bufs; i++)
    {
        if (pool_ptr->bufs[i].buffer_id == id)
        {
            pthread_mutex_unlock(&pool_ptr->pool_mtx);
            return &pool_ptr->bufs[i];
        }
    }

    pthread_mutex_unlock(&pool_ptr->pool_mtx);
    return NULL;
}

mpa_ion_buf_t *mpa_ion_buf_pool_get_buf_info_by_handle(mpa_ion_buf_pool_t *pool_ptr, buffer_handle_t *handle)
{
    pthread_mutex_lock(&pool_ptr->pool_mtx);

    for (uint32_t i = 0; i < pool_ptr->n_bufs; i++)
    {
        if (*handle == pool_ptr->bufs[i].handle)
        {
            pthread_mutex_unlock(&pool_ptr->pool_mtx);
            return &pool_ptr->bufs[i];
        }
    }

    pthread_mutex_unlock(&pool_ptr->pool_mtx);
    return NULL;
}

/* Begin a new production on buffer i (must hold mutex). */
static inline void _begin_produce_locked(mpa_ion_buf_pool_t *p, int i, int64_t now_ns)
{
    p->in_use[i] = 1;
    p->producer_active[i] = 1;

    p->client_mask[i] = 0;
    p->n_client_references[i] = 0;

    if (++p->generation[i] == 0)
        p->generation[i] = 1;
    p->bufs[i].generation = p->generation[i];

    p->time_sent_to_clients_ns[i] = now_ns;
}

mpa_ion_buf_t *mpa_ion_buf_pool_claim_unused_buf(mpa_ion_buf_pool_t *pool_ptr)
{
    pthread_mutex_lock(&pool_ptr->pool_mtx);
    int64_t now_ns = _time_monotonic_ns();

    // Find "free" buffer (no producer, no clients)
    for (uint32_t i = 0; i < pool_ptr->n_bufs; i++)
    {
        if (pool_ptr->in_use[i] == 0)
        {
            _begin_produce_locked(pool_ptr, i, now_ns);
            pthread_mutex_unlock(&pool_ptr->pool_mtx);
            return &pool_ptr->bufs[i];
        }
    }

    // Buffer exhaustion: reuse the oldest buffer that is NOT prdoucer_active
    int oldest_idx = -1;
    int64_t oldest_time = -1;

    for (uint32_t i = 0; i < pool_ptr->n_bufs; i++)
    {
        int64_t dt = now_ns - pool_ptr->time_sent_to_clients_ns[i];
        if (dt > oldest_time)
        {
            oldest_time = dt;
            oldest_idx = (int)i;
        }
    }

    if (oldest_idx >= 0)
    {
        _begin_produce_locked(pool_ptr, oldest_idx, now_ns);
        fprintf(stderr, "WARNING: ion buffer pool exhausted — reusing idx %d (age=%.2fms, gen=%u)\n",
                oldest_idx, (double)oldest_time * 1e-6, pool_ptr->generation[oldest_idx]);
        pthread_mutex_unlock(&pool_ptr->pool_mtx);
        return &pool_ptr->bufs[oldest_idx];
    }

    // Last option: Nothing safe to reuse (all buffers producers in flight), return NULL
    fprintf(stderr, "ERROR: ion buffer pool exhausted — could not find buffer to reuse\n");
    pthread_mutex_unlock(&pool_ptr->pool_mtx);
    return NULL;
}

buffer_handle_t *mpa_ion_buf_pool_claim_unused_handle(mpa_ion_buf_pool_t *pool_ptr)
{
    mpa_ion_buf_t *b = mpa_ion_buf_pool_claim_unused_buf(pool_ptr);
    return b ? &b->handle : NULL;
}

int mpa_ion_buf_flag_as_unused_by_id(mpa_ion_buf_pool_t *pool_ptr, int buf_id)
{
    pthread_mutex_lock(&pool_ptr->pool_mtx);
    if (buf_id < 0 || buf_id >= (int)pool_ptr->n_bufs)
    {
        pthread_mutex_unlock(&pool_ptr->pool_mtx);
        return -1;
    }

    pool_ptr->producer_active[buf_id] = 0;
    if (pool_ptr->n_client_references[buf_id] == 0 && pool_ptr->client_mask[buf_id] == 0)
        pool_ptr->in_use[buf_id] = 0;

    pthread_mutex_unlock(&pool_ptr->pool_mtx);
    return 0;
}

int mpa_ion_buf_flag_as_unused_by_handle(mpa_ion_buf_pool_t *pool_ptr, buffer_handle_t *handle)
{
    pthread_mutex_lock(&pool_ptr->pool_mtx);
    for (uint32_t i = 0; i < pool_ptr->n_bufs; ++i)
    {
        if (*handle == pool_ptr->bufs[i].handle)
        {
            int buf_id = pool_ptr->bufs[i].buffer_id;
            if (buf_id < 0 || buf_id >= (int)pool_ptr->n_bufs)
            {
                pthread_mutex_unlock(&pool_ptr->pool_mtx);
                return -1;
            }

            pool_ptr->producer_active[buf_id] = 0;
            if (pool_ptr->n_client_references[buf_id] == 0 && pool_ptr->client_mask[buf_id] == 0)
                pool_ptr->in_use[buf_id] = 0;

            pthread_mutex_unlock(&pool_ptr->pool_mtx);
            return 0;
        }
    }
    pthread_mutex_unlock(&pool_ptr->pool_mtx);
    return -1;
}

int mpa_ion_buf_flag_as_unused_by_vaddress(mpa_ion_buf_pool_t *pool_ptr, void *vaddress)
{
    pthread_mutex_lock(&pool_ptr->pool_mtx);

    for (uint32_t i = 0; i < pool_ptr->n_bufs; i++)
    {
        if (vaddress == pool_ptr->bufs[i].vaddress)
        {
            int buf_id = pool_ptr->bufs[i].buffer_id;
            if (buf_id < 0 || buf_id >= (int)pool_ptr->n_bufs)
            {
                pthread_mutex_unlock(&pool_ptr->pool_mtx);
                return -1;
            }

            pool_ptr->producer_active[buf_id] = 0;
            if (pool_ptr->n_client_references[buf_id] == 0 && pool_ptr->client_mask[buf_id] == 0)
                pool_ptr->in_use[buf_id] = 0;

            pthread_mutex_unlock(&pool_ptr->pool_mtx);
            return 0;
        }
    }

    pthread_mutex_unlock(&pool_ptr->pool_mtx);
    return -1;
}

int mpa_ion_buf_process_release_msg(mpa_ion_buf_pool_t *pool_ptr, ion_buf_release_msg_t msg)
{
    pthread_mutex_lock(&pool_ptr->pool_mtx);

    if (msg.buffer_id < 0 || msg.buffer_id > (int32_t)pool_ptr->n_bufs)
    {
        fprintf(stderr, "Failed to release mpa ion buffer: buffer_id %d is out of bounds\n", msg.buffer_id);
        pthread_mutex_unlock(&pool_ptr->pool_mtx);
        return 1;
    }

    // client_id needs to be [0, 31], since the client mask is a 32 bit value
    if (msg.client_id < 0 || msg.client_id >= 32)
    {
        fprintf(stderr, "Failed to release mpa ion buffer: client_id %d is out of bounds\n", msg.client_id);
        pthread_mutex_unlock(&pool_ptr->pool_mtx);
        return 1;
    }

    int i = msg.buffer_id;

    if (msg.generation != pool_ptr->generation[i])
    {
        fprintf(stderr, "Client (id: %d) failed to release mpa ion buffer, expected gen: %d, but got gen: %d\n",
                msg.client_id, pool_ptr->generation[i], msg.generation);
        pthread_mutex_unlock(&pool_ptr->pool_mtx);
        return 1;
    }

    pool_ptr->client_mask[i] &= ~(1 << msg.client_id);

    if (pool_ptr->n_client_references[i] > 0)
    {
        pool_ptr->n_client_references[i]--;
    }
    if (pool_ptr->n_client_references[i] == 0 && pool_ptr->client_mask[i] == 0)
    {
        pool_ptr->in_use[i] = 0;
    }

    pthread_mutex_unlock(&pool_ptr->pool_mtx);
    return 0;
}


// this function assumes that the buf is in this pool
int mpa_ion_buf_pool_mark_buf_unused(mpa_ion_buf_pool_t *pool_ptr, mpa_ion_buf_t buf)
{
    mpa_ion_buf_flag_as_unused_by_id(pool_ptr, buf.buffer_id);
    return 0;
}


int mpa_ion_buf_pool_get_num_unused(mpa_ion_buf_pool_t *pool_ptr)
{
    pthread_mutex_lock(&pool_ptr->pool_mtx);
    int unused_n = 0;

    for (uint32_t i = 0; i < pool_ptr->n_bufs; i++)
    {
        if (!pool_ptr->in_use[i])
        {
            unused_n++;
        }
    }

    pthread_mutex_unlock(&pool_ptr->pool_mtx);
    return unused_n;
}

#endif
