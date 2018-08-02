/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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
 * @file threadsafe_map.h
 *
 * This is a small wrapper for std::map to make it thread-safe.
 *
 * @author Julian Oes <julian@oes.ch>
 */
#pragma once

#include <pthread.h>
#include <map>


namespace px4_daemon
{

template <class Key, class Type>
class ThreadsafeMap
{
public:
	ThreadsafeMap() :
		_mutex(PTHREAD_MUTEX_INITIALIZER) {};
	~ThreadsafeMap() {};

	/**
	 * Insert an entry into the map.
	 */
	void
	insert(std::pair<Key, Type> pair)
	{
		_lock();
		_map.insert(pair);
		_unlock();
	}

	/**
	 * Get an entry into from the map.
	 */
	Type get(Key key)
	{
		// Supposedly locking is not needed to read but since we're
		// not sure, let's lock anyway.
		_lock();
		const Type temp = _map[key];
		_unlock();
		return temp;
	}

	void erase(Key key)
	{
		_lock();
		_map.erase(key);
		_unlock();
	}
private:
	void _lock()
	{
		pthread_mutex_lock(&_mutex);
	}

	void _unlock()
	{
		pthread_mutex_unlock(&_mutex);
	}

	std::map<Key, Type> _map;
	pthread_mutex_t _mutex;

};

} // namespace px4_daemon

