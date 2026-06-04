/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file vector_mission_item_store.h
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

#include "navigation.h"

namespace navigator_test
{

class VectorMissionItemStore
{
public:
	void setItems(const std::vector<mission_item_s> &items)
	{
		_items = items;
		clearLoadFailures();
	}

	void setLoadFailureIndices(std::initializer_list<int32_t> indices)
	{
		_load_failure_indices.assign(indices.begin(), indices.end());
	}

	void clearLoadFailures()
	{
		_load_failure_indices.clear();
	}

	bool loadItem(int32_t index, mission_item_s &mission_item) const
	{
		if (std::find(_load_failure_indices.begin(), _load_failure_indices.end(), index)
		    != _load_failure_indices.end()) {
			return false;
		}

		if (index < 0 || index >= static_cast<int32_t>(_items.size())) {
			return false;
		}

		mission_item = _items[static_cast<std::size_t>(index)];
		return true;
	}

	std::size_t itemCount() const
	{
		return _items.size();
	}

private:
	std::vector<mission_item_s> _items;
	std::vector<int32_t> _load_failure_indices;
};

} // namespace navigator_test
