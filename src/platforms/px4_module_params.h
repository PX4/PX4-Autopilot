/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file px4_module_params.h
 *
 * C++ base class for modules/classes using configuration parameters
 */

#pragma once

#include <containers/List.hpp>

#include "px4_param.h"

class ModuleParams : public ListNode<ModuleParams *>
{
public:

	ModuleParams(ModuleParams *parent)
	{
		if (parent) {
			parent->_children.add(this);
		}
	}

	virtual ~ModuleParams() = default;

	// no copy, assignment, move, move assignment
	ModuleParams(const ModuleParams &) = delete;
	ModuleParams &operator=(const ModuleParams &) = delete;
	ModuleParams(ModuleParams &&) = delete;
	ModuleParams &operator=(ModuleParams &&) = delete;

protected:
	/**
	 * Call this whenever the module gets a parameter change notification. It will automatically
	 * call updateParams() for all children, which then call updateParamsImpl().
	 */
	void updateParams()
	{
		ModuleParams *child = _children.getHead();

		while (child) {
			child->updateParams();
			child = child->getSibling();
		}

		updateParamsImpl();
	}

	/**
	 * The implementation for this is generated with the macro DEFINE_PARAMETERS()
	 */
	virtual void updateParamsImpl() {}

private:
	List<ModuleParams *> _children;
};
