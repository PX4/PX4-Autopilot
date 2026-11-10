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

#include <px4_platform_common/module_params.h>

void param_get_value(param_t handle, void *value, bool mark_used)
{
	if (mark_used) {
		param_get_mark_used(handle, value);

	} else {
		// the C function: the C++ param_get() wrapper needs the value's type
		(param_get)(handle, value);
	}
}

void param_get_values(const param_t *handles, unsigned count, void *values, bool mark_used)
{
	static_assert(sizeof(float) == 4 && sizeof(int32_t) == 4, "parameter values are 4 bytes");

	for (unsigned i = 0; i < count; i++) {
		param_get_value(handles[i], static_cast<uint8_t *>(values) + 4 * i, mark_used);
	}
}

ModuleParams::ModuleParams(ModuleParams *parent)
{
	setParent(parent);
}

void ModuleParams::setParent(ModuleParams *parent)
{
	if (parent) {
		parent->_children.add(this);
	}

	_parent = parent;
}

void ModuleParams::updateParams()
{
	for (const auto &child : _children) {
		child->updateParams();
	}

	updateParamsImpl();
}

ModuleParams::~ModuleParams()
{
	if (_parent) { _parent->_children.remove(this); }
}
