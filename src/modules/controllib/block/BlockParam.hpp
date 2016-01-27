/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file BlockParam.h
 *
 * Controller library code
 */

#pragma once

#include <systemlib/param/param.h>

#include "Block.hpp"
#include <containers/List.hpp>

namespace control
{

class Block;

/**
 * A base class for block params that enables traversing linked list.
 */
class __EXPORT BlockParamBase : public ListNode<BlockParamBase *>
{
public:
	/**
	 * Instantiate a block param base.
	 *
	 * @param parent_prefix Set to true to include the parent name in the parameter name
	 */
	BlockParamBase(Block *parent, const char *name, bool parent_prefix = true);
	virtual ~BlockParamBase() {};
	virtual void update() = 0;
	const char *getName() { return param_name(_handle); }
protected:
	param_t _handle;
};

/**
 * Parameters that are tied to blocks for updating and nameing.
 */

template <class T>
class BlockParam : public BlockParamBase
{
public:
	BlockParam(Block *block, const char *name,
		   bool parent_prefix = true, T *extern_address = NULL);
	T get();
	void commit();
	void set(T val);
	void update();
	virtual ~BlockParam();
protected:
	T _val;
	T *_extern_address;
};

typedef BlockParam<float> BlockParamFloat;
typedef BlockParam<int> BlockParamInt;

} // namespace control
