/****************************************************************************
 *
 *   Copyright (C) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file Block.cpp
 *
 * Controller library code
 */

#include "Block.hpp"
#include "BlockParam.hpp"

#include <cstdio>
#include <cstring>

#include <px4_platform_common/log.h>

namespace control
{

Block::Block(SuperBlock *parent, const char *name) :
	_name(name),
	_parent(parent)
{
	if (getParent() != nullptr) {
		getParent()->getChildren().add(this);
	}
}

void Block::getName(char *buf, size_t n)
{
	if (n == 0 || buf == nullptr) { return; }

	strncpy(buf, _name, n);
	buf[n - 1] = '\0';

	Block *p = getParent();

	// traverse through parents and prepend their names to the buffer
	while (p != nullptr) {
		if (p->_name[0] != '\0') {
			char temp[blockNameLengthMax];
			strncpy(temp, buf, sizeof(temp));
			temp[sizeof(temp) - 1] = '\0';

			if (buf[0] == '\0') {
				strncpy(buf, p->_name, n);

			} else {
				snprintf(buf, n, "%s_%s", p->_name, temp);
			}

			buf[n - 1] = '\0';
		}

		p = p->getParent();
	}
}

void Block::updateParams()
{
	BlockParamBase *param = getParams().getHead();
	int count = 0;

	while (param != nullptr) {
		if (count++ > maxParamsPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max params for block: %s", name);
			break;
		}

		//printf("updating param: %s\n", param->getName());
		param->update();
		param = param->getSibling();
	}

	updateParamsSubclass();
}

void SuperBlock::setDt(float dt)
{
	Block::setDt(dt);
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != nullptr) {
		if (count++ > maxChildrenPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max children for block: %s", name);
			break;
		}

		child->setDt(dt);
		child = child->getSibling();
	}
}

void SuperBlock::updateChildParams()
{
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != nullptr) {
		if (count++ > maxChildrenPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max children for block: %s", name);
			break;
		}

		child->updateParams();
		child = child->getSibling();
	}
}

} // namespace control

template class List<control::BlockParamBase *>;
