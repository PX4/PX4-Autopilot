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
 * @file Block.cpp
 *
 * Controller library code
 */

#include <math.h>
#include <string.h>
#include <stdio.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include "Block.hpp"
#include "BlockParam.hpp"

namespace control
{

Block::Block(SuperBlock *parent, const char *name) :
	_name(name),
	_parent(parent),
	_dt(0),
	_subscriptions(),
	_params()
{
	if (getParent() != NULL) {
		getParent()->getChildren().add(this);
	}
}

void Block::getName(char *buf, size_t n)
{
	if (getParent() == NULL) {
		strncpy(buf, _name, n);

	} else {
		char parentName[blockNameLengthMax];
		getParent()->getName(parentName, n);

		if (!strcmp(_name, "")) {
			strncpy(buf, parentName, blockNameLengthMax);

		} else {
			snprintf(buf, blockNameLengthMax, "%s_%s", parentName, _name);
		}
	}
}

void Block::updateParams()
{
	BlockParamBase *param = getParams().getHead();
	int count = 0;

	while (param != NULL) {
		if (count++ > maxParamsPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			printf("exceeded max params for block: %s\n", name);
			break;
		}

		//printf("updating param: %s\n", param->getName());
		param->update();
		param = param->getSibling();
	}
}

void Block::updateSubscriptions()
{
	uORB::SubscriptionNode *sub = getSubscriptions().getHead();
	int count = 0;

	while (sub != NULL) {
		if (count++ > maxSubscriptionsPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			printf("exceeded max subscriptions for block: %s\n", name);
			break;
		}

		sub->update();
		sub = sub->getSibling();
	}
}

void Block::updatePublications()
{
	uORB::PublicationNode *pub = getPublications().getHead();
	int count = 0;

	while (pub != NULL) {
		if (count++ > maxPublicationsPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			printf("exceeded max publications for block: %s\n", name);
			break;
		}

		pub->update();
		pub = pub->getSibling();
	}
}

void SuperBlock::setDt(float dt)
{
	Block::setDt(dt);
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != NULL) {
		if (count++ > maxChildrenPerBlock) {
			char name[40];
			getName(name, 40);
			printf("exceeded max children for block: %s\n", name);
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

	while (child != NULL) {
		if (count++ > maxChildrenPerBlock) {
			char name[40];
			getName(name, 40);
			printf("exceeded max children for block: %s\n", name);
			break;
		}

		child->updateParams();
		child = child->getSibling();
	}
}

void SuperBlock::updateChildSubscriptions()
{
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != NULL) {
		if (count++ > maxChildrenPerBlock) {
			char name[40];
			getName(name, 40);
			printf("exceeded max children for block: %s\n", name);
			break;
		}

		child->updateSubscriptions();
		child = child->getSibling();
	}
}

void SuperBlock::updateChildPublications()
{
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != NULL) {
		if (count++ > maxChildrenPerBlock) {
			char name[40];
			getName(name, 40);
			printf("exceeded max children for block: %s\n", name);
			break;
		}

		child->updatePublications();
		child = child->getSibling();
	}
}


} // namespace control

template class List<uORB::SubscriptionNode *>;
template class List<uORB::PublicationNode *>;
template class List<control::BlockParamBase *>;
