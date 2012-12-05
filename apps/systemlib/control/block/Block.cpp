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

#include "Block.h"
#include "BlockParam.h"
#include "UOrbSubscription.h"

namespace control
{

Block::Block(Block * parent, const char * name) :
    _name(),
    _parent(parent),
    _sibling(NULL),
    _child(NULL),
    _param(NULL),
    _subscription(NULL),
    _dt(0)
{
    if (parent == NULL)
    {
        strncpy(getName(),name,80);
    }
    else
    {
        if (!strcmp(name,""))
        {
            strncpy(getName(),parent->getName(),80);
        }
        else
        {
            snprintf(getName(),80,"%s_%s", parent->getName(), name);
        }
        getParent()->addChild(this);
    }
}

void Block::updateParams()
{
    BlockParamBase * param = getParam();
    int count = 0;
    while (param != NULL)
    {
        if (count++ > maxParamsPerBlock)
        {
            printf("exceeded max params for block: %s\n", getName());
            break;
        }
        param->update();
        param = param->getSibling();
    }

    if (getChild() != NULL) updateChildParams();
}

void Block::addParam(BlockParamBase * param)
{
    param->setSibling(getParam());
    setParam(param);
}

void Block::updateSubscriptions()
{
    UOrbSubscriptionBase * sub = getSubscription();
    int count = 0;
    while (sub != NULL)
    {
        if (count++ > maxSubscriptionsPerBlock)
        {
            printf("exceeded max subscriptions for block: %s\n", getName());
            break;
        }
        sub->update();
        sub = sub->getSibling();
    }

    if (getChild() != NULL) updateChildSubscriptions();

}

void Block::addSubscription(UOrbSubscriptionBase * sub)
{
    sub->setSibling(getSubscription());
    setSubscription(sub);
}

void Block::setDt(float dt) {
    _dt = dt;
    Block * child = getChild();
    int count = 0;
    while (child != NULL)
    {
        if (count++ > maxChildrenPerBlock)
        {
            printf("exceeded max children for block: %s\n", getName());
            break;
        }
        child->setDt(dt);
        child = child->getSibling();
    }
}

void Block::updateChildParams() {
    Block * child = getChild();
    int count = 0;
    while (child != NULL)
    {
        if (count++ > maxChildrenPerBlock)
        {
            printf("exceeded max children for block: %s\n", getName());
            break;
        }
        child->updateParams();
        child = child->getSibling();
    }
}

void Block::updateChildSubscriptions() {
    Block * child = getChild();
    int count = 0;
    while (child != NULL)
    {
        if (count++ > maxChildrenPerBlock)
        {
            printf("exceeded max children for block: %s\n", getName());
            break;
        }
        child->updateSubscriptions();
        child = child->getSibling();
    }
}

void Block::addChild(Block * child)
{
    child->setSibling(getChild());
    setChild(child);
}






} // namespace control
