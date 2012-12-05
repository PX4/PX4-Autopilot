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

namespace control
{

Block::Block(Block * parent, const char * name) :
    _name(),
    _parent(parent),
    _firstSibling(NULL),
    _firstChild(NULL),
    _firstParam(NULL),
    _dt(0)
{
    if (parent == NULL)
    {
        strncpy(_name,name,80);
    }
    else
    {
        if (!strcmp(name,""))
        {
            strncpy(_name,parent->getName(),80);
        }
        else
        {
            snprintf(_name,80,"%s_%s", parent->getName(), name);
        }
        getParent()->addChild(this);
    }
}

void Block::updateParams()
{
    BlockParamBase * param = _firstParam;
    int count = 0;
    while (param != NULL)
    {
        if (count++ > maxParamsPerBlock)
        {
            printf("exceeded max params for block: %s\n", getName());
            break;
        }
        param->update();
        param = param->getFirstSibling();
    }

    if (_firstChild != NULL) updateChildParams();
}

void Block::setDt(float dt) {
    _dt = dt;
    Block * child = getFirstChild();
    int count = 0;
    while (child != NULL)
    {
        if (count++ > maxChildrenPerBlock)
        {
            printf("exceeded max children for block: %s\n", getName());
            break;
        }
        child->setDt(dt);
        child = child->getFirstSibling();
    }
}

void Block::updateChildParams() {
    Block * child = getFirstChild();
    int count = 0;
    while (child != NULL)
    {
        if (count++ > maxChildrenPerBlock)
        {
            printf("exceeded max children for block: %s\n", getName());
            break;
        }
        child->updateParams();
        child = child->getFirstSibling();
    }
}

void Block::addChild(Block * child)
{
    child->setFirstSibling(getFirstChild());
    setFirstChild(child);
}
void Block::addParam(BlockParamBase * param)
{
    param->setFirstSibling(getFirstParam());
    setFirstParam(param);
}

} // namespace control
