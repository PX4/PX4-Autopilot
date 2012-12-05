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
 * @file Block.h
 *
 * Controller library code
 */

#pragma once

#include <stdint.h>

namespace control
{

static const uint16_t maxChildrenPerBlock = 100;
static const uint16_t maxParamsPerBlock = 100;

// forward declaration
class BlockParamBase;

class __EXPORT Block
{
public:
// methods
    Block(Block * parent, const char * name);
    virtual ~Block() {};
    virtual void updateParams();
// accessors
    void setDt(float dt);
    char * getName() { return _name; }
    float getDt() { return _dt; }
protected:
// methods
    void updateChildParams();
    void addChild(Block * child);
    void addParam(BlockParamBase * param);
// accessors
    Block * getParent() { return _parent; }
    Block * getFirstSibling() { return _firstSibling; }
    void setFirstSibling(Block * firstSibling) { _firstSibling = firstSibling; }
    Block * getFirstChild() { return _firstChild; }
    void setFirstChild(Block * firstChild) { _firstChild = firstChild; }
    BlockParamBase * getFirstParam() { return _firstParam; }
    void setFirstParam(BlockParamBase * firstParam) { _firstParam = firstParam; }
protected:
// attributes
    char _name[80];
    Block * _parent;
    Block * _firstSibling;
    Block * _firstChild;
    BlockParamBase * _firstParam;
    float _dt;
};

} // namespace control
