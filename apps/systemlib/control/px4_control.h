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
 * @file px4_control.h
 *
 * Controller library code
 */

#pragma once

#include <systemlib/control/control.h>
#include <systemlib/param/param.h>
#include "fixedwing.h"

namespace control
{
namespace px4
{

bool isValid(param_t param);
float getFloatParam(param_t param);
param_t findParam(const char * prefix, const char * name);

class __EXPORT BlockLowPass :
    public control::BlockLowPass
{
public:
    BlockLowPass(const char * name, Block * parent) :
        control::BlockLowPass(name, parent),
        _handle_fCut(findParam(getName(),"FCUT"))
    {}
    virtual ~BlockLowPass() {}
    void updateParams()
    {
        Block::updateParams();
        if (isValid(_handle_fCut)) setFCut(getFloatParam(_handle_fCut));
    }
private:
    param_t _handle_fCut;
};

class __EXPORT BlockHighPass :
    public control::BlockHighPass
{
public:
    BlockHighPass(const char * name, Block * parent) :
        control::BlockHighPass(name, parent),
        _handle_fCut(findParam(getName(),"FCUT"))
    {}
    virtual ~BlockHighPass() {}
    void updateParams()
    {
        Block::updateParams();
        if (isValid(_handle_fCut)) setFCut(getFloatParam(_handle_fCut));
    }
private:
    param_t _handle_fCut;
};

class __EXPORT PParam :
    public PBase
{
public:
    PParam(Block * parent) :
        PBase(parent),
        _handle_kP(findParam(parent->getName(),"P"))
    {}
    virtual ~PParam() {};
    void pParamsUpdate()
    {
        if (isValid(_handle_kP)) setKP(getFloatParam(_handle_kP));
    }
private:
    param_t _handle_kP;
};

class __EXPORT IParam :
    public IBase
{
public:
    IParam(Block * parent) :
        IBase(parent),
        _handle_kI(findParam(parent->getName(),"I")),
        _handle_iMin(findParam(parent->getName(),"IMIN")),
        _handle_iMax(findParam(parent->getName(),"IMAX"))
    {}
    virtual ~IParam() {};
    void iParamsUpdate()
    {
        if (isValid(_handle_kI)) setKI(getFloatParam(_handle_kI));
        if (isValid(_handle_iMin)) setIMin(getFloatParam(_handle_iMin));
        if (isValid(_handle_iMax)) setIMax(getFloatParam(_handle_iMax));
    }
private:
    param_t _handle_kI;
    param_t _handle_iMin;
    param_t _handle_iMax;
};

class __EXPORT DParam :
    public DBase
{
public:
    DParam(Block * parent) :
        DBase(parent),
        _handle_kD(findParam(parent->getName(),"D")),
        _handle_fCut(findParam(parent->getName(),"FCUT"))
    {
    }
    virtual ~DParam() {};
    void dParamsUpdate()
    {
        if (isValid(_handle_kD)) setKD(getFloatParam(_handle_kD));
        if (isValid(_handle_fCut)) setFCut(getFloatParam(_handle_fCut));
    }
private:
    param_t _handle_kD;
    param_t _handle_fCut;
};

typedef control::BlockP<PParam> BlockP;
typedef control::BlockPI<PParam,IParam> BlockPI;
typedef control::BlockPI<PParam,IParam> BlockPD;
typedef control::BlockPID<PParam,IParam,DParam> BlockPID;

typedef control::BlockFixedWingStabilization<BlockLowPass, BlockHighPass, BlockP> BlockFixedWingStabilization;

} // namespace px4
} // namespace control
