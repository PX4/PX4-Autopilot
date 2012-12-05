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
 * @file control.h
 *
 * Controller library code
 */

#pragma once

#include <systemlib/param/param.h>
#include <string.h>
#include <stdio.h>

namespace control
{

static const uint16_t maxChildren = 100;

class Block
{
public:
// methods
    Block(Block * parent, const char * name) :
        _name(),
        _parent(parent),
        _firstSibling(NULL),
        _firstChild(NULL)
    {
        if (parent == NULL)
        {
            strncpy(_name,name,80);
        }
        else
        {
            snprintf(_name,80,"%s_%s", parent->getName(), name);
            getParent()->addChild(this);
        }
    }
    virtual ~Block() {};
    virtual void updateParams()
    {
        if (_firstChild != NULL) updateChildParams();
    };
// accessors
    void setDt(float dt) {
        _dt = dt;
        Block * child = _firstChild;
        int count = 0;
        while (1)
        {
            if (child == NULL) break;
            if (count++ > maxChildren)
            {
                printf("exceeded max children for block: %s\n", getName());
                break;
            }
            child->setDt(dt);
            child = child->_firstSibling;
        }
    }
    char * getName() { return _name; }
    float getDt() { return _dt; }
protected:
// methods
    void updateChildParams() {
        Block * child = _firstChild;
        int count = 0;
        while (1)
        {
            if (child == NULL) break;
            if (count++ > maxChildren)
            {
                printf("exceeded max children for block: %s\n", getName());
                break;
            }
            child->updateParams();
            child = child->_firstSibling;
        }
    }
    void addChild(Block * child) {
        child->_firstSibling = _firstChild;
        _firstChild = child;
    }
    //void addParam(BlockParamBase * param) {
        //param->_firstSibling = _firstParam;
        //_firstParam = param;
    //}
// accessors
    Block * getParent() { return _parent; }
    Block * getFirstSibling() { return _firstSibling; }
    Block * getFirstChild() { return _firstChild; }
protected:
// attributes
    char _name[80];
    Block * _parent;
    Block * _firstSibling;
    Block * _firstChild;
    //BlockParamBase * _firstParam;
    float _dt;
};

/**
 * A base class for block params that enables traversing linked list.
 */
class BlockParamBase
{
public:
    BlockParamBase(Block * parent, const char * name) :
        _handle(PARAM_INVALID), 
        _nextBlockParam(NULL)
    {
        char fullname[80];
        if (parent == NULL) snprintf(fullname,80,"%s", name);
        else snprintf(fullname,80,"%s_%s", parent->getName(), name);

        _handle = param_find(fullname);
        if (_handle == PARAM_INVALID)
            printf("error finding param: %s\n", fullname);
    };
    virtual ~BlockParamBase() {};
    virtual void update() = 0;
    const char * getName() {
        return param_name(_handle);
    }
    BlockParamBase * getNextParam() { return _nextBlockParam; }
protected:
    param_t _handle;
    BlockParamBase * _nextBlockParam;
};

/**
 * Parameters that are tied to blocks for updating and nameing.
 */
template<class T>
class BlockParam : public BlockParamBase
{
public:
    BlockParam(Block * block, const char * name) :
        BlockParamBase(block, name),
        _val(0)
    {
    }
    T get() { return _val; }
    void set(T val) { _val == val; }
    void update()
    {
        if (_handle != PARAM_INVALID) param_get(_handle,&_val);
    }
protected:
    T _val;
};

/**
 * A limiter/ saturation.
 * The output of update is the input, bounded
 * by min/max.
 */
class __EXPORT BlockLimit : public Block
{
public:
// methods
    BlockLimit(Block * parent, const char * name) : 
        Block(parent, name),
        _min(this,"MIN"),
        _max(this,"MAX")
    {};
    virtual ~BlockLimit() {};
    float update(float input);
protected:
    BlockParam<float> _min;
    BlockParam<float> _max;
};

/**
 * A low pass filter as described here: 
 * http://en.wikipedia.org/wiki/Low-pass_filter.
 */
class __EXPORT BlockLowPass : public Block
{
public:
// methods
    BlockLowPass(Block * parent, const char * name) :
        Block(parent, name),
        _state(0),
        _fCut(this,"FCUT")
    {};
    virtual ~BlockLowPass() {};
    float update(float input);
protected:
    float getState() { return _state; }
    void setState(float state) { _state = state; }
    float _state;
    BlockParam<float> _fCut;
};


} // namespace control


/**
 * A high pass filter as described here: 
 * http://en.wikipedia.org/wiki/Low-pass_filter.
 */
//class __EXPORT HighPass
//{
//public:
//// methods
    //HighPass() : _state(0), _fCut(0) {};
    //virtual ~HighPass() {};
    //float update(float input, float dt);
//protected:
//// accessors
    //void setState(float state) {_state = state;}
    //float getState() {return _state;}
    //void setFCut(float fCut) {_fCut=fCut;}
    //float getFCut() {return _fCut;}
//// attributes
    //float _state; [>*< previous output <]
    //float _fCut; [>*< cut-off frequency, Hz <]
//};

/**
 * A trapezoidal integrator.
 * http://en.wikipedia.org/wiki/Trapezoidal_rule
 * A limiter is built into the class to bound the
 * integral's internal state. This is important
 * for windup protection.
 * @see Limit
 */
//class __EXPORT Integral
//{
//public: 
//// methods
    //Integral() : _state(0), _limit() {};
    //virtual ~Integral() {};
    //float update(float input, float dt);
//// accessors
    //void setState(float state) {_state = state;}
    //float getState() {return _state;}
    //void setMin(float min) {_limit.setMin(min);}
    //float getMin() {return _limit.getMin();}
    //void setMax(float max) {_limit.setMax(max);}
    //float getMax() {return _limit.getMax();}
//protected:
//// attributes
    //float _state; [>*< previous output <]
    //Limit _limit; [>*< limiter <]
//};

/**
 * A simple derivative approximation.
 * This uses the previous and current input.
 * This has a built in low pass filter.
 * @see LowPass
 */
//class __EXPORT Derivative
//{
//public:
//// methods
    //Derivative() : _state(0) {};
    //virtual ~Derivative() {};
    //float update(float input, float dt);
//// accessors
    //void setState(float state) {_state = state;}
    //float getState() {return _state;}
    //void setFCut(float fCut) {_lowPass.setFCut(fCut);}
    //float getFCut() {return _lowPass.getFCut();}
//protected:
//// attributes
    //float _state; [>*< previous input <]
    //LowPass _lowPass; [>*< low pass filter <]
//};

/**
 * Proportional control base class.
 */
//class __EXPORT PBase
//{
//public:
//// methods
    //PBase(Block * parent) : _kP(0) {};
    //virtual ~PBase() {};
    //virtual void pParamsUpdate() = 0;
//protected:
//// accessors
    //void setKP(float kP) {_kP = kP;}
    //float getKP() {return _kP;}
//// attributes
    //float _kP; [>*< proportional gain <]
//};

/**
 * Integral control base class.
 */
//class __EXPORT IBase
//{
//public:
//// methods
    //IBase(Block * parent) : _kI(0), _integral() {};
    //virtual ~IBase() {};
    //virtual void iParamsUpdate() = 0;
//protected:
//// accessors
    //void setKI(float kI) {_kI = kI;}
    //float getKI() {return _kI;}
    //void setIMin(float min) {IBase::getIntegral().setMin(min);}
    //float getIMin() {return IBase::getIntegral().getMin();}
    //void setIMax(float max) {IBase::getIntegral().setMax(max);}
    //float getIMax() {return IBase::getIntegral().getMax();}
//// protected methods
    //Integral & getIntegral() {return _integral;}
//// attributes
    //float _kI; [>*< proportional gain <]
    //Integral _integral; [>*< integral calculator <]
//};

/**
 * Integral control base class.
 */
//class __EXPORT DBase
//{
//public:
//// methods
    //DBase(Block * parent) : _kD(0), _derivative() {};
    //virtual ~DBase() {};
    //virtual void dParamsUpdate() = 0;
//protected:
//// accessors
    //void setKD(float kD) {_kD = kD;}
    //float getKD() {return _kD;}
    //void setFCut(float fCut) {DBase::getDerivative().setFCut(fCut);}
    //float getFCut() {return DBase::getDerivative().getFCut();}
//// attributes
    //float _kD; [>*< proportional gain <]
    //Derivative _derivative; [>*< derivative calculator <]
//// methods
    //Derivative & getDerivative() {return _derivative;}
//};

/**
 * A constant block.
 */
//class __EXPORT BlockConstant:
    //public Block
//{
//public:
//// methods
    //BlockConstant(const char * name, Block * parent) : Block(name, parent) {};
    //virtual ~BlockConstant() {};
//protected:
//// accessors
   //void set(float val) { _val = val; }
   //float get() { return _val; }
//// attributes
    //float _val;
//};

/**
 * A limiter/ saturation block.
 */
//class __EXPORT BlockLimit:
    //public Block,
    //public Limit
//{
//public:
//// methods
    //BlockLimit(const char * name, Block * parent) : Block(name, parent), Limit() {};
    //virtual ~BlockLimit() {};
//};



/**
 * A low pass filter block.
 * @link http://en.wikipedia.org/wiki/Low-Pass_filter
 */
//class __EXPORT BlockLowPass:
    //public Block,
    //public LowPass
//{
//public:
//// methods
    //BlockLowPass(const char * name, Block * parent) : Block(name, parent), LowPass() {};
    //float update(float input) { return LowPass::update(input,getDt()); }
    //virtual ~BlockLowPass() {};
//};

/**
 * A high pass filter block.
 * @link http://en.wikipedia.org/wiki/High-Pass_filter
 */
//class __EXPORT BlockHighPass:
    //public Block,
    //public HighPass
//{
//public:
//// methods
    //BlockHighPass(const char * name, Block * parent) : Block(name, parent), HighPass() {};
    //float update(float input) { return HighPass::update(input,getDt()); }
    //virtual ~BlockHighPass() {};
//};

/**
 * A proportional controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
//template<class PBase=PBase>
//class __EXPORT BlockP:
    //public Block,
    //public PBase
//{
//public:
//// methods
    //BlockP(const char * name, Block * parent) :
        //Block(name, parent),
        //PBase(this)
    //{};
    //virtual ~BlockP() {};
    //float update(float input)
    //{
        //return PBase::getKP()*input;
    //}
    //void updateParams()
    //{
        //Block::updateParams();
        //PBase::pParamsUpdate();
    //} 
//};

/**
 * A proportional-integral controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
//template<class PBase=PBase, class IBase=IBase>
//class __EXPORT BlockPI:
    //public Block,
    //public PBase,
    //public IBase
//{
//public:
//// methods
    //BlockPI(const char * name, Block * parent) :
        //Block(name, parent),
        //PBase(this),
        //IBase(this)
    //{};
    //virtual ~BlockPI() {};
    //float update(float input)
    //{
        //return PBase::getKP()*input +
            //IBase::getKI()*IBase::getIntegral().update(input, getDt());
    //}
    //void updateParams()
    //{
        //Block::updateParams();
        //PBase::pParamsUpdate();
        //IBase::iParamsUpdate();
    //}
//};

/**
 * A proportional-derivative controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
//template<class PBase=PBase, class DBase=DBase>
//class __EXPORT PDBlock:
    //public Block,
    //public PBase,
    //public DBase
//{
//public:
//// methods
    //PDBlock(const char * name, Block * parent) :
        //Block(name, parent),
        //PBase(this),
        //DBase(this)
    //{};
    //virtual ~PDBlock() {};
    //float update(float input)
    //{
        //return PBase::getKP()*input + 
            //DBase::getKD()*DBase::getDerivative().update(input, getDt());
    //}
    //void updateParams()
    //{
        //Block::updateParams();
        //PBase::pParamsUpdate();
        //DBase::dParamsUpdate();
    //}
//};

/**
 * A proportional-integral-derivative controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
//template<class PBase=PBase, class IBase=IBase, class DBase=DBase>
//class __EXPORT BlockPID:
    //public Block,
    //public PBase,
    //public IBase,
    //public DBase
//{
//public:
//// methods
    //BlockPID(const char * name, Block * parent) :
        //Block(name, parent),
        //PBase(this),
        //IBase(this),
        //DBase(this)
    //{};
    //virtual ~BlockPID() {};
    //float update(float input)
    //{
        //return PBase::getKP()*input + 
            //IBase::getKI()*IBase::getIntegral().update(input, getDt()) +
            //DBase::getKD()*DBase::getDerivative().update(input, getDt());
    //}
    //void updateParams()
    //{
        //Block::updateParams();
        //PBase::pParamsUpdate();
        //IBase::iParamsUpdate();
        //DBase::dParamsUpdate();
    //}
//};

//} // namespace control
