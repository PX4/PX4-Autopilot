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
 * @file UOrbPublication.h
 *
 */

#pragma once

#include <uORB/uORB.h>
#include "Block.h"
#include "List.h"


namespace control
{

class Block;

class __EXPORT UOrbPublicationBase : public ListNode<control::UOrbPublicationBase *>
{
public:
    /**
     * Constructor
     *
     * @param meta		The uORB metadata (usually from the ORB_ID() macro)
     *			for the topic.
     * @param interval	An interval period in milliseconds.
     */
    UOrbPublicationBase(
            List<UOrbPublicationBase *> * list,
            const struct orb_metadata * meta);
    void update()
    {
        orb_publish(_meta, _handle, getDataVoidPtr());
    }
    virtual void * getDataVoidPtr() = 0;
    virtual ~UOrbPublicationBase()
    {
        orb_unsubscribe(_handle);
    }
    const struct orb_metadata * getMeta() { return _meta; }
    int getHandle() { return _handle; }
private:
    const struct orb_metadata * _meta;
    orb_advert_t _handle;
};

template<class T>
class UOrbPublication :
    public UOrbPublicationBase,
    public T
{
public:
    UOrbPublication(
            List<UOrbPublicationBase *> * list,
            const struct orb_metadata * meta) :
        UOrbPublicationBase(list, meta),
        T() // initialize data structure to zero
    {
    }
    virtual ~UOrbPublication() {}
    void * getDataVoidPtr() { return (void *)(this); }
};

} // namespace control
