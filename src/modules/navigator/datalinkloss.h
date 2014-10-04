/***************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file datalinkloss.h
 * Helper class for Data Link Loss Mode acording to the OBC rules
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef NAVIGATOR_DATALINKLOSS_H
#define NAVIGATOR_DATALINKLOSS_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <uORB/Subscription.hpp>

#include "navigator_mode.h"
#include "mission_block.h"

class Navigator;

class DataLinkLoss : public MissionBlock
{
public:
	DataLinkLoss(Navigator *navigator, const char *name);

	~DataLinkLoss();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();

private:
	/* Params */
	control::BlockParamFloat _param_commsholdwaittime;
	control::BlockParamInt _param_commsholdlat; // * 1e7
	control::BlockParamInt _param_commsholdlon; // * 1e7
	control::BlockParamFloat _param_commsholdalt;
	control::BlockParamInt _param_airfieldhomelat; // * 1e7
	control::BlockParamInt _param_airfieldhomelon; // * 1e7
	control::BlockParamFloat _param_airfieldhomealt;
	control::BlockParamFloat _param_airfieldhomewaittime;
	control::BlockParamInt _param_numberdatalinklosses;
	control::BlockParamInt _param_skipcommshold;

	enum DLLState {
		DLL_STATE_NONE = 0,
		DLL_STATE_FLYTOCOMMSHOLDWP = 1,
		DLL_STATE_FLYTOAIRFIELDHOMEWP = 2,
		DLL_STATE_TERMINATE = 3,
		DLL_STATE_END = 4
	} _dll_state;

	/**
	 * Set the DLL item
	 */
	void		set_dll_item();

	/**
	 * Move to next DLL item
	 */
	void		advance_dll();

};
#endif
