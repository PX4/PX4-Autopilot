/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include "block/Block.hpp"

#include <conversion/rotation.h>
#include <matrix/math.hpp>
#include <px4_module_params.h>

namespace control
{

class __EXPORT BlockBoardRotation : public SuperBlock, public ModuleParams
{
public:

	BlockBoardRotation(SuperBlock *parent = nullptr) :
		SuperBlock(parent, ""),
		ModuleParams(nullptr)
	{
		_board_rotation.setIdentity();

		updateParams();
	}

	~BlockBoardRotation() = default;

	// no copy, assignment, move, move assignment
	BlockBoardRotation(const BlockBoardRotation &) = delete;
	BlockBoardRotation &operator=(const BlockBoardRotation &) = delete;
	BlockBoardRotation(BlockBoardRotation &&) = delete;
	BlockBoardRotation &operator=(BlockBoardRotation &&) = delete;


	const matrix::Dcmf &get() { return _board_rotation; }

	void updateParams()
	{
		SuperBlock::updateParams();

		const matrix::Dcmf offset(matrix::Eulerf(
						  math::radians(_sens_board_offset_x.get()),
						  math::radians(_sens_board_offset_y.get()),
						  math::radians(_sens_board_offset_z.get())));

		_board_rotation = offset * get_rot_matrix((enum Rotation)_sens_board_rot.get());
	}

private:

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_BOARD_ROT>) _sens_board_rot,

		(ParamFloat<px4::params::SENS_BOARD_X_OFF>) _sens_board_offset_x,
		(ParamFloat<px4::params::SENS_BOARD_Y_OFF>) _sens_board_offset_y,
		(ParamFloat<px4::params::SENS_BOARD_Z_OFF>) _sens_board_offset_z
	)

	matrix::Dcmf _board_rotation;

};

} // namespace control
