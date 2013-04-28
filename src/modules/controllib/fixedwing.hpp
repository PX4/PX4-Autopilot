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
 * @file fixedwing.h
 *
 * Controller library code
 */

#pragma once

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <poll.h>

#include "blocks.hpp"
#include "block/UOrbSubscription.hpp"
#include "block/UOrbPublication.hpp"

extern "C" {
#include <systemlib/geo/geo.h>
}

namespace control
{

namespace fixedwing
{

/**
 * BlockYawDamper
 *
 * This block has more explations to help new developers
 * add their own blocks. It includes a limited explanation
 * of some C++ basics.
 *
 * Block: The generic class describing a typical block as you
 * would expect in Simulink or ScicosLab. A block can have
 * parameters. It cannot have other blocks.
 *
 * SuperBlock: A superblock is a block that can have other
 * blocks. It has methods that manage the blocks beneath it.
 *
 * BlockYawDamper inherits from SuperBlock publically, this
 * means that any public function in SuperBlock are public within
 * BlockYawDamper and may be called from outside the
 * class methods. Any protected function within block
 * are private to the class and may not be called from
 * outside this class. Protected should be preferred
 * where possible to public as it is important to
 * limit access to the bare minimum to prevent
 * accidental errors.
 */
class __EXPORT BlockYawDamper : public SuperBlock
{
private:
	/**
	 * Declaring other blocks used by this block
	 *
	 * In this section we declare all child blocks that
	 * this block is composed of. They are private
	 * members so only this block has direct access to
	 * them.
	 */
	BlockLowPass _rLowPass;
	BlockHighPass _rWashout;
	BlockP _r2Rdr;

	/**
	 * Declaring output values and accessors
	 *
	 * If we have any output values for the block we
	 * declare them here. Output can be directly returned
	 * through the update function, but outputs should be
	 * declared here if the information will likely be requested
	 * again, or if there are multiple outputs.
	 *
	 * You should only be able to set outputs from this block,
	 * so the outputs are declared in the private section.
	 * A get accessor is provided
	 * in the public section for other blocks to get the
	 * value of the output.
	 */
	float _rudder;
public:
	/**
	 * BlockYawDamper Constructor
	 *
	 * The job of the constructor is to initialize all
	 * parameter in this block and initialize all child
	 * blocks. Note also, that the output values must be
	 * initialized here. The order of the members in the
	 * member initialization list should follow the
	 * order in which they are declared within the class.
	 * See the private member declarations above.
	 *
	 * Block Construction
	 *
	 * All blocks are constructed with their parent block
	 * and their name. This allows parameters within the
	 * block to construct a fully qualified name from
	 * concatenating the two. If the name provided to the
	 * block is "", then the block will use the parent
	 * name as it's name. This is useful in cases where
	 * you have a block that has parameters "MIN", "MAX",
	 * such as BlockLimit and you do not want an extra name
	 * to qualify them since the parent block has no "MIN",
	 * "MAX" parameters.
	 *
	 * Block Parameter Construction
	 *
	 * Block parameters are named constants, they are
	 * constructed using:
	 * BlockParam::BlockParam(Block * parent, const char * name)
	 * This funciton takes both a parent block and a name.
	 * The constructore then uses the parent name and the name of
	 * the paramter to ask the px4 param library if it has any
	 * parameters with this name. If it does, a handle to the
	 * parameter is retrieved.
	 *
	 * Block/ BlockParam Naming
	 *
	 * When desigining new blocks, the naming of the parameters and
	 * blocks determines the fully qualified name of the parameters
	 * within the ground station, so it is important to choose
	 * short, easily understandable names. Again, when a name of
	 * "" is passed, the parent block name is used as the value to
	 * prepend to paramters names.
	 */
	BlockYawDamper(SuperBlock *parent, const char *name);
	/**
	 * Block deconstructor
	 *
	 * It is always a good idea to declare a virtual
	 * deconstructor so that upon calling delete from
	 * a class derived from this, all of the
	 * deconstructors all called, the derived class first, and
	 * then the base class
	 */
	virtual ~BlockYawDamper();

	/**
	 * Block update function
	 *
	 * The job of the update function is to compute the output
	 * values for the block. In a simple block with one output,
	 * the output may be returned directly. If the output is
	 * required frequenly by other processses, it might be a
	 * good idea to declare a member to store the temporary
	 * variable.
	 */
	void update(float rCmd, float r);

	/**
	 * Rudder output value accessor
	 *
	 * This is a public accessor function, which means that the
	 * private value _rudder is returned to anyone calling
	 * BlockYawDamper::getRudder(). Note thate a setRudder() is
	 * not provided, this is because the updateParams() call
	 * for a block provides the mechanism for updating the
	 * paramter.
	 */
	float getRudder() { return _rudder; }
};

/**
 * Stability augmentation system.
 * Aircraft Control and Simulation, Stevens and Lewis, pg. 292, 299
 */
class __EXPORT BlockStabilization : public SuperBlock
{
private:
	BlockYawDamper _yawDamper;
	BlockLowPass _pLowPass;
	BlockLowPass _qLowPass;
	BlockP _p2Ail;
	BlockP _q2Elv;
	float _aileron;
	float _elevator;
public:
	BlockStabilization(SuperBlock *parent, const char *name);
	virtual ~BlockStabilization();
	void update(float pCmd, float qCmd, float rCmd,
		    float p, float q, float r);
	float getAileron() { return _aileron; }
	float getElevator() { return _elevator; }
	float getRudder() { return _yawDamper.getRudder(); }
};

/**
 * Frontside/ Backside Control Systems
 *
 * Frontside :
 *   velocity error -> throttle
 *   altitude error -> elevator
 *
 * Backside :
 *   velocity error -> elevator
 *   altitude error -> throttle
 *
 * Backside control systems are more resilient at
 * slow speeds on the back-side of the power
 * required curve/ landing etc. Less performance
 * than frontside at high speeds.
 */

/**
 * Waypoint Guidance block
 */
class __EXPORT BlockWaypointGuidance : public SuperBlock
{
private:
	BlockLimitSym _xtYawLimit;
	BlockP _xt2Yaw;
	float _psiCmd;
public:
	BlockWaypointGuidance(SuperBlock *parent, const char *name);
	virtual ~BlockWaypointGuidance();
	void update(vehicle_global_position_s &pos,
		    vehicle_attitude_s &att,
		    vehicle_global_position_setpoint_s &posCmd,
		    vehicle_global_position_setpoint_s &lastPosCmd);
	float getPsiCmd() { return _psiCmd; }
};

/**
 * UorbEnabledAutopilot
 */
class __EXPORT BlockUorbEnabledAutopilot : public SuperBlock
{
protected:
	// subscriptions
	UOrbSubscription<vehicle_attitude_s> _att;
	UOrbSubscription<vehicle_attitude_setpoint_s> _attCmd;
	UOrbSubscription<vehicle_rates_setpoint_s> _ratesCmd;
	UOrbSubscription<vehicle_global_position_s> _pos;
	UOrbSubscription<vehicle_global_position_setpoint_s> _posCmd;
	UOrbSubscription<manual_control_setpoint_s> _manual;
	UOrbSubscription<vehicle_status_s> _status;
	UOrbSubscription<parameter_update_s> _param_update;
	// publications
	UOrbPublication<actuator_controls_s> _actuators;
public:
	BlockUorbEnabledAutopilot(SuperBlock *parent, const char *name);
	virtual ~BlockUorbEnabledAutopilot();
};

/**
 * Multi-mode Autopilot
 */
class __EXPORT BlockMultiModeBacksideAutopilot : public BlockUorbEnabledAutopilot
{
private:
	// stabilization
	BlockStabilization _stabilization;

	// heading hold
	BlockP _psi2Phi;
	BlockP _phi2P;
	BlockLimitSym _phiLimit;

	// velocity hold
	BlockPID _v2Theta;
	BlockPID _theta2Q;
	BlockLimit _theLimit;
	BlockLimit _vLimit;

	// altitude/ roc hold
	BlockPID _h2Thr;
	BlockPID _roc2Thr;

	// guidance
	BlockWaypointGuidance _guide;

	// block params
	BlockParam<float> _trimAil;
	BlockParam<float> _trimElv;
	BlockParam<float> _trimRdr;
	BlockParam<float> _trimThr;
	BlockParam<float> _vCmd;
	BlockParam<float> _rocMax;

	struct pollfd _attPoll;
	vehicle_global_position_setpoint_s _lastPosCmd;
	enum {CH_AIL, CH_ELV, CH_RDR, CH_THR};
	uint64_t _timeStamp;
public:
	BlockMultiModeBacksideAutopilot(SuperBlock *parent, const char *name);
	void update();
	virtual ~BlockMultiModeBacksideAutopilot();
};


} // namespace fixedwing

} // namespace control

