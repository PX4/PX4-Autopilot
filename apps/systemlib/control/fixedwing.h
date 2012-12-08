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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <poll.h>

#include "blocks.h"
#include "block/UOrbSubscription.h"
#include "block/UOrbPublication.h"

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
class BlockYawDamper : public SuperBlock
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
    BlockYawDamper(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _rLowPass(this, "R_LP"),
        _rWashout(this, "R_HP"),
        _r2Rdr(this, "R2RDR"),
        _rudder(0)
    {
    }
    /**
     * Block deconstructor
     *
     * It is always a good idea to declare a virtual 
     * deconstructor so that upon calling delete from
     * a class derived from this, all of the
     * deconstructors all called, the derived class first, and
     * then the base class
     */
    virtual ~BlockYawDamper() {};

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
    void update(float rCmd, float r)
    {
        _rudder = _r2Rdr.update(rCmd -
                _rWashout.update(_rLowPass.update(r)));
    }

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
class BlockStabilization : public SuperBlock
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
    BlockStabilization(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _yawDamper(this, ""),
        _pLowPass(this, "P_LP"),
        _qLowPass(this, "Q_LP"),
        _p2Ail(this, "P2AIL"),
        _q2Elv(this, "Q2ELV"),
        _aileron(0),
        _elevator(0)
    {
    }
    virtual ~BlockStabilization() {};
    void update(float pCmd, float qCmd, float rCmd,
            float p, float q, float r)
    {
        _aileron = _p2Ail.update(
                    pCmd - _pLowPass.update(p));
        _elevator = _q2Elv.update(
                    qCmd - _qLowPass.update(q));
        _yawDamper.update(rCmd, r);
    }
    float getAileron() { return _aileron; }
    float getElevator() { return _elevator; }
    float getRudder() { return _yawDamper.getRudder(); }
};

/**
 * Heading hold autopilot block.
 * Aircraft Control and Simulation, Stevens and Lewis
 * Heading hold, pg. 348
 */
class BlockHeadingHold : public SuperBlock
{
private:
    BlockP _psi2Phi;
    BlockP _p2Phi;
    BlockP _phi2Ail;
    BlockLimitSym _phiLimit;
    float _aileron;
public:
    BlockHeadingHold(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _psi2Phi(this, "PSI2PHI"),
        _p2Phi(this, "P2PHI"),
        _phi2Ail(this, "PHI2AIL"),
        _phiLimit(this, "PHI_LIM"),
        _aileron(0)
    {
    }
    virtual ~BlockHeadingHold() {};
    void update(float psiCmd, float phi, float psi, float p)
    {
        float psiError = _wrap_pi(psiCmd - psi);
        float phiCmd = _phiLimit.update(_psi2Phi.update(psiError));
        float phiError = phiCmd - phi;
        _aileron = _phi2Ail.update(phiError - _p2Phi.update(p));
    }
    float getAileron() { return _aileron; }
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
 * Backside velocity hold autopilot block.
 * v -> theta -> q -> elevator
 */
class BlockVelocityHoldBackside : public SuperBlock
{
private:
    BlockPID _v2Theta;
    BlockPID _theta2Q;
    float _elevator;
public:
    BlockVelocityHoldBackside(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _v2Theta(this,"V2THE"),
        _theta2Q(this,"THE2Q"),
        _elevator(0)
    {
    }
    virtual ~BlockVelocityHoldBackside() {};
    void update(float vCmd, float v, float theta, float q)
    {
        float thetaCmd = _v2Theta.update(vCmd - v);
        float qCmd = _theta2Q.update(thetaCmd - theta);
        _elevator = qCmd - q;
    }
    float getElevator() { return _elevator; }
};

/**
 * Frontside velocity hold autopilot block.
 * v -> throttle
 */
class BlockVelocityHoldFrontside : public SuperBlock
{
private:
    BlockPID _v2Thr;
    float _throttle;
public:
    BlockVelocityHoldFrontside(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _v2Thr(this,"V2THR"),
        _throttle(0)
    {
    }
    virtual ~BlockVelocityHoldFrontside() {};
    void update(float vCmd, float v)
    {
        _throttle = _v2Thr.update(vCmd - v);
    }
    float getThrottle() { return _throttle; }
};

/**
 * Backside altitude hold autopilot block.
 * h -> throttle
 */
class BlockAltitudeHoldBackside : public SuperBlock
{
private:
    BlockPID _h2Thr;
    float _throttle;
public:
    BlockAltitudeHoldBackside(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _h2Thr(this, "H2THR"),
        _throttle(0)
    {
    }
    virtual ~BlockAltitudeHoldBackside() {};
    void update(float hCmd, float h)
    {
        _throttle = _h2Thr.update(hCmd - h);
    }
    float getThrottle() { return _throttle; }
};

/**
 * Frontside altitude hold autopilot block.
 * h -> theta > q -> elevator
 */
class BlockAltitudeHoldFrontside : public SuperBlock
{
private:
    BlockPID _h2Theta;
    BlockPID _theta2Q;
    float _elevator;
public:
    BlockAltitudeHoldFrontside(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _h2Theta(this, "H2THE"),
        _theta2Q(this, "THE2Q"),
        _elevator(0)
    {
    }
    virtual ~BlockAltitudeHoldFrontside() {};
    void update(float hCmd, float h, float theta, float q)
    {
        float thetaCmd = _h2Theta.update(hCmd - h);
        float qCmd = _theta2Q.update(thetaCmd - theta);
        _elevator = qCmd - q;
    }
    float getElevatorCmd() { return _elevator; }
};

/**
 * Backside autopilot
 */
class BlockBacksideAutopilot : public SuperBlock
{
private:
    BlockYawDamper _yawDamper;
    BlockHeadingHold _headingHold;
    BlockVelocityHoldBackside _velocityHold;
    BlockAltitudeHoldBackside _altitudeHold;
public:
    BlockBacksideAutopilot(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _yawDamper(this,""),
        _headingHold(this,""),
        _velocityHold(this,""),
        _altitudeHold(this,"")
    {
    }
    virtual ~BlockBacksideAutopilot() {};
    void update(float hCmd, float vCmd, float rCmd, float psiCmd,
            float h, float v,
            float phi, float theta, float psi,
            float p, float q, float r)
    {
        _yawDamper.update(rCmd, r);
        _headingHold.update(psiCmd, phi, psi, p);
        _velocityHold.update(vCmd, v, theta, q);
        _altitudeHold.update(hCmd, h);
    };
    float getRudder() { return _yawDamper.getRudder(); }
    float getAileron() { return _headingHold.getAileron(); }
    float getElevator() { return _velocityHold.getElevator(); }
    float getThrottle() { return _altitudeHold.getThrottle(); }
};

/**
 * Output conditioning block
 */
class BlockOutputs : public SuperBlock
{
private:
    BlockOutput _aileron;
    BlockOutput _elevator;
    BlockOutput _rudder;
    BlockOutput _throttle;
public:
    BlockOutputs(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        _aileron(this,"AIL"),
        _elevator(this,"ELV"),
        _rudder(this,"RDR"),
        _throttle(this,"THR")
    {
    }
    virtual ~BlockOutputs() {};
    void update(float aileron, float elevator, float rudder, float throttle)
    {
        _aileron.update(aileron);
        _elevator.update(elevator);
        _rudder.update(rudder);
        _throttle.update(throttle);
    }
    void setActuators(actuator_controls_s & actuators)
    {
        actuators.control[0] = getAileron();
        actuators.control[1] = getElevator();
        actuators.control[2] = getRudder();
        actuators.control[3] = getThrottle();
    }
    float getAileron() { return _aileron.get(); }
    float getElevator() { return _elevator.get(); }
    float getRudder() { return _rudder.get(); }
    float getThrottle() { return _throttle.get(); }
};

/**
 * Waypoint Guidance block
 */
class BlockWaypointGuidance : public SuperBlock
{
private:
    BlockLimitSym _xtYawLimit;
    BlockP _xt2Yaw;
    float _psiCmd;
public:
    BlockWaypointGuidance(SuperBlock * parent, const char * name) :
        SuperBlock(parent,name),
        _xtYawLimit(this,"XT2YAW"),
        _xt2Yaw(this,"XT2YAW"),
        _psiCmd(0)
    {
    }
    void update(vehicle_global_position_s & pos,
            vehicle_attitude_s & att,
            vehicle_global_position_setpoint_s & posCmd,
            vehicle_global_position_setpoint_s & lastPosCmd) {

        // heading to waypoint
        float psiTrack = get_bearing_to_next_waypoint(
                (double)pos.lat / (double)1e7d,
                (double)pos.lon / (double)1e7d,
                (double)posCmd.lat / (double)1e7d,
                (double)posCmd.lon / (double)1e7d);

        // cross track
        struct crosstrack_error_s xtrackError;
        get_distance_to_line(&xtrackError,
                (double)pos.lat / (double)1e7d,
                (double)pos.lon / (double)1e7d,
                (double)lastPosCmd.lat / (double)1e7d,
                (double)lastPosCmd.lon / (double)1e7d,
                (double)posCmd.lat / (double)1e7d,
                (double)posCmd.lon / (double)1e7d);

        _psiCmd = _wrap_2pi(psiTrack + 
            _xtYawLimit.update(_xt2Yaw.update(xtrackError.distance)));
    }
    float getPsiCmd() { return _psiCmd; }
};

/**
 * UorbEnabledAutopilot
 */
class BlockUorbEnabledAutopilot : public SuperBlock
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
    // publications
    UOrbPublication<actuator_controls_s> _actuators;
public:
    BlockUorbEnabledAutopilot(SuperBlock * parent, const char * name) :
        SuperBlock(parent, name),
        // subscriptions
        _att(&getSubscriptions(), ORB_ID(vehicle_attitude),20),
        _attCmd(&getSubscriptions(), ORB_ID(vehicle_attitude_setpoint),20),
        _ratesCmd(&getSubscriptions(), ORB_ID(vehicle_rates_setpoint),20),
        _pos(&getSubscriptions() ,ORB_ID(vehicle_global_position),20),
        _posCmd(&getSubscriptions(), ORB_ID(vehicle_global_position_setpoint),20),
        _manual(&getSubscriptions(), ORB_ID(manual_control_setpoint),20),
        _status(&getSubscriptions(), ORB_ID(vehicle_status),20),
        // publications
        _actuators(&getPublications(), ORB_ID(actuator_controls_0))
    {
    }
};

/**
 * Multi-mode Autopilot
 */
class BlockMultiModeBacksideAutopilot : public BlockUorbEnabledAutopilot
{
private:
    BlockStabilization _stabilization;
    BlockBacksideAutopilot _backsideAutopilot;
    BlockWaypointGuidance _guide;
    BlockOutputs _outputs;
    BlockParam<float> _spdCmd;
    uint8_t _loopCount;
    struct pollfd _attPoll;
    vehicle_global_position_setpoint_s _lastPosCmd;
public:
    BlockMultiModeBacksideAutopilot(SuperBlock * parent, const char * name) :
        BlockUorbEnabledAutopilot(parent, name),
        _stabilization(this,""), // no name needed, already unique
        _backsideAutopilot(this,""),
        _guide(this,""),
        _outputs(this,"OUT"),
        _spdCmd(this,"SPDCMD"),
        _loopCount(0),
		_attPoll(),
        _lastPosCmd()
    {
        setDt(1.0f /50.0f);
		_attPoll.fd = _att.getHandle();
        _attPoll.events = POLLIN;
    }
    void update()
    {
        /* wait for a sensor update, check for exit condition every 500 ms */
        poll(&_attPoll, 1, 500);

        // store old position command before update if new command sent
        if(_posCmd.updated())
        {
            _lastPosCmd = _posCmd.getData();
        }
        
        // get new information from subscriptions
        updateSubscriptions();

        // handle autopilot modes
        if (_status.state_machine == SYSTEM_STATE_STABILIZED)
        {
            _stabilization.update(
                    _ratesCmd.roll, _ratesCmd.pitch, _ratesCmd.yaw,
                    _att.rollspeed, _att.pitchspeed, _att.yawspeed);
            _outputs.update(_stabilization.getAileron(),
                    _stabilization.getElevator(),
                    _stabilization.getRudder(),
                    _manual.throttle);
        }
        else if (_status.state_machine == SYSTEM_STATE_AUTO)
        {
            // update guidance
            _guide.update(_pos, _att, _posCmd, _lastPosCmd);

            // calculate velocity, XXX should be airspeed, but using ground speed for now
            float v = sqrtf(_pos.vx * _pos.vx + _pos.vy * _pos.vy + _pos.vz * _pos.vz);

            // commands
            float rCmd = 0;

            _backsideAutopilot.update(
                _posCmd.altitude, _spdCmd.get(), rCmd, _guide.getPsiCmd(),
                _pos.alt, v,
                _att.roll, _att.pitch, _att.yaw,
                _att.rollspeed, _att.pitchspeed, _att.yawspeed
                );
            _outputs.update(_backsideAutopilot.getAileron(),
                    _backsideAutopilot.getElevator(),
                    _backsideAutopilot.getRudder(),
                    _backsideAutopilot.getThrottle());
        }
        else if (_status.state_machine == SYSTEM_STATE_MANUAL)
        {
            _outputs.update(
                    _manual.roll,
                    _manual.pitch,
                    _manual.yaw,
                    _manual.throttle
                    );
        }

        // update all publications
        _outputs.setActuators(_actuators);
        updatePublications();

        // update parameters every 100 cycles
        if (_loopCount-- <= 0)
        {
            _loopCount = 100;
            _stabilization.updateParams();
            _backsideAutopilot.updateParams();
            _outputs.updateParams();
            //printf("t: %8.4f, u: %8.4f\n", (double)t, (double)u);
            printf("control mode: %d\n", _status.state_machine);
            printf("aileron: %8.4f, elevator: %8.4f, "
                    "rudder: %8.4f, throttle: %8.4f\n",
                    (double)_outputs.getAileron(),
                    (double)_outputs.getElevator(),
                    (double)_outputs.getRudder(),
                    (double)_outputs.getThrottle());
            fflush(stdout);
        }

        // sleep for approximately the right amount of time for update, 
        // neglects lag from calculations
        usleep(1000000*getDt());
    }
};


} // namespace fixedwing

} // namespace control

