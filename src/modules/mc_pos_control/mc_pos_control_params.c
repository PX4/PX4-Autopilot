/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_params.c
 * Multicopter position controller parameters.
 * 多旋翼位置控制参数
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Minimum thrust in auto thrust control
 * 自动推力控制下的最小推力（推力一般理解为油门的变化量）
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 * 推荐将此值设置为一个大于0的值以避免在没有推力的时候自由落体
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MIN, 0.12f);

/**
 * Hover thrust
 * 悬停推力
 *
 * Vertical thrust required to hover.
 * This value is mapped to center stick for manual throttle control.
 * With this value set to the thrust required to hover, transition
 * from manual to ALTCTL mode while hovering will occur with the
 * throttle stick near center, which is then interpreted as (near)
 * zero demand for vertical speed.
 * 对应手动模式油门中值，用来表示悬停需要的推力，
 * 手动模式转定高模式可触发悬停。
 *
 * @unit norm
 * @min 0.2
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 0.5f);

/**
 * ALTCTL throttle curve breakpoint
 * 定高模式的油门曲线断点
 *
 * Halfwidth of deadband or reduced sensitivity center portion of curve.
 * This is the halfwidth of the center region of the ALTCTL throttle
 * curve. It extends from center-dz to center+dz.
 * 定高油门曲线的中心区域的半宽量，
 * 即中心区域=（中点-断点）~（中点+断点），
 * 该区域内保持定高，垂直方向不增减推力
 *
 * @unit norm
 * @min 0.0
 * @max 0.2
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ALTCTL_DZ, 0.1f);

/**
 * ALTCTL throttle curve breakpoint height
 * 定高模式油门曲线断点的高度
 *
 * Controls the slope of the reduced sensitivity region.
 * This is the height of the ALTCTL throttle
 * curve at center-dz and center+dz.
 * 控制降低灵敏度区域的斜率。这是定高模式下油门在中心上下dz范围内的高度曲线
 *
 * @min 0.0
 * @max 0.2
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ALTCTL_DY, 0.0f);

/**
 * Maximum thrust in auto thrust control
 * 自动推力控制的最大推力
 *
 * Limit max allowed thrust. Setting a value of one can put
 * the system into actuator saturation as no spread between
 * the motors is possible any more. A value of 0.8 - 0.9
 * is recommended.
 * 限制可允许的最大推力。将此值设置为1可能使系统驱动饱和，由于电机之间不可能存在spread
 * 建议将此值设置为0.8~0.9
 *
 * @unit norm
 * @min 0.0
 * @max 0.95
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MAX, 0.9f);

/**
 * Minimum manual thrust
 * 最小的手动推力
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 * 最小的垂直推力。建议将此值设置成一个大于0的值以避免在推力为0时自由落体
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MANTHR_MIN, 0.08f);

/**
 * Maximum manual thrust
 * 最大的手动推力
 *
 * Limit max allowed thrust. Setting a value of one can put
 * the system into actuator saturation as no spread between
 * the motors is possible any more. A value of 0.8 - 0.9
 * is recommended.
 * 限制允许的最大推力。将此值设置为1可能使系统驱动饱和，由于电机之间不可能存在spread
 * 建议将此值设置为0.8~0.9
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MANTHR_MAX, 0.9f);

/**
 * Proportional gain for vertical position error
 * 垂直位置误差的比例增益 
 *
 * @min 0.0
 * @max 1.5
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_P, 1.0f);

/**
 * Proportional gain for vertical velocity error
 * 垂直速度误差的比例增益
 * @min 0.1
 * @max 0.4
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_P, 0.2f);

/**
 * Integral gain for vertical velocity error
 * 垂直速度误差的积分增益
 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
 * 此值非零时允许飞行器在自稳或者自动起飞模式下的悬停推力估计
 *
 * @min 0.01
 * @max 0.1
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_I, 0.02f);

/**
 * Differential gain for vertical velocity error
 * 垂直速度误差的微分增益
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_D, 0.0f);

/**
 * Maximum vertical ascent velocity
 * 最大垂直上升速度
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 * 自动模式下的最大垂直速度，自稳模式下的终值（定高、定点）
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP, 3.0f);

/**
 * Maximum vertical descent velocity
 * 最大垂直下降速度
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 * 自动模式下的最大垂直速度，自稳模式下的终值（定高、定点）
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX, 1.0f);

/**
 * Transitional support, do not change / use
 * 转换支持，不要改变/使用
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN, 1.0f);

/**
 * Vertical velocity feed forward
 * 垂直速度前馈
 * Feed forward weight for altitude control in stabilized modes (ALTCTRL, POSCTRL). 
 * 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 * 自稳模式下对高度进行控制的前馈权重。
 * 此值为0时会使得响应慢无超调，此值为1时响应快超调大
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_FF, 0.5f);

/**
 * Proportional gain for horizontal position error
 * 水平位置误差的比例增益
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_P, 0.95f);

/**
 * Proportional gain for horizontal velocity error
 * 水平速度误差的比例增益
 * @min 0.06
 * @max 0.15
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P, 0.09f);

/**
 * Integral gain for horizontal velocity error
 * 水平速度误差的积分增益
 * Non-zero value allows to resist wind.
 * 此值非0时可以起到抗风的作用
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_I, 0.02f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. 
 * If value is too big oscillations will appear again.
 * 水平速度误差的微分增益。此值较小时可帮助降低快速的震荡，如果值太大，震荡会再次发生
 *
 * @min 0.005
 * @max 0.1
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_D, 0.01f);

/**
 * Nominal horizontal velocity
 *
 * Normal horizontal velocity in AUTO modes (includes
 * also RTL / hold / etc.) and endpoint for
 * position stabilized mode (POSCTRL).
 * 自动模式下的一般水平速度（包括RTL/HOLD等等），定点模式下的的终值
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_CRUISE, 5.0f);

/**
 * Maximum horizontal velocity
 * 最大水平速度
 * Maximum horizontal velocity in AUTO mode. If higher speeds
 * are commanded in a mission they will be capped to this velocity.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 8.0f);

/**
 * Horizontal velocity feed forward
 * 水平速度前馈
 * Feed forward weight for position control in position control mode (POSCTRL). 
 * 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_FF, 0.5f);

/**
 * Maximum tilt angle in air
 * 飞行时的最大倾斜角
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 * 限制飞行时在自动模式以及定点模式下的最大倾斜角
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR, 45.0f);

/**
 * Maximum tilt during landing
 * 着陆时的最大倾斜角
 * Limits maximum tilt angle on landing.
 * 
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_LND, 12.0f);

/**
 * Landing descend rate
 * 降落时的下降速度
 * @unit m/s
 * @min 0.2
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_SPEED, 0.5f);

/**
 * Takeoff climb rate
 * 起飞时的上升速度
 * @unit m/s
 * @min 1
 * @max 5
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TKO_SPEED, 1.5f);

/**
 * Max manual roll
 * 手动模式下的最大横滚 
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_R_MAX, 35.0f);

/**
 * Max manual pitch
 * 手动模式下的最大俯仰
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_P_MAX, 35.0f);

/**
 * Max manual yaw rate
 * 手动模式下的最大偏航
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_MAX, 200.0f);

/**
 * Deadzone of X,Y sticks where position hold is enabled
 * 使能定点模式时X,Y遥杆的死区 
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_XY_DZ, 0.1f);

/**
 * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
 * 使能位置保持(定点)模式时的最大水平速度（将此值设置为0以禁用检测）
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_XY, 0.8f);

/**
 * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
 * 使能位置保持(定点)模式时的最大垂直速度（将此值设置为0以禁用检测）
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_Z, 0.6f);

/**
 * Low pass filter cut freq. for numerical velocity derivative
 * 低通滤波器的截止频率，用于速度微分
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VELD_LP, 5.0f);

/**
 * Maximum horizonal acceleration in velocity controlled modes
 * 在速度控制模式下的最大水平加速度
 *
 * @unit m/s/s
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX, 5.0f);

/**
 * Altitude control mode, note mode 1 only tested with LPE
 * 定高模式，请注意，模式1仅用于LPE测试 
 *
 * @min 0
 * @max 1
 * @value 0 Altitude following
 * @value 1 Terrain following
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_ALT_MODE, 0);
