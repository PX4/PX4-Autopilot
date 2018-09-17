/****************************************************************************
 *
 *   Copyright (C) 2018 AirMind Development Team. All rights reserved.
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
 * 3. Neither the name AirMind nor the names of its contributors may be
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
 * @file pwmgroups.h
 *
 * @Author: ning.roland@mindpx.net (or ning.roland@gmail.com)
 *
 * Base class for System-wide pwm device resource allocation coordination and management.
 *
 * All pwm devices should subclass this base class so that the access to system pwm resources is
 * coordinated through a common set of methods.
 *
 * PwmGroups base class
 * ===========
 *
 * PwmGroups class maintains a static, global system map of timers and channels, which keeps tracking the
 * allocation of these resources to each pwm devices.
 *
 * Each derived pwm device class has its own required map of timers and channels, in global mapping format.
 * This map is statically defined in board configuration file, and imported into pwm device class in run time.
 *
 * Local timer/channel map is used in human-machine interface (ioctl). This base class needs to
 * convert local timer/channel map to global map.
 *
 * pwm device class
 * =============
 *
 * The global allocation map is defined statically in board configuration. Local timer/channel map is
 * used in human-machine interface (ioctl). This base class
 * convert local timer/channel map to global map based on global allocation map.
 
 * While this map is unique for each
 * pwm device, it may have several different working channel map for different working mode. During
 * initialization, a pwm device use register method to request timer/channels from system map that are needed
 * in specific working mode. The base class check the availability of each timer/channels and grant the requests
 * accordingly.
 *
 *
 * static pwm device and floating pwm device
 * -------
 *
 * A static pwm device is a pwm device with its requested tiemr/channel resources statically defined in board configuration.
 * A floating pwm device is a pwm device with tis requested timer/channel resources dynamically allocated in run time.
 * For example, fmu pwm output is a static pwm device, as its output channels are pre-defined in board configuration and can not
 * be channged. A pwm heater is a floating device, as it may be connected to any free pwm output channels. An external pwm led is
 * also a floating pwm device. The channel resources of a floating pwm device is defined from man-machine interface.
 * A static pwm deivce can have differen working modes, and it is the channel map for a specific working mode matters in
 * resource allocation.
 * A static pwm device may share its channels with a floating pwm device if conditions are met.
 *
 *
 * global channel map and local channel map
 * ------
 * Pwm devices use global channel map to represent channel allocations internally. When interacting with user/pilot, local channel
 * map is used as they do not have knowledge of global channel definition. So the input to pwm devices through man-machine interface
 * is in local channel map format.
 * The pwm device class converts the input local channel map into global format, with the help of channel map offset defined
 * in board configuration.
 * For floating pwm devices, like a pwm heater, or external pwm LEDs, pilots can configure its pwm channel map from GCS - reboot
 * will be required to take affects of course. Pilot is responsible for the conflictness of resource allocation.
 * The runtime code of base class is responsible for resource confliction detection and reject any bad resource request.
 *
 * Configuration
 * --------
 *
 * All resources used by static pwm devices need to be defined in board configuration. When vehicle is boot, the system initialize
 * pwm timers and channels according to these definition first.
 * The resources used by floating pwm devices are defined through airframes, or by user manually through UI of GCS.
 * The system initializes these pwm devices after static pwm devices are initialized.
 *
 *
 * Base class
 * pwm device sub class
 * static device, floating device
 * global channel map
 * group offset map, working channel map
 * local device channel map;
 *
 */


#ifndef _SYSTEMLIB_PWM_GROUPS_H
#define _SYSTEMLIB_PWM_GROUPS_H value

#include <px4_config.h>
#include "../drivers/device/CDev.hpp"

typedef enum {
    PWM_GROUP_RATE_ALT,
    PWM_GROUP_RATE_DEFAULT,
    PWM_GROUP_RATE_MIXED,
    PWM_GROUP_RATE_UNSPECIFIED
} pwm_groups_rate_t;

#define PWM_OUTPUT_RATE_ALT 400
#define PWM_OUTPUT_RATE_DEFAULT 50

/**
 * Abstract class defining a mixer mixing zero or more inputs to
 * one or more outputs.
 */
class __EXPORT PwmGroups : public device::CDev
{
public:
	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked when reading controls.
	 */
    PwmGroups(const char *name, const char *devname);
    ~PwmGroups() {};

	/**
	 * Return the timer map under current working mode.
	 *
	 *
	 * @return			timer map of current working mode.
	 */
    uint8_t working_timer_map (void);

	/**
	 * Return the channel map under current working mode.
	 *
	 * @return			channel map of current working mode.
	 */
    uint32_t working_channel_map (void);

    /**
     * Calculate the offset of local channel map given a global channel map.
     *
     * @param[in]  channel_map  global channel map.
     * 
     * @return            the offset value.
     */
    uint8_t group_channel_map_offset(uint32_t channel_map);

    /**
     * Calculate the offset of local timer map given a global timer map.
     *
     * @param[in]  timer_map  global timer map.
     *
     * @return            the offset value.
     */
    uint8_t group_timer_map_offset(uint8_t timer_map);

    int set_pwm_rate_map (uint32_t device_rate_map);

    /**
	 * Set rate of a group of pwm channels
	 * that are required.
	 *
	 * @param groups		A bitmask of groups (0-31) to set rate.
     * @param rate          target rate.
     * @return              0 on success.
	 */
    int set_pwm_channel_rates (uint8_t alt_rate, uint8_t default_rate);

	/**
	 * @brief      Set a specific channel's pwm value.
	 *
	 * @param[in]  device_channel_idx  index of the channel to set, in local format.
	 * @param[in]  value               target value.
     * @return     0 on success.
     *
	 */
    int set_pwm_channel_value_single (uint8_t device_channel_idx, uint8_t value);

	/**
	 * @brief Set current working mode, hence the working channel map.
	 *
	 * @return 0 on success.
	 */
	//uint8_t set_device_pwm_mode(mode_map);

	/**
	 * @brief Reqeust to register all working channels in global channel map.
	 *
	 * @return 0 on success.
	 */
    int register_working_channels();


protected:
    //so these are visible in derived class.
    //statically allocated, share-able, alt-rate.
    static uint8_t all_timers;
    static uint32_t all_channels;
    static uint32_t _default_rate_channels;
    static uint32_t _alt_rate_channels;
    static uint8_t _alt_rate_timers;
    static uint8_t _default_rate_timers;

    uint8_t _group_timer_map;
    //group channels must be continously allocated;
    uint32_t _group_channel_map;
    uint32_t _working_channel_map;
    //map of channels with alt rate, different from _working_channel_map in some cases,
    //e.g., a mixer group includes both main output and camera trigger;
    uint32_t _channel_rate_map;
    
    //channel map for capture function;
    uint32_t _capture_channel_map;
    
    uint8_t _timer_map_offset;
    uint32_t _channel_map_offset;
    
    uint16_t _default_rate;
    uint16_t _alt_rate;
    uint16_t _working_pwm_rate_up_limit;
    uint16_t _working_pwm_rate_low_limit;
    
    pwm_groups_rate_t _working_rate;
    
private:
    bool is_free_channels_available(uint32_t channel_map, pwm_groups_rate_t rate_type);
    void update_default_alt_rate_map (pwm_groups_rate_t rate_type, uint32_t channel_map);
    uint32_t get_channel_mask_of_rate_type (pwm_groups_rate_t rate_type);
};

/**
 * Derived class from PwmGroups for static allocated pwm channel
 * devices.
 */
class __EXPORT StaticPwmDevice : public PwmGroups
{
public:

    StaticPwmDevice(const char *name, const char *devname);
    ~StaticPwmDevice() {};

	/**
	 * Get channel map of group.
	 *
	 * @return			group channel map.
	 */
	uint32_t				get_group_channel_map();

	/**
	 * Remove all the mixers from the group.
	 */

};

/**
 * Derived class from PwmGroups for pwm
 * devices can hook to arbitary compatible channels.
 */
class __EXPORT FloatingPwmDevice : public PwmGroups
{
public:
	FloatingPwmDevice(const char *name, const char *devname);
    ~FloatingPwmDevice() {};

    uint8_t set_group_channel_map_offset (uint8_t offset);

};



#endif
