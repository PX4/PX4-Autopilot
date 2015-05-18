/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
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
 * @file uavcan_configuration.hpp
 *
 * UAVCAN Remote Node Configurator
 *
 * @author Marco Bauer <marco@wtns.de>
 */

#pragma once

#include <fcntl.h>
 
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/EnumerationRequest.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/SaveErase.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <mavlink/mavlink_log.h>

#include <vector>

#include <drivers/drv_tone_alarm.h>


/*
 * Commands (Mavlink CMD Long Param #1 - #7) Mavlink Command ID: 600
 * ---------------------------------------------------------------------
 * UAVCAN_CMD_PARAM_SAVE:
 * ------------------------------------
 * #1: UAVCAN_CMD_PARAM_SAVE
 * #2: <remote NodeID>
 * #3: 0
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_PARAM_GET_ALL:
 * ------------------------------------
 * #1: UAVCAN_CMD_PARAM_GET_ALL
 * #2: <remote NodeID>
 * #3: 0
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_PARAM_GET:
 * ------------------------------------
 * #1: UAVCAN_CMD_PARAM_GET
 * #2: <remote node id>
 * #3: <parameter index>
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_PARAM_SET:
 * ------------------------------------
 * #1: UAVCAN_CMD_PARAM_SET
 * #2: <remote node id>
 * #3: <parameter index>
 * #4: <param value>
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_DISCOVER_FOR_NODES:
 * ------------------------------------
 * #1: UAVCAN_CMD_DISCOVER_FOR_NODES
 * #2: <remote node id>
 * #3: 0
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 *
 * UAVCAN_CMD_NODE_GET_INFO:
 * ------------------------------------
 * #1: UAVCAN_CMD_NODE_GET_INFO
 * #2: <remote node id>
 * #3: 0
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 *
 * UAVCAN_CMD_NODE_FACTORY_RESET:
 * ------------------------------------
 * #1: UAVCAN_CMD_NODE_FACTORY_RESET
 * #2: <remote node id>
 * #3: 0
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_NODE_FACTORY_RESET_ALL_NODES:
 * ------------------------------------
 * #1: UAVCAN_CMD_NODE_FACTORY_RESET_ALL_NODES
 * #2: 0
 * #3: 0
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_NODE_RESTART:
 * ------------------------------------
 * #1: UAVCAN_CMD_NODE_RESTART
 * #2: <remote node id>
 * #3: 0
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_ENUMERATE:
 * ------------------------------------
 * #1: UAVCAN_CMD_ENUMERATE
 * #2: <enum node id>
 * #3: <timeout sec>
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_DISCOVER_FOR_SPECIFIC_NODE:
 * ------------------------------------
 * #1: UAVCAN_CMD_DISCOVER_FOR_SPECIFIC_NODE
 * #2: <enum node id>
 * #3: <timeout millisec>
 * #4: 0
 * #5: 0
 * #6: 0
 * #7: 0
 * 
 * UAVCAN_CMD_CFG_ESC:
 * ------------------------------------
 * #1: UAVCAN_CMD_CFG_ESC
 * #2: <enum node id>
 * #3: <motorindex>
 * #4: <timeout sec>
 * #5: 0
 * #6: 0
 * #7: 0
 */



enum UAVCAN_ESC_CMD {
	UAVCAN_CMD_DUMMY = 0,						/* Dummy command to prevent unwanted effects */
	UAVCAN_CMD_PARAM_SAVE, 						/* store changed parameters into node flash memory */
	UAVCAN_CMD_PARAM_GET_ALL, 					/* get all configuration parameter names, types and values from remote node */
	UAVCAN_CMD_PARAM_GET, 						/* get parameter by index */
	UAVCAN_CMD_PARAM_SET, 						/* set parameter by index */
	UAVCAN_CMD_DISCOVER_FOR_NODES,				/* discover canbus for uavcan nodes */
	UAVCAN_CMD_NODE_GET_INFO,					/* retrieve nodeinfos */
	UAVCAN_CMD_NODE_FACTORY_RESET,				/* revert all configuration parameters to their factory values */
	UAVCAN_CMD_NODE_FACTORY_RESET_ALL_NODES,	/* revert all configuration parameters to their factory values on all nodes */
	UAVCAN_CMD_NODE_RESTART,					/* restart remote node */
	UAVCAN_CMD_ENUMERATE,						/* start enumeration */
	UAVCAN_CMD_DISCOVER_FOR_SPECIFIC_NODE,		/* discover canbus for a specific uavcan node */
	UAVCAN_CMD_CFG_ESC							/* configure esc uavcan node */
};


class UavcanRemoteConfiguration
{
public:
	UavcanRemoteConfiguration(uavcan::INode& node);
	~UavcanRemoteConfiguration();

	int init();
	void uavcan_configuration(float val_1, float val_2, float val_3, float val_4, float val_5, float val_6, float val_7);


private:
	int		_mavlink_fd;			/**< the file descriptor to send messages over mavlink */

	void cmd_param_save(int targetnode, int factoryreset);
	void cmd_param_get_all(int targetnode);
	void cmd_param_get(int targetnode, int paramindex);
	void cmd_set_param_set(int targetnode, int paramindex, float value);
	void cmd_enumerate(int targetnode, int timeout);
	void cmd_node_restart(int targetnode);
	int  cmd_discover_for_nodes(int targetnode, int timeout, int factoryreset);
	void cmd_node_get_info(int targetnode);
	int  cmd_esc_cfg(int targetnode, int motorindex, int timeout);

	void play_px4_tone(int toneid);
	
	/*
	 * libuavcan related things
	 */
	uavcan::INode							&_node;
	uavcan::Publisher<uavcan::protocol::EnumerationRequest>		_uavcan_pub_enumeration_cmd;

};
