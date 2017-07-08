/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_command_sender.cpp
 * Mavlink commands sender with support for retransmission.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include "mavlink_command_sender.h"
#include <px4_log.h>
#include <cassert>

#define CMD_DEBUG(FMT, ...) PX4_LOG_NAMED_COND("cmd sender", _debug_enabled, FMT, ##__VA_ARGS__)

MavlinkCommandSender *MavlinkCommandSender::_instance = nullptr;
px4_sem_t MavlinkCommandSender::_lock;

void MavlinkCommandSender::initialize()
{
	px4_sem_init(&_lock, 1, 1);

	if (_instance == nullptr) {
		_instance = new MavlinkCommandSender();
	}
}

MavlinkCommandSender &MavlinkCommandSender::instance()
{

	return *_instance;
}

MavlinkCommandSender::MavlinkCommandSender() :
	_commands(3)
{
}

MavlinkCommandSender::~MavlinkCommandSender()
{
	px4_sem_destroy(&_lock);
}

int MavlinkCommandSender::handle_vehicle_command(const struct vehicle_command_s &command, mavlink_channel_t channel)
{
	lock();
	CMD_DEBUG("getting vehicle command with timestamp %" PRIu64 ", channel: %d", command.timestamp, channel);

	mavlink_command_long_t msg = {};
	msg.target_system = command.target_system;
	msg.target_component = command.target_component;
	msg.command = command.command;
	msg.confirmation = command.confirmation;
	msg.param1 = command.param1;
	msg.param2 = command.param2;
	msg.param3 = command.param3;
	msg.param4 = command.param4;
	msg.param5 = command.param5;
	msg.param6 = command.param6;
	msg.param7 = command.param7;
	mavlink_msg_command_long_send_struct(channel, &msg);

	bool already_existing = false;
	_commands.reset_to_start();

	while (command_item_t *item = _commands.get_next()) {
		if (item->timestamp_us == command.timestamp) {

			// We should activate the channel by setting num_sent_per_channel from -1 to 0.
			item->num_sent_per_channel[channel] = 0;

			CMD_DEBUG("already existing");
			already_existing = true;

			break;
		}
	}

	if (!already_existing) {

		command_item_t new_item;
		new_item.command = msg;
		new_item.timestamp_us = command.timestamp;
		new_item.num_sent_per_channel[channel] = 0;
		new_item.last_time_sent_us = hrt_absolute_time();
		_commands.put(new_item);
	}

	unlock();
	return 0;
}

void MavlinkCommandSender::handle_mavlink_command_ack(const mavlink_command_ack_t &ack,
		uint8_t from_sysid, uint8_t from_compid)
{
	CMD_DEBUG("handling result %d for command %d: %d from %d",
		 static_cast<unsigned int>(ack.result), static_cast<unsigned int>(ack.command),
		 static_cast<unsigned int>(from_sysid), static_cast<unsigned int>(from_compid));
	lock();

	_commands.reset_to_start();

	while (command_item_t *item = _commands.get_next()) {
		// Check if the incoming ack matches any of the commands that we have sent.
		if (item->command.command == ack.command &&
		    from_sysid == item->command.target_system &&
		    from_compid == item->command.target_component) {
			// Drop it anyway because the command seems to have arrived at the destination, even if we
			// receive IN_PROGRESS because we trust that it will be handled after that.
			_commands.drop_current();
			break;
		}
	}

	unlock();
}

void MavlinkCommandSender::check_timeout(mavlink_channel_t channel)
{
	lock();

	_commands.reset_to_start();

	while (command_item_t *item = _commands.get_next()) {
		if (hrt_elapsed_time(&item->last_time_sent_us) <= TIMEOUT_US) {
			// We keep waiting for the timeout.
			continue;
		}

		// The goal of this is to retry from all channels. Therefore, we keep
		// track of the retry count for each channel.
		//
		// When the first channel does a retry, the timeout is reset.
		// (e.g. all channel have done 2 retries, then channel 0 is called
		// and does retry number 3, and also resets the timeout timestamp).

		// First, we need to determine what the current max and min retry level
		// are because we can only level up, if all have caught up.
		// If num_sent_per_channel is at -1, the channel is inactive.
		int8_t max_sent = 0;
		int8_t min_sent = INT8_MAX;

		for (unsigned i = 0; i < MAX_MAVLINK_CHANNEL; ++i) {
			if (item->num_sent_per_channel[i] > max_sent) {
				max_sent = item->num_sent_per_channel[i];
			}

			if ((item->num_sent_per_channel[i] != -1) &&
			    (item->num_sent_per_channel[i] < min_sent)) {
				min_sent = item->num_sent_per_channel[i];
			}
		}

		if (item->num_sent_per_channel[channel] < max_sent) {
			// We are behind and need to do a retransmission.
			mavlink_msg_command_long_send_struct(channel, &item->command);
			item->num_sent_per_channel[channel]++;

			CMD_DEBUG("%x timeout (behind), retries: %d/%d, channel: %d",
				 static_cast<unsigned int>(item), item->num_sent_per_channel[channel],
				 max_sent, channel);

		} else if (item->num_sent_per_channel[channel] == max_sent &&
			   min_sent == max_sent) {

			// If the next retry would be above the needed retries anyway, we can
			// drop the item, and continue with other items.
			if (item->num_sent_per_channel[channel] + 1 > RETRIES) {
				_commands.drop_current();
				CMD_DEBUG("%x, timeout dropped", static_cast<unsigned int>(item));
				continue;
			}

			// We are the first of a new retransmission series.
			mavlink_msg_command_long_send_struct(channel, &item->command);
			item->num_sent_per_channel[channel]++;
			// Therefore, we are the ones setting the timestamp of this retry round.
			item->last_time_sent_us = hrt_absolute_time();

			CMD_DEBUG("%x timeout (first), retries: %d/%d, channel: %d",
				 static_cast<unsigned int>(item), item->num_sent_per_channel[channel],
				 max_sent, channel);

		} else {
			// We are already ahead, so this should not happen.
			// If it ever does, just ignore it. It will timeout eventually.
			continue;
		}
	}

	unlock();
}
