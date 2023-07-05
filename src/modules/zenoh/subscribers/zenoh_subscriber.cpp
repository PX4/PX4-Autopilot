/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file zenoh_subscriber.cpp
 *
 * Zenoh subscriber
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#include "zenoh_subscriber.hpp"

static void data_handler_cb(const z_sample_t *sample, void *arg)
{
	static_cast<Zenoh_Subscriber *>(arg)->data_handler(sample);
}

void Zenoh_Subscriber::data_handler(const z_sample_t *sample)
{
	z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
	printf(">> [Subscriber] Received ('%s' size '%d')\n", z_str_loan(&keystr), (int)sample->payload.len);
	z_str_drop(z_str_move(&keystr));
}


Zenoh_Subscriber::Zenoh_Subscriber(bool rostopic)
{
	this->_rostopic = rostopic;
	this->_topic[0] = 0x0;
}

Zenoh_Subscriber::~Zenoh_Subscriber()
{
	undeclare_subscriber();
}

int Zenoh_Subscriber::undeclare_subscriber()
{
	z_undeclare_subscriber(z_subscriber_move(&_sub));
	return 0;
}

int Zenoh_Subscriber::declare_subscriber(z_session_t s, const char *keyexpr)
{
	z_owned_closure_sample_t callback = z_closure_sample(data_handler_cb, NULL, this);

	if (_rostopic) {
		strncpy(this->_topic, (char *)_rt_prefix, _rt_prefix_offset);

		if (keyexpr[0] == '/') {
			strncpy(this->_topic + _rt_prefix_offset, keyexpr + 1, sizeof(this->_topic) - _rt_prefix_offset);

		} else {
			strncpy(this->_topic + _rt_prefix_offset, keyexpr, sizeof(this->_topic) - _rt_prefix_offset);
		}

	} else {
		strncpy(this->_topic, (char *)keyexpr, sizeof(this->_topic));
	}

	_sub = z_declare_subscriber(s, z_keyexpr(this->_topic), z_closure_sample_move(&callback), NULL);


	if (!z_subscriber_check(&_sub)) {
		printf("Unable to declare subscriber for key expression!\n %s\n", keyexpr);
		return -1;
	}

	return 0;
}

void Zenoh_Subscriber::print()
{
	printf("Topic: %s\n", this->_topic);
}

void Zenoh_Subscriber::print(const char *type_string, const char *topic_string)
{
	printf("Topic: %s -> %s %s \n", this->_topic, type_string, topic_string);
}
