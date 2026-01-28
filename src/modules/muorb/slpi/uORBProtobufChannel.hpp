/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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
#ifndef _uORBProtobufChannel_hpp_
#define _uORBProtobufChannel_hpp_

#include <stdint.h>
#include <stdio.h>
#include <string>
#include <map>
#include <pthread.h>
#include <termios.h>

#include "uORB/uORBCommunicator.hpp"
#include "mUORBAggregator.hpp"

namespace uORB
{
class ProtobufChannel;
}

class uORB::ProtobufChannel : public uORBCommunicator::IChannel
{
public:
	/**
	 * static method to get the IChannel Implementor.
	 */
	static uORB::ProtobufChannel *GetInstance()
	{
		return &(_Instance);
	}

	/**
	 * @brief Interface to notify the remote entity of a topic being advertised.
	 *
	 * @param messageName
	 * 	This represents the uORB message name(aka topic); This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	int16_t topic_advertised(const char *messageName);

	/**
	 * @brief Interface to notify the remote entity of interest of a
	 * subscription for a message.
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages. This parameter is unused.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	int16_t add_subscription(const char *messageName, int32_t msgRateInHz);

	/**
	 * @brief Interface to notify the remote entity of removal of a subscription
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not necessarily mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	int16_t remove_subscription(const char *messageName);

	/**
	 * Register Message Handler.  This is internal for the IChannel implementer*
	 */
	int16_t register_handler(uORBCommunicator::IChannelRxHandler *handler);

	/**
	 * @brief Sends the data message over the communication link.
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent.
	 * @param data
	 * 	The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	int16_t send_message(const char *messageName, int32_t length, uint8_t *data);

	uORBCommunicator::IChannelRxHandler *GetRxHandler()
	{
		return _RxHandler;
	}

	void RegisterSendHandler(mUORB::Aggregator::sendFuncPtr func)
	{
		_Aggregator.RegisterSendHandler(func);
	}

	int AddRemoteSubscriber(const std::string &messageName)
	{
		int currentRemoteSubscribers;
		pthread_mutex_lock(&_rx_mutex);
		currentRemoteSubscribers = _AppsSubscriberCache[messageName]++;
		pthread_mutex_unlock(&_rx_mutex);
		return currentRemoteSubscribers;
	}

	void RemoveRemoteSubscriber(const std::string &messageName)
	{
		pthread_mutex_lock(&_rx_mutex);

		if (_AppsSubscriberCache[messageName]) {
			_AppsSubscriberCache[messageName]--;
		}

		pthread_mutex_unlock(&_rx_mutex);
	}

	bool DebugEnabled()	{ return _debug; }

	void SendAggregateData()
	{
		pthread_mutex_lock(&_tx_mutex);
		_Aggregator.SendData();
		pthread_mutex_unlock(&_tx_mutex);
	}

private:
	/**
	 * Data Members
	 */
	static uORB::ProtobufChannel                _Instance;
	static uORBCommunicator::IChannelRxHandler *_RxHandler;
	static mUORB::Aggregator					_Aggregator;
	static std::map<std::string, int>           _AppsSubscriberCache;
	static pthread_mutex_t                      _tx_mutex;
	static pthread_mutex_t                      _rx_mutex;
	static bool                                 _debug;

	/**
	 * Class Members
	 */
	ProtobufChannel() {};
};

// TODO: This has to be defined in the slpi_proc build and in the PX4 build.
// Make it accessible from one file to both builds.
typedef struct {
	int (*advertise_func_ptr)(const char *topic_name);
	int (*subscribe_func_ptr)(const char *topic_name);
	int (*unsubscribe_func_ptr)(const char *topic_name);
	int (*topic_data_func_ptr)(const char *name, const uint8_t *data, int data_len_in_bytes);
	// device::SPI::_config_spi_bus_func_t config_spi_bus;
	// device::SPI::_spi_transfer_func_t spi_transfer;
	int (*_config_spi_bus_func_t)();
	int (*_spi_transfer_func_t)(int, const uint8_t *, uint8_t *, const unsigned);
	// device::I2C::_config_i2c_bus_func_t config_i2c_bus;
	// device::I2C::_set_i2c_address_func_t set_i2c_address;
	// device::I2C::_i2c_transfer_func_t i2c_transfer;
	int (*_config_i2c_bus_func_t)(uint8_t, uint8_t, uint32_t);
	int (*_set_i2c_address_func_t)(int, uint8_t);
	int (*_i2c_transfer_func_t)(int, const uint8_t *, const unsigned, uint8_t *, const unsigned);
	// open_uart_func_t open_uart_func;
	// write_uart_func_t write_uart_func;
	// read_uart_func_t read_uart_func;
	int (*open_uart_func_t)(uint8_t, speed_t);
	int (*write_uart_func_t)(int, const void *, size_t);
	int (*read_uart_func_t)(int,  void *, size_t);
	int (*register_interrupt_callback)(int (*)(int, void *, void *), void *arg);
} fc_func_ptrs;

extern "C" {

	int px4muorb_orb_initialize(fc_func_ptrs *func_ptrs, int32_t clock_offset_us) __EXPORT;

	int px4muorb_topic_advertised(const char *name) __EXPORT;

	int px4muorb_add_subscriber(const char *name) __EXPORT;

	int px4muorb_remove_subscriber(const char *name) __EXPORT;

	int px4muorb_send_topic_data(const char *name, const uint8_t *data, int data_len_in_bytes) __EXPORT;

	float px4muorb_get_cpu_load(void) __EXPORT;
}

#endif // _uORBProtobufChannel_hpp_
