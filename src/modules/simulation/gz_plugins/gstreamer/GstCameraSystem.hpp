/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_set>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

namespace custom
{

class CameraStream
{
public:
	CameraStream(const std::string &udpHost, int udpPort, bool useCuda,
		     bool useRtmp, const std::string &rtmpLocation,
		     const std::string &cameraTopic);
	void start();

	void stop();

	int getUdpPort() const
	{
		return _udpPort;
	}

	~CameraStream();
private:
	std::string _cameraTopic;

	// Transport
	gz::transport::Node _node;

	// Image processing
	gz::msgs::Image _currentFrame;
	std::mutex _frameMutex;
	std::atomic<bool> _newFrameAvailable{};

	// GStreamer elements
	GMainLoop *_gstLoop{};
	GstElement *_pipeline{};
	GstElement *_source{};
	std::thread _gstThread;
	std::atomic<bool> _running{};

	// stream params
	int _width{0};
	int _height{0};
	double _rate{30.0};
	std::string _udpHost;
	int _udpPort = 5600;
	bool _useRtmp{};
	std::string _rtmpLocation;
	bool _useCuda = true;

	void onCameraInfo(const gz::msgs::Image &msg);
	void onImage(const gz::msgs::Image &msg);

	void gstThreadFunc();
};


class GstCameraSystem :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPostUpdate
{
public:
	GstCameraSystem();

	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void PostUpdate(const gz::sim::UpdateInfo &_info,
			const gz::sim::EntityComponentManager &_ecm) override;


private:
// Video streams
	std::unordered_map<gz::sim::Entity, std::unique_ptr<CameraStream>> _streams;



	std::string _buildTopic(const gz::sim::EntityComponentManager &_ecm,
				const gz::sim::Entity &sensorEntity,
				const gz::sim::components::Name *sensorName,
				const gz::sim::components::ParentEntity *parent);
	// Configuration
	std::string _worldName;
	std::string _udpHost;
	int _baseUdpPort = 5600;
	bool _useRtmp {};
	std::string _rtmpLocation;
	bool _useCuda = true;

	// methods below are used to maintain consistency in udp ports when restarting instances. For example, if we have
	// instance 1 streaming on port 5601 we also want it to stream to the same port if we shutdown and restart the
	// instance
	std::unordered_set<int> _usedUdpPorts;

	int getAvailableUdpPort()
	{
		int port = _baseUdpPort;

		while (_usedUdpPorts.count(port) > 0) {
			port++;
		}

		_usedUdpPorts.insert(port);
		return port;
	}

	void freeUdpPort(int port)
	{
		auto it_port = _usedUdpPorts.find(port);

		if (it_port != _usedUdpPorts.end()) {
			_usedUdpPorts.erase(it_port);
		}
	}


};
}  // namespace custom
