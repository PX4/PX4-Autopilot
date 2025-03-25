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

#include "GstCameraSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/transport/TopicUtils.hh>
#include <gz/sim/Util.hh>

using namespace custom;

//////////////////////////////////////////////////
GstCameraSystem::GstCameraSystem()
{
	// Default UDP host
	const char *host_ip = std::getenv("PX4_VIDEO_HOST_IP");

	if (host_ip) {
		_udpHost = std::string(host_ip);

	} else {
		_udpHost = "127.0.0.1";
	}

	// Initialize gstreamer
	static bool gstInitialized = false;

	if (!gstInitialized) {
		gst_init(nullptr, nullptr);
		gstInitialized = true;
	}

	// Setup camera topic regex pattern
	_cameraTopicPattern = std::regex("/world/([^/]+)/model/([^/]+)/link/([^/]+)/sensor/([^/]+)/image");
}

//////////////////////////////////////////////////
GstCameraSystem::~GstCameraSystem()
{
	_running = false;

	if (_gstLoop) {
		g_main_loop_quit(_gstLoop);
	}

	if (_gstThread.joinable()) {
		_gstThread.join();
	}

	if (_pipeline) {
		gst_element_set_state(_pipeline, GST_STATE_NULL);
		gst_object_unref(_pipeline);
	}
}

//////////////////////////////////////////////////
void GstCameraSystem::Configure(const gz::sim::Entity &_entity,
				const std::shared_ptr<const sdf::Element> &_sdf,
				gz::sim::EntityComponentManager &_ecm,
				gz::sim::EventManager &/*_eventMgr*/)
{
	// Check if this is a world entity
	if (!_ecm.EntityHasComponentType(_entity, gz::sim::components::World::typeId)) {
		gzerr << "GstCameraSystem should be attached to a world entity" << std::endl;
		return;
	}

	// Get world name
	auto worldNameComp = _ecm.Component<gz::sim::components::Name>(_entity);

	if (worldNameComp) {
		_worldName = worldNameComp->Data();

	} else {
		gzerr << "Failed to get world name" << std::endl;
		return;
	}

	gzdbg << "GstCameraSystem configured for world [" << _worldName << "]" << std::endl;

	// Parse SDF parameters
	if (_sdf->HasElement("udpHost")) {
		_udpHost = _sdf->Get<std::string>("udpHost");
	}

	if (_sdf->HasElement("udpPort")) {
		_udpPort = _sdf->Get<int>("udpPort");
	}

	if (_sdf->HasElement("rtmpLocation")) {
		_rtmpLocation = _sdf->Get<std::string>("rtmpLocation");
		_useRtmp = true;
	}

	if (_sdf->HasElement("useCuda")) {
		_useCuda = _sdf->Get<bool>("useCuda");
	}

	gzdbg << "GstCameraSystem parameters:" << std::endl
	      << "  UDP Host: " << _udpHost << std::endl
	      << "  UDP Port: " << _udpPort << std::endl
	      << "  Use RTMP: " << (_useRtmp ? "true" : "false") << std::endl
	      << "  RTMP Location: " << _rtmpLocation << std::endl
	      << "  Use CUDA: " << (_useCuda ? "true" : "false") << std::endl;
}

//////////////////////////////////////////////////
void GstCameraSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
				 const gz::sim::EntityComponentManager &/*_ecm*/)
{
	if (_info.paused || _initialized) {
		return;
	}

	// Find a camera topic
	findCameraTopic();
}

//////////////////////////////////////////////////
void GstCameraSystem::findCameraTopic()
{
	// Get all available topics
	std::vector<std::string> topics;
	_node.TopicList(topics);

	for (const auto &topic : topics) {
		std::smatch matches;

		// Check if the topic matches our camera pattern
		if (std::regex_search(topic, matches, _cameraTopicPattern)) {
			std::string worldName = matches[1].str();

			// Only process cameras in our world
			if (worldName == _worldName) {
				_cameraTopic = topic;

				gzdbg << "Found camera topic: " << _cameraTopic << std::endl;

				// Subscribe to first message to get camera info
				_node.Subscribe(_cameraTopic, &GstCameraSystem::onCameraInfo, this);

				_initialized = true;
				return;
			}
		}
	}

	if (!_initialized) {
		gzdbg << "No camera topics found yet, will check again next update" << std::endl;
	}
}

//////////////////////////////////////////////////
void GstCameraSystem::onCameraInfo(const gz::msgs::Image &msg)
{
	_width = msg.width();
	_height = msg.height();

	gzdbg << "Camera info: " << _width << "x" << _height << std::endl;

	// Unsubscribe from the initial subscription
	_node.Unsubscribe(_cameraTopic);

	// Subscribe to actual stream with our callback
	_node.Subscribe(_cameraTopic, &GstCameraSystem::onImage, this);

	// Start GStreamer pipeline
	_gstThread = std::thread(&GstCameraSystem::gstThreadFunc, this);
}

//////////////////////////////////////////////////
void GstCameraSystem::onImage(const gz::msgs::Image &msg)
{
	if (!_running) {
		return;
	}

	// Check pixel format and convert if necessary
	if (msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
		// Process the frame
		std::lock_guard<std::mutex> lock(_frameMutex);
		_currentFrame = msg;
		_newFrameAvailable = true;

	} else {
		gzwarn << "Unsupported pixel format: " << msg.pixel_format_type() << std::endl;
	}
}

//////////////////////////////////////////////////
void GstCameraSystem::gstThreadFunc()
{
	gzdbg << "Starting GStreamer thread" << std::endl;

	_gstLoop = g_main_loop_new(nullptr, FALSE);

	if (!_gstLoop) {
		gzerr << "Failed to create GStreamer main loop" << std::endl;
		return;
	}

	_pipeline = gst_pipeline_new(nullptr);

	if (!_pipeline) {
		gzerr << "Failed to create GStreamer pipeline" << std::endl;
		g_main_loop_unref(_gstLoop);
		_gstLoop = nullptr;
		return;
	}

	// Create elements
	_source = gst_element_factory_make("appsrc", nullptr);
	GstElement *queue1 = gst_element_factory_make("queue", nullptr);
	GstElement *videoRate = gst_element_factory_make("videorate", nullptr);
	GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
	GstElement *queue2 = gst_element_factory_make("queue", nullptr);

	// Configure source and queues for better buffering
	g_object_set(G_OBJECT(queue1),
		     "max-size-buffers", 30,
		     "max-size-time", 0,
		     "max-size-bytes", 0,
		     "leaky", 2, // downstream (newer buffers)
		     NULL);

	g_object_set(G_OBJECT(queue2),
		     "max-size-buffers", 30,
		     "max-size-time", 0,
		     "max-size-bytes", 0,
		     NULL);

	// Configure video rate to reduce tearing
	g_object_set(G_OBJECT(videoRate),
		     "max-rate", 30,
		     "drop-only", TRUE,
		     NULL);

	GstElement *encoder;

	if (_useCuda) {
		encoder = gst_element_factory_make("nvh264enc", nullptr);

		if (encoder) {
			// Higher quality NVIDIA encoder settings
			g_object_set(G_OBJECT(encoder),
				     "bitrate", 4000,       // Increased bitrate for higher quality
				     "preset", 2,           // Higher quality preset (HP)
				     "rc-mode", 1,          // Constant bitrate mode
				     "zerolatency", TRUE,   // Reduce latency
				     "qp-const", 20,        // Lower QP (higher quality)
				     NULL);

		} else {
			gzerr << "NVIDIA H.264 encoder not available, falling back to software encoder" << std::endl;
			encoder = gst_element_factory_make("x264enc", nullptr);
			g_object_set(G_OBJECT(encoder),
				     "bitrate", 4000,
				     "speed-preset", 4,     // Higher quality preset (slower)
				     "tune", 4,             // 'zerolatency' tune option
				     "key-int-max", 30,     // Keyframe every 30 frames (1s at 30 fps)
				     "threads", 4,          // Use multiple threads
				     "pass", 5,             // Quality-based VBR
				     "quantizer", 20,       // Lower = higher quality
				     NULL);
		}

	} else {
		encoder = gst_element_factory_make("x264enc", nullptr);
		g_object_set(G_OBJECT(encoder),
			     "bitrate", 4000,
			     "speed-preset", 4,       // Higher quality preset (slower)
			     "tune", 4,               // 'zerolatency' tune option
			     "key-int-max", 30,       // Keyframe every 30 frames (1s at 30 fps)
			     "threads", 4,            // Use multiple threads
			     "pass", 5,               // Quality-based VBR
			     "quantizer", 20,         // Lower = higher quality
			     NULL);
	}

	GstElement *payloader;
	GstElement *sink;

	if (_useRtmp) {
		payloader = gst_element_factory_make("flvmux", nullptr);
		g_object_set(G_OBJECT(payloader), "streamable", TRUE, NULL);
		sink = gst_element_factory_make("rtmpsink", nullptr);
		g_object_set(G_OBJECT(sink), "location", _rtmpLocation.c_str(), NULL);

	} else {
		payloader = gst_element_factory_make("rtph264pay", nullptr);
		// Improve RTP settings for local streaming
		g_object_set(G_OBJECT(payloader),
			     "config-interval", 1,    // Send SPS/PPS with every I-frame
			     "mtu", 1400,             // Large MTU for local network
			     NULL);

		sink = gst_element_factory_make("udpsink", nullptr);
		g_object_set(G_OBJECT(sink),
			     "host", _udpHost.c_str(),
			     "port", _udpPort,
			     "sync", FALSE,           // Don't sync, reduce latency
			     "async", FALSE,          // Don't async, reduce latency
			     NULL);
	}

	if (!_source || !queue1 || !videoRate || !converter || !queue2 || !encoder || !payloader || !sink) {
		gzerr << "Failed to create one or more GStreamer elements" << std::endl;
		gst_object_unref(_pipeline);
		g_main_loop_unref(_gstLoop);
		_pipeline = nullptr;
		_gstLoop = nullptr;
		return;
	}

	// Configure source
	GstCaps *sourceCaps = gst_caps_new_simple("video/x-raw",
			      "format", G_TYPE_STRING, "RGB",
			      "width", G_TYPE_INT, _width,
			      "height", G_TYPE_INT, _height,
			      "framerate", GST_TYPE_FRACTION, (unsigned int)_rate, 1,
			      NULL);

	g_object_set(G_OBJECT(_source),
		     "caps", sourceCaps,
		     "is-live", TRUE,
		     "do-timestamp", TRUE,
		     "stream-type", GST_APP_STREAM_TYPE_STREAM,
		     "format", GST_FORMAT_TIME,
		     "min-latency", 0,
		     "max-latency", 0,
		     "emit-signals", TRUE,
		     NULL);

	gst_caps_unref(sourceCaps);

	// Set caps filter after videorate to ensure consistent framerate
	GstElement *capsFilter = gst_element_factory_make("capsfilter", nullptr);
	GstCaps *rateCaps = gst_caps_new_simple("video/x-raw",
						"framerate", GST_TYPE_FRACTION, (unsigned int)_rate, 1,
						NULL);
	g_object_set(G_OBJECT(capsFilter), "caps", rateCaps, NULL);
	gst_caps_unref(rateCaps);

	// Add elements to pipeline
	gst_bin_add_many(GST_BIN(_pipeline), _source, queue1, videoRate, capsFilter, converter, queue2, encoder, payloader,
			 sink, nullptr);

	// Link elements
	if (!gst_element_link_many(_source, queue1, videoRate, capsFilter, converter, queue2, encoder, payloader, sink,
				   nullptr)) {
		gzerr << "Failed to link GStreamer elements" << std::endl;
		gst_object_unref(_pipeline);
		g_main_loop_unref(_gstLoop);
		_pipeline = nullptr;
		_gstLoop = nullptr;
		return;
	}

	// Start pipeline
	if (gst_element_set_state(_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
		gzerr << "Failed to set GStreamer pipeline to playing state" << std::endl;
		gst_object_unref(_pipeline);
		g_main_loop_unref(_gstLoop);
		_pipeline = nullptr;
		_gstLoop = nullptr;
		return;
	}

	gzdbg << "GStreamer pipeline started, streaming to "
	      << (_useRtmp ? _rtmpLocation : (_udpHost + ":" + std::to_string(_udpPort))) << std::endl;

	_running = true;

	// Process frames
	while (_running) {
		std::unique_lock<std::mutex> lock(_frameMutex);

		if (_newFrameAvailable) {
			// Push RGB data directly - we configured the caps to accept RGB
			const guint size = _width * _height * 3; // RGB is 3 bytes per pixel
			GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

			if (buffer) {
				GstMapInfo map;

				if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
					// Copy RGB data directly from the current frame
					memcpy(map.data, _currentFrame.data().c_str(), size);
					gst_buffer_unmap(buffer, &map);

					// Add timing information for smoother playback
					GstClock *clock = gst_system_clock_obtain();
					GstClockTime timestamp = gst_clock_get_time(clock);
					gst_object_unref(clock);

					GST_BUFFER_PTS(buffer) = timestamp;
					GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, (int)_rate);

					GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(_source), buffer);

					if (ret != GST_FLOW_OK) {
						gzerr << "Failed to push buffer to GStreamer pipeline: " << ret << std::endl;
					}

				} else {
					gzerr << "Failed to map GStreamer buffer" << std::endl;
					gst_buffer_unref(buffer);
				}

			} else {
				gzerr << "Failed to allocate GStreamer buffer" << std::endl;
			}

			_newFrameAvailable = false;
		}

		lock.unlock();

		// Sleep to prevent high CPU usage when no frames are available
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	// Cleanup
	gst_element_set_state(_pipeline, GST_STATE_NULL);
	gst_object_unref(_pipeline);
	g_main_loop_unref(_gstLoop);
	_pipeline = nullptr;
	_gstLoop = nullptr;

	gzdbg << "GStreamer thread stopped" << std::endl;
}

// Register this plugin
GZ_ADD_PLUGIN(GstCameraSystem,
	      gz::sim::System,
	      GstCameraSystem::ISystemConfigure,
	      GstCameraSystem::ISystemPostUpdate)

// Add plugin alias for custom namespace
GZ_ADD_PLUGIN_ALIAS(GstCameraSystem, "custom::GstCameraSystem")
