/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>	// for usleep()

#include <vector>
#include <list>
#include <mutex>

#include <modal_pipe.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Function prototypes */

// Generic MPA functions
int create_pub(char* pub_name, char* server_name, char* data_type, int pipe_size);
void close_pub(int channel);
void close_sub(int channel);
int get_num_clients(int channel);

// Camera specific MPA functions
int camera_subscribe(char* camera_name, char* client_name);
int get_image(int channel, uint8_t* frame, camera_image_metadata_t *meta);
int publish_image(int channel, uint8_t* image_data, camera_image_metadata_t* pmeta);

// IMU specific MPA functions
int publish_imu(int channel, imu_data_t *imu_data);

#ifdef __cplusplus
}
#endif

class CameraFrame
{
public:
	CameraFrame(char* frame_in, camera_image_metadata_t meta_in)
	{
		meta = meta_in;
		frame.resize(meta.size_bytes);
		memcpy(&frame[0], frame_in, meta.size_bytes);
	}

	camera_image_metadata_t meta;
	std::vector<uint8_t> frame;
};

class CameraFrameList
{
public:
	CameraFrameList(int channel) : channel(channel) { m = new std::mutex; }

	int GetChannel() { return channel; }

	void StoreFrame(char* frame, camera_image_metadata_t meta) {
		std::lock_guard<std::mutex> lock(*m);
		frame_list.push_back(CameraFrame(frame, meta));

		// Prune stale frames
		while (frame_list.size() > BUFFER_MAX_NUM_FRAMES) {
			frame_list.pop_front();
		}
	}

	int GetFrame(uint8_t* image_data, camera_image_metadata_t* pmeta) {
		uint32_t img_size = 0;

		const int ntries = 20;

		for (int i=0; i<ntries; i++) {
			{
				std::lock_guard<std::mutex> lock(*m);

				if (frame_list.size() > 0)
				{
					CameraFrame &buf = frame_list.front();
					memcpy(image_data, &(buf.frame[0]), buf.meta.size_bytes);   //copy image
					memcpy(pmeta, &buf.meta, sizeof(camera_image_metadata_t));  //copy meta
					img_size = buf.meta.size_bytes;

					frame_list.pop_front();
				}
			}

			if (img_size > 0) break;
			else usleep(2000);
		}

		return img_size;
	}


private:
	static const int BUFFER_MAX_NUM_FRAMES{10};
	int channel{-1};
	std::mutex *m;
	std::list<CameraFrame> frame_list;
};

class CameraFrameManager
{
public:
	CameraFrameManager() {}

	void AddChannel(int channel) {
		frame_list.push_back(CameraFrameList(channel));
	}

	void StoreFrame(int channel, char* frame, camera_image_metadata_t meta) {
		for (std::list<CameraFrameList>::iterator it = frame_list.begin(); it != frame_list.end(); ++it) {
			if (it->GetChannel() == channel) {
				it->StoreFrame(frame, meta);
				break;
			}
		}
	}

	int GetFrame(int channel, uint8_t* image_data, camera_image_metadata_t* pmeta)
	{
		int image_bytes = 0;
		for (std::list<CameraFrameList>::iterator it = frame_list.begin(); it != frame_list.end(); ++it) {
			if (it->GetChannel() == channel) {
				image_bytes = it->GetFrame(image_data, pmeta);
				break;
			}
		}
		return image_bytes;
	}

private:
	std::list<CameraFrameList> frame_list;
};

static CameraFrameManager frame_manager;

/* Function definitions */

// camera helper callback whenever a frame arrives
static void _helper_cb(int ch, camera_image_metadata_t meta, char* frame, __attribute__((unused)) void* context)
{
	frame_manager.StoreFrame(ch, frame, meta);
	return;
}

int camera_subscribe(char* camera_name, char* client_name)
{
	char pipe_name[64];
	int ch = -1;

	printf("connecting to camera %s\n", camera_name);

	if (pipe_expand_location_string(camera_name, pipe_name) < 0) {
		fprintf(stderr, "ERROR: Invalid pipe name: %s\n", camera_name);
	} else {
		ch = pipe_client_get_next_available_channel();
		if (ch != -1) {
			if (pipe_client_set_camera_helper_cb(ch, _helper_cb, NULL) != -1) {
				int buf_len = 0;
				if (pipe_client_open(ch, pipe_name, client_name, EN_PIPE_CLIENT_CAMERA_HELPER, buf_len) != -1) {
					frame_manager.AddChannel(ch);
					printf("subscribed to camera %s, channel %d\n", pipe_name, ch);
				}
			}
		} else {
			fprintf(stderr, "ERROR: couldn't allocate new channel for camera %s\n", camera_name);
		}
	}

	return ch;
}

int get_image(int channel, uint8_t* frame, camera_image_metadata_t* meta) {
	return frame_manager.GetFrame(channel, frame, meta);
}


int create_pub(char* pub_name, char* server_name, char* data_type, int pipe_size)
{
	pipe_info_t info;
	strcpy(info.name       , pub_name);
	strcpy(info.type       , data_type);
	strcpy(info.server_name, server_name);
	info.size_bytes = pipe_size;

	int ch    = pipe_server_get_next_available_channel();
	int flags = 0;

	pipe_server_create(ch, info, flags);
	printf("created output pipe %s, channel %d, flags %d\n", info.name, ch, flags);

	return ch;
}

int publish_image(int channel, uint8_t* image_data, camera_image_metadata_t* pmeta)
{
	return pipe_server_write_camera_frame(channel, *pmeta, image_data);
}

int publish_imu(int channel, imu_data_t *imu_data)
{
	return pipe_server_write(channel, (const void*) imu_data, sizeof(imu_data_t));
}

int get_num_clients(int channel)
{
	return pipe_server_get_num_clients(channel);
}

void close_pub(int channel)
{
	printf("Closing output pipe %d\n", channel);
	pipe_server_close(channel);
}

void close_sub(int channel)
{
	printf("Closing input pipe %d\n", channel);
	pipe_client_close(channel);
}
