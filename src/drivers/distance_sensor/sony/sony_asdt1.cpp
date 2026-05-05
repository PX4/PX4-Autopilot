/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file sony_asdt1.cpp
 * @author Andrew Brahim <brahim@ascendengineer.com>
 * @author Apoorv Thapliyal
 *
 * Driver for the Sony ASDT1 lidar
 */

#include "sony_asdt1.hpp"
#include <px4_platform_common/defines.h>
#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>

AS_DT1::AS_DT1(const char *device, uint8_t rotation):
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_px4_rangefinder(0, rotation)
{
	// store port name
	strncpy(_device, device, sizeof(_device) - 1);

	// enforce null termination
	_device[sizeof(_device) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_ASDT1;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	// uint8_t bus_num = atoi(&_device[strlen(_device) - 1]); // Assuming '/dev/ttySx'
	//
	// if (bus_num < 10) {
	// 	device_id.devid_s.bus = bus_num;
	// }

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	// Initialize with 2 rows (index 0 and 1)
	// points.resize(2);
	// Reserve space for 576 points each
	// points[0].reserve(576);
	// points[1].reserve(576);

	// _MPDATATBL - transmission order to raw index mapping
	const int MPDATATBL[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
		48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,
		288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311,
		336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359,
		96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
		144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167,
		384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407,
		432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455,
		192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215,
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263,
		480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503,
		528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551,
		24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
		312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335,
		360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383,
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
		168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
		408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431,
		456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479,
		216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
		264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287,
		504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527,
		552, 553, 554, 555, 556, 557, 558, 559, 560, 561, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574, 575
	};

	// Build inverse mapping: xyDataTbl[mp] = xycnt


	for (int xycnt = 0; xycnt < mpdata_s; xycnt++) {
		int mp = MPDATATBL[xycnt];
		xyDataTbl(mp) = xycnt;
	}
}
AS_DT1::~AS_DT1()
{

	if (_uart.isOpen()) {
		_uart.close();
	}
}
int AS_DT1::init()
{
	// If connection fails, print error and return false
	if (_uart.isOpen()) {
		if (_uart.setBaudrate(baud) == false) {
			PX4_ERR("Failed to set UART %lu baud", static_cast<unsigned long>(baud));
		}
	}

	return PX4_OK;
}

bool AS_DT1::writeCommand(const uint8_t *data, size_t length)
{
	//TODO:
	//const uint8_t *command = data + "\r";

	// If write fails, print error and return false
	if (!_uart.write(const_cast<uint8_t *>(data), length)) {
		PX4_ERR("Failed to write to AS-DT1");
		return PX4_ERROR;
	}

	return PX4_OK;
}

// void AS_DT1::readThreadFunction()
// {
// 	while (reading) {
// 		char buf[50000];
// 		int n = SERIAL_.read(buf, sizeof(buf));
//
// 		if (n > 0) {
// 			read_buffer.append(buf, n);
// 			// std::cout.write(buf, n);
// 			// std::cout.flush();
//
// 			// If "END" found, clear buffer and send command
// 			if (read_buffer.find("END") != std::string::npos) {
// 				// writeCommand("");
//
// 				// Store the latest read data
// 				latest_read_data = read_buffer;
//
// 				// Clear the buffer for the next read
// 				read_buffer.clear();
// 			}
// 		}
//
// 		// std::this_thread::sleep_for(std::chrono::milliseconds(1));
// 	}
// }
// TODO:
void AS_DT1::print_info()
{

}
int AS_DT1::collect()
{

}

AS_DT1::start()
{
	//TODO:
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(7_ms);
}

void
AS_DT1::stop()
{
	ScheduleClear();
}


bool AS_DT1::convertBinaryToPCD()
{
	// Thread-safe copy of latest read data
	std::string binary_pcd_data;
	{
		std::lock_guard<std::mutex> lock(buffer_mutex);
		binary_pcd_data = latest_read_data;
	}

	std::cout << "Latest read data length: " << binary_pcd_data.size() << "\n";
	// std::cout << "Latest read data : \n" << binary_pcd_data << "\n";

	// If the data is empty, print error and return false
	if (binary_pcd_data.empty()) {
		std::cerr << "No binary data to convert to PCD\n";
		return false;
	}

	// Find "BEGIN MP" and "END"
	size_t begin_pos = binary_pcd_data.find("BEGIN MP");
	size_t end_pos = binary_pcd_data.find("END");

	// Print the string before BEGIN MP
	// std::cout << "Data before BEGIN MP: \n" << binary_pcd_data.substr(0, begin_pos) << "\n";

	std::cout << "BEGIN MP position: " << begin_pos << "\n";
	std::cout << "END position: " << end_pos << "\n";
	// std::cout << "Data between BEGIN MP and END: \n" << binary_pcd_data.substr(begin_pos+8, end_pos-begin_pos-8) << "\n";

	// If either "BEGIN MP" or "END" is not found, print error and return false
	if (begin_pos == std::string::npos || end_pos == std::string::npos || end_pos <= begin_pos) {
		std::cerr << "Invalid binary data format: missing BEGIN MP or END\n";
		return false;
	}

	// Extract the binary data between "BEGIN MP" and "END"
	size_t data_start = begin_pos + std::string("BEGIN MP\r\n").length();
	size_t data_length = end_pos - data_start;
	std::cout << "Binary data length: " << data_length << "\n";
	const uint8_t *data_ptr = (const uint8_t *)binary_pcd_data.c_str() + data_start;

	// Read length is set to 4320
	size_t read_length = 4320;

	//TODO:
	// Temporary points, fill it with 0s first, then we will fill it with the extracted points
	// matrix::Vector3f pcd_raw(2);
	// pcd_raw[0].reserve(576);
	// pcd_raw[1].reserve(576);

	// // Fill temporary points with 0s
	// for (int i = 0; i < 576; ++i) {
	//     pcd_raw[0].emplace_back(Point3D{0, 0, 0});
	//     pcd_raw[1].emplace_back(Point3D{0, 0, 0});
	// }

	// int count = 0;
	for (size_t dr_cnt = 0; dr_cnt + 15 <= 4320; dr_cnt += 15) {
		try {
			// Extract 20-bit values from 15 bytes
			// XXXXX YYYYY ZZZZZ XXXXX YYYYY ZZZZZ
			// XX XX XY YY YY ZZ ZZ ZX XX XX YY YY YZ ZZ ZZ
			//  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14
			int32_t x1 = (data_ptr[dr_cnt + 0] << 12) | (data_ptr[dr_cnt + 1] << 4) | (data_ptr[dr_cnt + 2] >> 4);
			int32_t y1 = ((data_ptr[dr_cnt + 2] & 0x0f) << 16) | (data_ptr[dr_cnt + 3] << 8) | data_ptr[dr_cnt + 4];
			int32_t z1 = (data_ptr[dr_cnt + 5] << 12) | (data_ptr[dr_cnt + 6] << 4) | (data_ptr[dr_cnt + 7] >> 4);
			int32_t x2 = ((data_ptr[dr_cnt + 7] & 0x0f) << 16) | (data_ptr[dr_cnt + 8] << 8) | data_ptr[dr_cnt + 9];
			int32_t y2 = (data_ptr[dr_cnt + 10] << 12) | (data_ptr[dr_cnt + 11] << 4) | (data_ptr[dr_cnt + 12] >> 4);
			int32_t z2 = ((data_ptr[dr_cnt + 12] & 0x0f) << 16) | (data_ptr[dr_cnt + 13] << 8) | data_ptr[dr_cnt + 14];

			// Convert from 20-bit two's complement to signed
			if (x1 & 0x80000) { x1 = -(0x100000 - x1); }

			if (y1 & 0x80000) { y1 = -(0x100000 - y1); }

			if (x2 & 0x80000) { x2 = -(0x100000 - x2); }

			if (y2 & 0x80000) { y2 = -(0x100000 - y2); }

			// Store pcd_raw[0][count] and pcd_raw[1][count]
			Eigen::Vector3f p0, p1;
			p0 << x1 / 4.0f, y1 / 4.0f, z1 / 4.0f;
			p1 << x2 / 4.0f, y2 / 4.0f, z2 / 4.0f;

			pcd_raw[0].push_back(p0);
			pcd_raw[1].push_back(p1);
			// count++;

		} catch (...) {
			std::cerr << "Error processing data\n";
		}
	}

	// Make the PCD from the temp points
	std::vector<std::vector<Eigen::Vector3f>> pcd(2);
	pcd[0].resize(576);
	pcd[1].resize(576);

	float thrMin = range_min;
	float thrMax = range_max;

	// Iterate over 576 points
	for (int cnt = 0; cnt < 576; cnt++) {
		// tx order to picture order
		int mp = xyDataTbl[cnt];

		// pcd[1][cnt] = pcd_raw[1][mp];
		Eigen::Vector3f x1_y1_z1 = pcd_raw[1][mp];
		pcd[1][cnt] = x1_y1_z1;

		// Range filtering
		Eigen::Vector3f x_y_z = pcd_raw[0][mp];
		float z = x_y_z(2);
		float z1 = x1_y1_z1(2);

		if (std::abs(z) < thrMin || std::abs(z) > thrMax) {
			if (std::abs(z1) < thrMin || std::abs(z1) > thrMax) {
				// ignore both z and z1
				pcd[0][cnt] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

			} else {
				// z is invalid, z1 is valid
				pcd[0][cnt] = x1_y1_z1;
			}

		} else {
			// z is valid
			pcd[0][cnt] = x_y_z;
		}
	}

	// Step 3: Convert to meters (divide by 1000)
	for (int i = 0; i < 2; i++) {
		for (auto &p : pcd[i]) {
			p(0) /= 1000.0f;
			p(1) /= 1000.0f;
			p(2) /= 1000.0f;
		}
	}

	// Rotate -90 degrees around y-axis
	for (int i = 0; i < 2; i++) {
		for (auto &p : pcd[i]) {
			float x_new = p(2);
			float z_new = -p(0);
			p(0) = x_new;
			p(2) = z_new;
		}
	}

	std::cout << "Successfully converted " << pcd[0].size() << " points\n";

	// Print all the points
	for (size_t i = 0; i < pcd[0].size(); ++i) {
		const Eigen::Vector3f &p = pcd[0][i];
		std::cout << "Point " << i << ": (" << p(0) << ", " << p(1) << ", " << p(2) << ")\n";
	}

	// Update points
	// {
	//     std::lock_guard<std::mutex> lock(buffer_mutex);
	//     points = pcd;
	// }

	return true;
}
