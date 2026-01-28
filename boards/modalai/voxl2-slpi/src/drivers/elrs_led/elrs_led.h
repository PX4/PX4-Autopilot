/****************************************************************************
 *
 *   Copyright (c) 2023 ModalAI, Inc. All rights reserved.
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

#include <string>
#include <map>

#define crclen 256
#define CRSF_CRC_POLY 0xd5
#define CRSF_FRAME_NOT_COUNTED_BYTES 2
#define PWM_FRAME_SIZE 9


enum class ControllerInput : uint32_t {
	DLEFT = 0x2000,
	DRIGHT = 0x4000,
	DDOWN = 0x1000,
	DUP = 0x800,
	BACK = 0x10,
	START = 0x40,
	Y = 0x08,
	B = 0x02,
	A = 0x01,
	X = 0x04,
	STICK_RIGHT = 0x100,
	STICK_LEFT = 0x80,
	BUMPER_RIGHT = 0x400,
	BUMPER_LEFT = 0x200,
	DEFAULT = 0xFFFFFFFF
};

enum class LEDState : uint8_t {
	OFF = 0x00,
	ON = 0x01,
	IR = 0x02,
	DEFAULT = 0xFF
};


class GENERIC_CRC8
{
public:
	GENERIC_CRC8() {};
	uint8_t calc(const uint8_t *data, uint16_t len, uint8_t crc)
	{
		while (len--) {
			crc = crc8tab[crc ^ *data++];
		}

		return crc;
	}

private:
	uint8_t crc8tab[crclen] = {0, 213, 127, 170, 254, 43, 129, 84, 41, 252, 86, 131, 215, 2, 168, 125, 82, 135,
				   45, 248, 172, 121, 211, 6, 123, 174, 4, 209, 133, 80, 250, 47, 164, 113, 219, 14,
				   90, 143, 37, 240, 141, 88, 242, 39, 115, 166, 12, 217, 246, 35, 137, 92, 8, 221,
				   119, 162, 223, 10, 160, 117, 33, 244, 94, 139, 157, 72, 226, 55, 99, 182, 28, 201,
				   180, 97, 203, 30, 74, 159, 53, 224, 207, 26, 176, 101, 49, 228, 78, 155, 230, 51,
				   153, 76, 24, 205, 103, 178, 57, 236, 70, 147, 199, 18, 184, 109, 16, 197, 111, 186,
				   238, 59, 145, 68, 107, 190, 20, 193, 149, 64, 234, 63, 66, 151, 61, 232, 188, 105,
				   195, 22, 239, 58, 144, 69, 17, 196, 110, 187, 198, 19, 185, 108, 56, 237, 71, 146,
				   189, 104, 194, 23, 67, 150, 60, 233, 148, 65, 235, 62, 106, 191, 21, 192, 75, 158,
				   52, 225, 181, 96, 202, 31, 98, 183, 29, 200, 156, 73, 227, 54, 25, 204, 102, 179, 231,
				   50, 152, 77, 48, 229, 79, 154, 206, 27, 177, 100, 114, 167, 13, 216, 140, 89, 243,
				   38, 91, 142, 36, 241, 165, 112, 218, 15, 32, 245, 95, 138, 222, 11, 161, 116, 9, 220,
				   118, 163, 247, 34, 136, 93, 214, 3, 169, 124, 40, 253, 87, 130, 255, 42, 128, 85, 1,
				   212, 126, 171, 132, 81, 251, 46, 122, 175, 5, 208, 173, 120, 210, 7, 83, 134, 44, 249
				  };
};


ControllerInput getKey(const std::map<ControllerInput, std::string> &map, const std::string &value)
{
	for (const auto &pair : map) {
		if (pair.second == value) {
			return pair.first;
		}
	}

	return ControllerInput::DEFAULT;
}
