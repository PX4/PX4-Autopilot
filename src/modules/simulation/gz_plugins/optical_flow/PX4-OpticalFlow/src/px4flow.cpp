/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 *			     Lorenz Meier <lm@inf.ethz.ch>
 *			     Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 *   Modified: Christoph Tobler <christoph@px4.io>
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

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "px4flow.hpp"


#define TILE_SIZE	8										// x & y tile size
#define NUM_BLOCKS	5 // x & y number of tiles to check


PX4Flow::PX4Flow(uint32_t image_width_, uint32_t search_size_,
		 uint32_t flow_feature_threshold_, uint32_t flow_value_threshold_) :
	image_width(image_width_),
	search_size(search_size_),
	flow_feature_threshold(flow_feature_threshold_),
	flow_value_threshold(flow_value_threshold_)
{

}

uint32_t PX4Flow::__USAD8(uint32_t val1, uint32_t val2)
{
	uint32_t res = 0;
	uint8_t *val1_bytes = (uint8_t *)(&val1);
	uint8_t *val2_bytes = (uint8_t *)(&val2);

	for (int i = 0; i < 4; i++) {
		int16_t v1 = val1_bytes[i];
		int16_t v2 = val2_bytes[i];
		res += (uint32_t)(abs(v1 - v2));
	}

	return res;
}

uint32_t PX4Flow::__USADA8(uint32_t val1, uint32_t val2, uint32_t val3)
{
	uint32_t res = val3;
	uint8_t *val1_bytes = (uint8_t *)(&val1);
	uint8_t *val2_bytes = (uint8_t *)(&val2);

	for (int i = 0; i < 4; i++) {
		int16_t v1 = val1_bytes[i];
		int16_t v2 = val2_bytes[i];
		res += (uint32_t)(abs(v1 - v2));
	}

	return res;
}

uint32_t PX4Flow::__UHADD8(uint32_t val1, uint32_t val2)
{
	uint32_t res = 0;
	uint8_t *res_bytes = (uint8_t *)(&res);
	uint8_t *val1_bytes = (uint8_t *)(&val1);
	uint8_t *val2_bytes = (uint8_t *)(&val2);

	for (int i = 0; i < 4; i++) {
		res_bytes[i] = (val1_bytes[i] + val2_bytes[i]) >> 1;
	}

	return res;
}

/**
 * @brief Compute the average pixel gradient of all horizontal and vertical steps
 *
 * TODO compute_diff is not appropriate for low-light mode images
 *
 * @param image ...
 * @param offX x coordinate of upper left corner of 8x8 pattern in image
 * @param offY y coordinate of upper left corner of 8x8 pattern in image
 */
uint32_t PX4Flow::compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off = (offY + 2) * row_size + (offX + 2); // we calc only the 4x4 pattern
	uint32_t acc;

	/* calc row diff */
	acc = __USAD8(*((uint32_t *) &image[off + 0 + 0 * row_size]), *((uint32_t *) &image[off + 0 + 1 * row_size]));
	acc = __USADA8(*((uint32_t *) &image[off + 0 + 1 * row_size]), *((uint32_t *) &image[off + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t *) &image[off + 0 + 2 * row_size]), *((uint32_t *) &image[off + 0 + 3 * row_size]), acc);

	/* we need to get columns */
	uint32_t col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 0 + 1 * row_size] << 16 | image[off + 0 + 2 *
			row_size] << 8 | image[off + 0 + 3 * row_size];
	uint32_t col2 = (image[off + 1 + 0 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 1 + 2 *
			row_size] << 8 | image[off + 1 + 3 * row_size];
	uint32_t col3 = (image[off + 2 + 0 * row_size] << 24) | image[off + 2 + 1 * row_size] << 16 | image[off + 2 + 2 *
			row_size] << 8 | image[off + 2 + 3 * row_size];
	uint32_t col4 = (image[off + 3 + 0 * row_size] << 24) | image[off + 3 + 1 * row_size] << 16 | image[off + 3 + 2 *
			row_size] << 8 | image[off + 3 + 3 * row_size];

	/* calc column diff */
	acc = __USADA8(col1, col2, acc);
	acc = __USADA8(col2, col3, acc);
	acc = __USADA8(col3, col4, acc);

	return acc;

}

/**
 * @brief Compute SAD distances of subpixel shift of two 8x8 pixel patterns.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 * @param acc array to store SAD distances for shift in every direction
 */
uint32_t PX4Flow::compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y,
				   uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for (uint16_t i = 0; i < 8; i++) {
		acc[i] = 0;
	}


	/*
	 * calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	 * every iteration is one line of the 8x8 field.
	 *
	 *  + - + - + - + - + - + - + - + - +
	 *  |   |   |   |   |   |   |   |   |
	 *  + - + - + - + - + - + - + - + - +
	 *
	 *
	 */

	for (uint16_t i = 0; i < 8; i++) {
		/*
		 * first column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  | x | x | x | x |   |   |   |   |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 * the 8 s values are from following positions for each pixel (X):
		 *  + - + - + - +
		 *  +   5   7   +
		 *  + - + 6 + - +
		 *  +   4 X 0   +
		 *  + - + 2 + - +
		 *  +   3   1   +
		 *  + - + - + - +
		 *
		 *  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		 *
		 */

		/* compute average of two pixel values */
		s0 = (__UHADD8(*((uint32_t *) &image2[off2 +  0 + (i + 0) * row_size]),
			       *((uint32_t *) &image2[off2 + 1 + (i + 0) * row_size])));
		s1 = (__UHADD8(*((uint32_t *) &image2[off2 +  0 + (i + 1) * row_size]),
			       *((uint32_t *) &image2[off2 + 1 + (i + 1) * row_size])));
		s2 = (__UHADD8(*((uint32_t *) &image2[off2 +  0 + (i + 0) * row_size]),
			       *((uint32_t *) &image2[off2 + 0 + (i + 1) * row_size])));
		s3 = (__UHADD8(*((uint32_t *) &image2[off2 +  0 + (i + 1) * row_size]),
			       *((uint32_t *) &image2[off2 - 1 + (i + 1) * row_size])));
		s4 = (__UHADD8(*((uint32_t *) &image2[off2 +  0 + (i + 0) * row_size]),
			       *((uint32_t *) &image2[off2 - 1 + (i + 0) * row_size])));
		s5 = (__UHADD8(*((uint32_t *) &image2[off2 +  0 + (i - 1) * row_size]),
			       *((uint32_t *) &image2[off2 - 1 + (i - 1) * row_size])));
		s6 = (__UHADD8(*((uint32_t *) &image2[off2 +  0 + (i + 0) * row_size]),
			       *((uint32_t *) &image2[off2 + 0 + (i - 1) * row_size])));
		s7 = (__UHADD8(*((uint32_t *) &image2[off2 +  0 + (i - 1) * row_size]),
			       *((uint32_t *) &image2[off2 + 1 + (i - 1) * row_size])));

		/* these 4 t values are from the corners around the center pixel */
		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		/*
		 * finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		 *  + - + - + - +
		 *  |   |   |   |
		 *  + - 5 6 7 - +
		 *  |   4 X 0   |
		 *  + - 3 2 1 - +
		 *  |   |   |   |
		 *  + - + - + - +
		 */

		/* fill accumulation vector */
		acc[0] = __USADA8((*((uint32_t *) &image1[off1 + 0 + i * row_size])), s0, acc[0]);
		acc[1] = __USADA8((*((uint32_t *) &image1[off1 + 0 + i * row_size])), t1, acc[1]);
		acc[2] = __USADA8((*((uint32_t *) &image1[off1 + 0 + i * row_size])), s2, acc[2]);
		acc[3] = __USADA8((*((uint32_t *) &image1[off1 + 0 + i * row_size])), t3, acc[3]);
		acc[4] = __USADA8((*((uint32_t *) &image1[off1 + 0 + i * row_size])), s4, acc[4]);
		acc[5] = __USADA8((*((uint32_t *) &image1[off1 + 0 + i * row_size])), t5, acc[5]);
		acc[6] = __USADA8((*((uint32_t *) &image1[off1 + 0 + i * row_size])), s6, acc[6]);
		acc[7] = __USADA8((*((uint32_t *) &image1[off1 + 0 + i * row_size])), t7, acc[7]);

		/*
		 * same for second column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  |   |   |   |   | x | x | x | x |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 */

		s0 = (__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 0) * row_size]),
			       *((uint32_t *) &image2[off2 + 5 + (i + 0) * row_size])));
		s1 = (__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 1) * row_size]),
			       *((uint32_t *) &image2[off2 + 5 + (i + 1) * row_size])));
		s2 = (__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 0) * row_size]),
			       *((uint32_t *) &image2[off2 + 4 + (i + 1) * row_size])));
		s3 = (__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 1) * row_size]),
			       *((uint32_t *) &image2[off2 + 3 + (i + 1) * row_size])));
		s4 = (__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 0) * row_size]),
			       *((uint32_t *) &image2[off2 + 3 + (i + 0) * row_size])));
		s5 = (__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i - 1) * row_size]),
			       *((uint32_t *) &image2[off2 + 3 + (i - 1) * row_size])));
		s6 = (__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i + 0) * row_size]),
			       *((uint32_t *) &image2[off2 + 4 + (i - 1) * row_size])));
		s7 = (__UHADD8(*((uint32_t *) &image2[off2 + 4 + (i - 1) * row_size]),
			       *((uint32_t *) &image2[off2 + 5 + (i - 1) * row_size])));

		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		acc[0] = __USADA8((*((uint32_t *) &image1[off1 + 4 + i * row_size])), s0, acc[0]);
		acc[1] = __USADA8((*((uint32_t *) &image1[off1 + 4 + i * row_size])), t1, acc[1]);
		acc[2] = __USADA8((*((uint32_t *) &image1[off1 + 4 + i * row_size])), s2, acc[2]);
		acc[3] = __USADA8((*((uint32_t *) &image1[off1 + 4 + i * row_size])), t3, acc[3]);
		acc[4] = __USADA8((*((uint32_t *) &image1[off1 + 4 + i * row_size])), s4, acc[4]);
		acc[5] = __USADA8((*((uint32_t *) &image1[off1 + 4 + i * row_size])), t5, acc[5]);
		acc[6] = __USADA8((*((uint32_t *) &image1[off1 + 4 + i * row_size])), s6, acc[6]);
		acc[7] = __USADA8((*((uint32_t *) &image1[off1 + 4 + i * row_size])), t7, acc[7]);
	}

	return 0;
}

/**
 * @brief Compute SAD of two 8x8 pixel windows.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 */
uint32_t PX4Flow::compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y,
				  uint16_t off2X, uint16_t off2Y, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t acc;
	acc = __USAD8(*((uint32_t *) &image1[off1 + 0 + 0 * row_size]), *((uint32_t *) &image2[off2 + 0 + 0 * row_size]));
	acc = __USADA8(*((uint32_t *) &image1[off1 + 4 + 0 * row_size]), *((uint32_t *) &image2[off2 + 4 + 0 * row_size]), acc);

	acc = __USADA8(*((uint32_t *) &image1[off1 + 0 + 1 * row_size]), *((uint32_t *) &image2[off2 + 0 + 1 * row_size]), acc);
	acc = __USADA8(*((uint32_t *) &image1[off1 + 4 + 1 * row_size]), *((uint32_t *) &image2[off2 + 4 + 1 * row_size]), acc);

	acc = __USADA8(*((uint32_t *) &image1[off1 + 0 + 2 * row_size]), *((uint32_t *) &image2[off2 + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t *) &image1[off1 + 4 + 2 * row_size]), *((uint32_t *) &image2[off2 + 4 + 2 * row_size]), acc);

	acc = __USADA8(*((uint32_t *) &image1[off1 + 0 + 3 * row_size]), *((uint32_t *) &image2[off2 + 0 + 3 * row_size]), acc);
	acc = __USADA8(*((uint32_t *) &image1[off1 + 4 + 3 * row_size]), *((uint32_t *) &image2[off2 + 4 + 3 * row_size]), acc);

	acc = __USADA8(*((uint32_t *) &image1[off1 + 0 + 4 * row_size]), *((uint32_t *) &image2[off2 + 0 + 4 * row_size]), acc);
	acc = __USADA8(*((uint32_t *) &image1[off1 + 4 + 4 * row_size]), *((uint32_t *) &image2[off2 + 4 + 4 * row_size]), acc);

	acc = __USADA8(*((uint32_t *) &image1[off1 + 0 + 5 * row_size]), *((uint32_t *) &image2[off2 + 0 + 5 * row_size]), acc);
	acc = __USADA8(*((uint32_t *) &image1[off1 + 4 + 5 * row_size]), *((uint32_t *) &image2[off2 + 4 + 5 * row_size]), acc);

	acc = __USADA8(*((uint32_t *) &image1[off1 + 0 + 6 * row_size]), *((uint32_t *) &image2[off2 + 0 + 6 * row_size]), acc);
	acc = __USADA8(*((uint32_t *) &image1[off1 + 4 + 6 * row_size]), *((uint32_t *) &image2[off2 + 4 + 6 * row_size]), acc);

	acc = __USADA8(*((uint32_t *) &image1[off1 + 0 + 7 * row_size]), *((uint32_t *) &image2[off2 + 0 + 7 * row_size]), acc);
	acc = __USADA8(*((uint32_t *) &image1[off1 + 4 + 7 * row_size]), *((uint32_t *) &image2[off2 + 4 + 7 * row_size]), acc);

	return acc;
}

/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * and calculates the average offset of all.
 *
 * @param image1 previous image buffer
 * @param image2 current image buffer (new)
 * @param x_rate gyro x rate
 * @param y_rate gyro y rate
 * @param z_rate gyro z rate
 *
 * @return quality of flow calculation
 */
uint8_t PX4Flow::compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate,
			      float z_rate, float *pixel_flow_x, float *pixel_flow_y)
{

	/* constants */
	const int16_t winmin = -search_size;
	const int16_t winmax = search_size;
	const uint16_t hist_size = 2 * (winmax - winmin + 1) + 1;

	/* variables */
	uint16_t pixLo = search_size + 1;
	uint16_t pixHi = image_width - (search_size + 1) - TILE_SIZE;
	uint16_t pixStep = (pixHi - pixLo) / NUM_BLOCKS + 1;
	uint16_t i, j;
	uint32_t acc[8]; // subpixels
	uint16_t histx[hist_size]; // counter for x shift
	uint16_t histy[hist_size]; // counter for y shift
	int8_t  dirsx[64]; // shift directions in x
	int8_t  dirsy[64]; // shift directions in y
	uint8_t  subdirs[64]; // shift directions of best subpixels
	float meanflowx = 0.0f;
	float meanflowy = 0.0f;
	uint16_t meancount = 0;
	float histflowx = 0.0f;
	float histflowy = 0.0f;

	/* initialize with 0 */
	for (j = 0; j < hist_size; j++) { histx[j] = 0; histy[j] = 0; }

	/* iterate over all patterns
	 */
	for (j = pixLo; j < pixHi; j += pixStep) {
		for (i = pixLo; i < pixHi; i += pixStep) {
			/* test pixel if it is suitable for flow tracking */
			uint32_t diff = compute_diff(image1, i, j, image_width);

			if (diff < flow_feature_threshold) {
				continue;
			}

			uint32_t dist = 0xFFFFFFFF; // set initial distance to "infinity"
			int8_t sumx = 0;
			int8_t sumy = 0;
			int8_t ii, jj;

			for (jj = winmin; jj <= winmax; jj++) {

				for (ii = winmin; ii <= winmax; ii++) {
					uint32_t temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, image_width);

					if (temp_dist < dist) {
						sumx = ii;
						sumy = jj;
						dist = temp_dist;
					}
				}
			}

			/* acceptance SAD distance threshhold */
			if (dist < flow_value_threshold) {
				meanflowx += (float) sumx;
				meanflowy += (float) sumy;

				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, image_width);
				uint32_t mindist = dist; // best SAD until now
				uint8_t mindir = 8; // direction 8 for no direction

				for (uint8_t k = 0; k < 8; k++) {
					if (acc[k] < mindist) {
						// SAD becomes better in direction k
						mindist = acc[k];
						mindir = k;
					}
				}

				dirsx[meancount] = sumx;
				dirsy[meancount] = sumy;
				subdirs[meancount] = mindir;
				meancount++;

				/* feed histogram filter*/
				uint8_t hist_index_x = 2 * sumx + (winmax - winmin + 1);

				if (subdirs[i] == 0 || subdirs[i] == 1 || subdirs[i] == 7) { hist_index_x += 1; }

				if (subdirs[i] == 3 || subdirs[i] == 4 || subdirs[i] == 5) { hist_index_x += -1; }

				uint8_t hist_index_y = 2 * sumy + (winmax - winmin + 1);

				if (subdirs[i] == 5 || subdirs[i] == 6 || subdirs[i] == 7) { hist_index_y += -1; }

				if (subdirs[i] == 1 || subdirs[i] == 2 || subdirs[i] == 3) { hist_index_y += 1; }

				histx[hist_index_x]++;
				histy[hist_index_y]++;

			}
		}
	}

	/* evaluate flow calculation */
	if (meancount > 10) {
		meanflowx /= meancount;
		meanflowy /= meancount;

		// int16_t maxpositionx = 0;
		// int16_t maxpositiony = 0;
		// uint16_t maxvaluex = 0;
		// uint16_t maxvaluey = 0;
		//
		// /* position of maximal histogram peek */
		// for (j = 0; j < hist_size; j++) {
		// 	if (histx[j] > maxvaluex) {
		// 		maxvaluex = histx[j];
		// 		maxpositionx = j;
		// 	}
		//
		// 	if (histy[j] > maxvaluey) {
		// 		maxvaluey = histy[j];
		// 		maxpositiony = j;
		// 	}
		// }

		/* check if there is a peak value in histogram */
		if (1) { //(histx[maxpositionx] > meancount / 6 && histy[maxpositiony] > meancount / 6)

			/* use average of accepted flow values */
			uint32_t meancount_x = 0;
			uint32_t meancount_y = 0;

			for (uint8_t h = 0; h < meancount; h++) {
				float subdirx = 0.0f;

				if (subdirs[h] == 0 || subdirs[h] == 1 || subdirs[h] == 7) { subdirx = 0.5f; }

				if (subdirs[h] == 3 || subdirs[h] == 4 || subdirs[h] == 5) { subdirx = -0.5f; }

				histflowx += (float)dirsx[h] + subdirx;
				meancount_x++;

				float subdiry = 0.0f;

				if (subdirs[h] == 5 || subdirs[h] == 6 || subdirs[h] == 7) { subdiry = -0.5f; }

				if (subdirs[h] == 1 || subdirs[h] == 2 || subdirs[h] == 3) { subdiry = 0.5f; }

				histflowy += (float)dirsy[h] + subdiry;
				meancount_y++;
			}

			histflowx /= meancount_x;
			histflowy /= meancount_y;

			/* without gyro compensation */
			*pixel_flow_x = histflowx;
			*pixel_flow_y = histflowy;

		}

		/* no peak value in histogram */
		else {
			*pixel_flow_x = 0.0f;
			*pixel_flow_y = 0.0f;
			return 0;
		}
	}

	/* no peak value in histogram */
	else {
		*pixel_flow_x = 0.0f;
		*pixel_flow_y = 0.0f;
		return 0;
	}

	/* calc quality */
	uint8_t qual = (uint8_t)(meancount * 255 / (NUM_BLOCKS * NUM_BLOCKS));

	return qual;
}
