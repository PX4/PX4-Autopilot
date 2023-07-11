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
 * @file Sorting.hpp
 * Implementation of Sorting algorithms.
 *
 */


/**
 * @brief Performs an insert sort on the input pair array.
 * First array is used as the main key. Second array is sorted in the same order as the first one.
 * @param arr1 The base array to be sorted
 * @param arr2 The secondary array to be sorted
 * @param size Size of the input array
 * @param inverse false to order from lower to higher values. true to order from higher to lower
 * @param result void
 */
template<typename T, typename T1>
void sortPair(T arr[], T1 arr1[], int size, bool inverse = false)
{
	T key{};
	T1 index_key{};
	int j = 0;

	for (int i = 0; i < size; i++) {
		key = arr[i];
		index_key = arr1[i];
		j = i - 1;

		if (!inverse) {
			while (j >= 0 && arr[j] > key) {
				arr[j + 1] = arr[j];
				arr1[j + 1] = arr1[j];
				j--;
			}

		} else {
			while (j >= 0 && arr[j] <= key) {
				arr[j + 1] = arr[j];
				arr1[j + 1] = arr1[j];
				j--;
			}

		}

		arr[j + 1] = key;
		arr1[j + 1] = index_key;
	}

}
