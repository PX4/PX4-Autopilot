/**
 * @file tracker.cpp
 *
 * Tracks the flight graph in a memory optimized format.
 * See tracker.h for an overview.
 *
 * @author Samuel Sadok <samuel.sadok@bluewin.ch>
 */

#include <limits.h>     // CHAR_BIT
#include <lib/mathlib/math/Limits.hpp>  // std::min, max
#include <cstdlib>      // std::abs
#include <cmath>        // std::abs
#include <float.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include "tracker.h"


using namespace std;


template<class T> void myswap(T &a, T &b)
{
	T temp(a);
	a = b;
	b = temp;
}


float Tracker::fast_sqrt(int val, bool fallback_to_infinity)
{
	const float values[] = {
		0.000000000000000f, 1.000000000000000f, 1.414213562373095f, 1.732050807568877f,
		2.000000000000000f, 2.236067977499790f, 2.449489742783178f, 2.645751311064591f,
		2.828427124746190f, 3.000000000000000f, 3.162277660168380f, 3.316624790355400f,
		3.464101615137754f, 3.605551275463989f, 3.741657386773941f, 3.872983346207417f,
		4.000000000000000f, 4.123105625617661f, 4.242640687119285f, 4.358898943540674f,
		4.472135954999580f, 4.582575694955840f, 4.690415759823430f, 4.795831523312719f,
		4.898979485566356f, 5.000000000000000f, 5.099019513592784f, 5.196152422706632f,
		5.291502622129181f, 5.385164807134504f, 5.477225575051661f, 5.567764362830022f
	};

	if (val < sizeof(values) / sizeof(values[0])) {
		return values[val];
	}

	if (fallback_to_infinity) {
		return INFINITY;
	}

	float result = sqrt(val);
	return result;
}


int Tracker::dot(ipos_t vec1, ipos_t vec2)
{
	if (!fits_into_far_delta(vec1) || !fits_into_far_delta(vec2)) {
		return INT_MAX;
	}

	return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}


Tracker::ipos_t Tracker::get_point_to_line_delta(ipos_t point, ipos_t line_delta, ipos_t line_end, int &coef)
{
	point -= line_end;

	// The negative coefficient is obtained by projecting the point along the line direction
	coef = float_to_coef(-dot(point, line_delta) / (float)dot(line_delta));

	// Constrain coefficient to beginning or end of line
	int max_coefficient = MAX_COEFFICIENT;
	coef = math::max(0, math::min(max_coefficient, coef));

	//ipos_t result = -apply_coef(line_delta, coef) - point;
	//TRACKER_DBG("projecting (%d %d %d) to (delta %d %d %d end %d %d %d) gives coef %d, which gives (%d %d %d)", point.x, point.y, point.z,
	//    line_delta.x, line_delta.y, line_delta.z,
	//    line_end.x, line_end.y, line_end.z,
	//    coef,
	//    result.x, result.y, result.z);

	// Return result
	return -apply_coef(line_delta, coef) - point;
}


Tracker::ipos_t Tracker::get_line_to_line_delta(ipos_t delta1, ipos_t end1, ipos_t delta2, ipos_t end2, int &coef1,
		int &coef2, bool pin_coef1, bool pin_coef2)
{

	// If either of the two coefficients is pinned, apply it to the line end
	if (pin_coef1) {
		end1 -= apply_coef(delta1, coef1);
	}

	if (pin_coef2) {
		end2 -= apply_coef(delta2, coef2);
	}

	if (pin_coef1 && pin_coef2) {
		return end2 - end1;
	}

	// A point-to-line distance is sufficient if a coefficient is pinned.
	if (pin_coef1) {
		return get_point_to_line_delta(end1, delta2, end2, coef2);
	}

	if (pin_coef2) {
		return -get_point_to_line_delta(end2, delta1, end1, coef1);
	}

	// None of the coefficients are pinned, so we go on to calculate the true line-to-line delta

	// Find the direction that is normal to both lines (using cross product)
	ipos_t normal = {
		.x = delta1.y * delta2.z - delta1.z * delta2.y,
		.y = delta1.z * delta2.x - delta1.x * delta2.z,
		.z = delta1.x * delta2.y - delta1.y * delta2.x
	};

	// Project a connection between both lines along the direction of the normal.
	ipos_t end_to_end_delta = end2 - end1;
	fpos_t temp_delta = to_fpos(normal) * (dot(end_to_end_delta, normal) / (float)dot(normal));

	// Remove the orthogonal component - the end-to-end delta is now a linear combination of both line directions
	end_to_end_delta -= to_ipos(temp_delta);

	// Now we have the following system of equations:
	// (-delta1 delta2) * (coef1 coef2)^T = end_to_end_delta
	//
	// This is over-determined, so what we solve instead is:
	// (coef1 coef2)^T = ((-delta1 delta2)^T * (-delta1 delta2))^-1 * (-delta1 delta2)^T * end_to_end_delta
	//                    \---------------- A ----------------/       \---------------- b ----------------/
	//
	// A = | dot(-deltaA, -deltaA)   dot(-deltaA, deltaB) |
	//     | dot(-deltaA, deltaB)    dot(deltaB, deltaB)  |

	// Precalculate A and b
	int delta1delta1 = dot(delta1, delta1);
	int delta1delta2 = dot(delta1, delta2);
	int delta2delta2 = dot(delta2, delta2);
	int b1 = -dot(delta1, end_to_end_delta);
	int b2 = dot(delta2, end_to_end_delta);

	// If rounding temp_delta is not desired, use this (with unmodified end_to_end_delta!):
	//temp_delta = to_fpos(end_to_end_delta) - temp_delta;
	//float b1 = -(delta1.x * temp_delta.x + delta1.y * temp_delta.y + delta1.z * temp_delta.z);
	//float b2 = (delta2.x * temp_delta.x + delta2.y * temp_delta.y + delta2.z * temp_delta.z);

	// Prepare inverse of A
	float determinant = delta1delta1 * delta2delta2 - delta1delta2 * delta1delta2;

	// Calculate coefficients and convert to integers
	coef1 = float_to_coef((delta2delta2 * b1 + delta1delta2 * b2) / determinant);
	coef2 = float_to_coef((delta1delta2 * b1 + delta1delta1 * b2) / determinant);

	// If both coefficients are within the representable range, we can use this as the result
	if (coef1 >= 0 && coef1 <= MAX_COEFFICIENT && coef2 >= 0 && coef2 <= MAX_COEFFICIENT) {
		end1 -= apply_coef(delta1, coef1);
		end2 -= apply_coef(delta2, coef2);

		// The result should theoretically be equal to temp_delta.
		// In practice, we will probably have a different discretization error when applying the coefficients.
		// We want to be sure here that the delta is really what is obtained by applying the returned coefficient.
		return end2 - end1;
	}


	// If a coefficient is out of reach, we need return one out of four possible point-to-line distances.
	// There are probably optimizations we could apply to this, but it's probably not neccessary.

	// Calculate the delta from each point to the other line.
	int temp_coef1, temp_coef2;
	ipos_t start1 = end1 - apply_coef(delta1, MAX_COEFFICIENT);
	ipos_t start2 = end2 - apply_coef(delta2, MAX_COEFFICIENT);
	ipos_t start1_to_line2 = get_point_to_line_delta(start1, delta2, end2, temp_coef2);
	ipos_t start2_to_line1 = get_point_to_line_delta(start2, delta1, end1, temp_coef1);
	ipos_t end1_to_line2 = get_point_to_line_delta(end1, delta2, end2, coef2);
	ipos_t end2_to_line1 = get_point_to_line_delta(end2, delta1, end1, coef1);

	//TRACKER_DBG("delta1: (%d %d %d), delta2: (%d %d %d), delta3: (%d %d %d), delta4: (%d %d %d)",
	//    start1_to_line2.x, start1_to_line2.y, start1_to_line2.z,
	//    end1_to_line2.x, end1_to_line2.y, end1_to_line2.z,
	//    start2_to_line1.x, start2_to_line1.y, start2_to_line1.z,
	//    end2_to_line1.x, end2_to_line1.y, end2_to_line1.z);

	// Of all the point-to-line deltas, return the smallest one.
	int best_to_line1 = math::min(dot(start2_to_line1), dot(end2_to_line1));
	int best_to_line2 = math::min(dot(start1_to_line2), dot(end1_to_line2));

	if (best_to_line1 < best_to_line2) {
		if (dot(start2_to_line1) <= best_to_line1) {
			coef1 = temp_coef1;
			coef2 = MAX_COEFFICIENT;
			return -start2_to_line1;

		} else { // end2_to_line1 is the shortest delta
			coef2 = 0;
			return -end2_to_line1;
		}

	} else {
		if (dot(start1_to_line2) <= best_to_line2) {
			coef1 = MAX_COEFFICIENT;
			coef2 = temp_coef2;
			return start1_to_line2;

		} else { // end1_to_line2 is the shortest delta
			coef1 = 0;
			return end1_to_line2;
		}
	}
}


inline Tracker::delta_item_t Tracker::pack_compact_delta_item(ipos_t delta)
{
	return ((delta.x & 0x1F) << 10) | ((delta.y & 0x1F) << 5) | ((delta.z & 0x1F) << 0);
}

inline Tracker::ipos_t Tracker::unpack_compact_delta_item(delta_item_t delta)
{
	const int SHIFT_COUNT = CHAR_BIT * sizeof(int) - 5;
	return {
		.x = ((int)(delta << (SHIFT_COUNT - 10))) >> SHIFT_COUNT,
		.y = ((int)(delta << (SHIFT_COUNT - 5))) >> SHIFT_COUNT,
		.z = ((int)(delta << (SHIFT_COUNT - 0))) >> SHIFT_COUNT
	};
}

inline bool Tracker::fits_into_far_delta(ipos_t vec)
{
	return (((vec.x >> 14) == 0) || ((vec.x >> 14) == -1)) &&
	       (((vec.y >> 14) == 0) || ((vec.y >> 14) == -1)) &&
	       (((vec.z >> 14) == 0) || ((vec.z >> 14) == -1));
}


bool Tracker::push_delta(size_t &index, ipos_t delta, bool jump, size_t max_space, delta_item_t *buffer)
{
	int max_val = math::max(math::max(delta.x, delta.y), delta.z);
	int min_val = math::min(math::min(delta.x, delta.y), delta.z);

	// If the delta is too large for a single compact delta item, we split it into multiple items
	int split_count = 1;

	if (max_val > COMPACT_DELTA_MAX || min_val < COMPACT_DELTA_MIN) {
		split_count = (int)ceil(math::max((float)max_val / (float)COMPACT_DELTA_MAX,
						  (float)min_val / (float)COMPACT_DELTA_MIN));
	}

	int far_delta_size = FAR_DELTA_SIZE;
	split_count = jump ? far_delta_size : math::min(split_count, far_delta_size);

	if (max_space < split_count) {
		return false;
	}

	if (split_count >= FAR_DELTA_SIZE || jump) {
		if (max_val > FAR_DELTA_MAX || min_val < FAR_DELTA_MIN) {
			PX4_ERR("far delta overflow");
			return false;
		}

		buffer[index++] = (delta.x & 0x7FFF) | 0x8000;
		buffer[index++] = (delta.y & 0x7FFF) | (jump ? 0x8000 : 0);
		buffer[index++] = (delta.z & 0x7FFF) | 0x8000;

	} else {

		ipos_t compact_delta_sum = { .x = 0, .y = 0, .z = 0 };

		for (int i = 1; i < split_count; i++) {
			ipos_t compact_delta = to_ipos(to_fpos(delta) * ((float)i / (float)split_count)) - compact_delta_sum;
			buffer[index++] = pack_compact_delta_item(compact_delta);
			compact_delta_sum += compact_delta;
		}

		delta -= compact_delta_sum;

		buffer[index++] = pack_compact_delta_item(delta);
	}

	return true;
}


Tracker::ipos_t Tracker::fetch_delta(size_t index, bool &is_jump, delta_item_t *buffer)
{
	const int SHIFT_COUNT = CHAR_BIT * sizeof(int) - 15;

	if (!((buffer[index] >> 15) & 1)) {
		is_jump = false;
		return unpack_compact_delta_item(buffer[index]);
	}

	//if (buffer[index] == OBSOLETE_DELTA || buffer[index - 1] == OBSOLETE_DELTA || buffer[index - 2] == OBSOLETE_DELTA)
	//GRAPH_ERR("fetching invalid delta element");

	is_jump = (buffer[index - 1] >> 15) & 1;
	return {
		.x = ((int)((buffer[index - 2] & 0x7FFF) << SHIFT_COUNT)) >> SHIFT_COUNT,
		.y = ((int)((buffer[index - 1] & 0x7FFF) << SHIFT_COUNT)) >> SHIFT_COUNT,
		.z = ((int)((buffer[index - 0] & 0x7FFF) << SHIFT_COUNT)) >> SHIFT_COUNT
	};
}


int Tracker::get_granularity_at(ipos_t pos)
{
	int dist_squared = dot(home_position - pos);

	// Calculate max(0, floor(log2(dist_squared))): 0->0, 1->0, 2->1, 3->1, 4->2, 7->2, 8->3, ...
	int log_dist = 0;

	while (dist_squared >>= 1) {
		log_dist++;
	}

	log_dist = (log_dist - HIGH_PRECISION_RANGE) >> 1;

	// The graph precision is lowered every second time the memory pressure increases
	int p = (memory_pressure >> 1) + 1;

	int margin = math::max(((p - 1) >> 1) + 1, log_dist * p);
	//                    \___ regime 1 ___/  \ regime 2 /

	// Regime 1: the margin close to home shall increase every second time we reduce precision
	// Regime 2: the margin further from home shall increase proportionally to log(distance), where the slope is proportional to pressure
	// Everything at distance 2^(HIGH_PRECISION_RANGE/2+1) or closer falls into regime 1

	return margin * margin;
}

void Tracker::set_home(float x, float y, float z)
{
	home_position = to_ipos({ .x = x, .y = y, .z = z });

	// Reset the distance-to-home of all nodes to infinity (except the home node)
	for (size_t i = 1; i < node_count; i++) {
		node_at(0).distance = MAX_DISTANCE;
	}

	node_t &home_node = node_at(0);
	home_node.index1 = 0;
	home_node.index2 = 0;
	home_node.coef1 = 0;
	home_node.coef2 = 0;
	home_node.dirty = 1;
	home_node.obsolete = 0;
	home_node.use_line2 = 0;
	home_node.go_forward = 0;
	home_node.distance = 0;

	consolidate_graph("new home position");

	did_set_home = true;

	TRACKER_DBG("tracker received new home position %zu-%.2f (%.2f, %.2f, %.2f)", nodes->index1,
		    (double)coef_to_float(nodes->coef1), (double)x, (double)y, (double)z);
}

void Tracker::update(vehicle_local_position_s *position)
{
	if (_ref_timestamp == 0 && position->ref_timestamp != 0) {
		map_projection_init(&_ref_pos, position->ref_lat, position->ref_lon);
		_ref_alt = position->ref_alt;
		_ref_timestamp = position->ref_timestamp;

	} else if (_ref_timestamp != position->ref_timestamp) {
		// todo: implement changing reference
		return;
	}

	if (position->xy_valid && position->z_valid && _ref_timestamp != 0) {
		update(position->x, position->y, position->z);
	}
}

void Tracker::update(float x, float y, float z)
{
	last_known_position = {
		.x = x,
		.y = y,
		.z = z
	};

	if (recent_path_tracking_enabled) {
		push_recent_path(last_known_position);
	}

	if (graph_tracking_enabled) {
		push_graph(last_known_position);
	}

	if (!did_set_home) {
		set_home(x, y, z);
	}
}

void Tracker::push_recent_path(fpos_t &position)
{
	ipos_t new_pos = to_ipos(position);

	bool rollback = false;
	size_t index = recent_path_next_write;
	ipos_t head = recent_path_head;

	if (recent_path_next_read != RECENT_PATH_LENGTH) {

		// If we're still close to the most recent position, don't update
		if (dot(head, new_pos) <= 1) {
			return;
		}

		// Scan the buffer for a close position, starting at the latest one
		do {
			index = (index ? index : RECENT_PATH_LENGTH) - 1;

			if ((recent_path[index] >> 15) & 1) {
				// If this item is a bumper, we shouldn't roll over this invalid position (todo: use far jump element as bumper)
				break;
			}

			head -= unpack_compact_delta_item(recent_path[index]);
			rollback = dot(head, new_pos) <= 1;
		} while ((index != recent_path_next_read) && !rollback);

		// If the preceding item would be a bumper, we shouldn't roll back to this invalid position
		if (rollback && index != recent_path_next_read)
			if ((recent_path[(index ? index : RECENT_PATH_LENGTH) - 1] >> 15) & 1) {
				rollback = false;
			}
	}


	if (rollback) {

		// If there was a close position in the path, roll the path back to that position
		recent_path_head = head;

		if (index == recent_path_next_write) {
			TRACKER_DBG("recent path complete rollback");
			recent_path_next_write = 0;
			recent_path_next_read = RECENT_PATH_LENGTH;

		} else {
			TRACKER_DBG("recent path rollback to %zu", index);
			recent_path_next_write = index;
		}

	} else {

		// If there was no close position in the path, add the current position to the buffer
		// todo: handle large distances

		ipos_t old_head = recent_path_head;

		delta_item_t item = pack_compact_delta_item(new_pos - old_head);

		recent_path_head = new_pos;
		recent_path[recent_path_next_write] = item;

		if (recent_path_next_write++ == recent_path_next_read) {
			recent_path_next_read = recent_path_next_write;
		}

		if (recent_path_next_write == RECENT_PATH_LENGTH) {
			recent_path_next_write = 0;
		}

		if (recent_path_next_read == RECENT_PATH_LENGTH) {
			recent_path_next_read = 0;
		}
	}
}


bool Tracker::pop_recent_path(fpos_t &position)
{
	position = to_fpos(recent_path_head);

	// Check if path is empty
	if (recent_path_next_read == RECENT_PATH_LENGTH) {
		return false;
	}

	int last_index = (recent_path_next_write ? recent_path_next_write : RECENT_PATH_LENGTH) - 1;

	// Don't consume bumper item
	if ((recent_path[last_index] >> 15) & 1) {
		return false;
	}

	// Roll back most recent position
	recent_path_head -= unpack_compact_delta_item(recent_path[last_index]);

	if ((recent_path_next_write = last_index) == recent_path_next_read) {
		recent_path_next_write = 0;
		recent_path_next_read = RECENT_PATH_LENGTH;
	}

	return true;
}


bool Tracker::pop_recent_path(double &lat, double &lon, float &alt)
{
	fpos_t position;

	if (!pop_recent_path(position)) {
		return false;
	}

	map_projection_reproject(&_ref_pos, position.x, position.y, &lat, &lon);
	alt = _ref_alt - position.z;
	return true;
}


void Tracker::push_graph(fpos_t &position)
{
	ipos_t new_pos = to_ipos(position);

	// Don't update if we're still close to the last known line
	if (graph_next_write) {
		int coef;
		bool is_jump;
		ipos_t delta = get_point_to_line_delta(new_pos, fetch_delta(graph_next_write - 1, is_jump), graph_head_pos, coef);

		if (!is_jump && dot(delta) <= get_granularity_at(new_pos)) {
			return;
		}
	}

	TRACKER_DBG("tracker got pos %d %d %d", new_pos.x, new_pos.y, new_pos.z);

	// While there is not enough space left, increase memory pressure (at most 5 attempts)
	for (int i = 0; get_free_graph_space() < FAR_DELTA_SIZE * sizeof(delta_item_t); i++) {
		if (i >= 5 || memory_pressure == INT_MAX) {
			GRAPH_ERR("increasing memory pressure %d times (to %d) did not help", i, memory_pressure);
			return;
		}

		PX4_WARN("flight graph reached limit at memory pressure %d - flight graph will be optimized", memory_pressure);

		compress_perf_t *perf = memory_pressure > MAX_PERF_MEASUREMENTS ? NULL : &(perf_measurements[memory_pressure - 1] = {
			.runtime = 0,
			.deltas_before = graph_next_write,
			.nodes_before = node_count
		});

		memory_pressure++;

		size_t free_space_before = get_free_graph_space();

		hrt_abstime start_time = hrt_absolute_time();

		// Rewrite graph every other time the memory limit is reached
		if (memory_pressure & 1) {
			rewrite_graph(); // the rewrite pass includes consolidation

		} else {
			consolidated_head_pos = graph_start;
			consolidated_head_index = 0;
			consolidate_graph("reached memory limit");
		}

		start_time = hrt_absolute_time() - start_time;
		//TRACKER_DBG("time: %f", start_time / 1e6f);

		if (perf) {
			perf->runtime = start_time;
			perf->deltas_after = graph_next_write;
			perf->nodes_after = node_count;
		};

		(void)free_space_before;

		TRACKER_DBG("could reduce graph density from %.3f%% to %.3f%% in %f ms",
			    (double)100 * ((double)1 - (double)free_space_before / (GRAPH_LENGTH * sizeof(delta_item_t))),
			    (double)100 * ((double)1 - (double)get_free_graph_space() / (GRAPH_LENGTH * sizeof(delta_item_t))),
			    (double)(start_time / 1e3f));
	}


	if (!graph_next_write) {
		graph_head_pos = new_pos; // make sure that the very first delta is a compact delta
		graph_start = new_pos;
		consolidated_head_pos = new_pos;
		consolidated_head_index = 0;
	}

	push_delta(graph_next_write, new_pos - graph_head_pos, false);
	graph_head_pos = new_pos;

	if (graph_next_write - 1 - consolidated_head_index >= MAX_CONSOLIDATION_DEBT) {
		consolidate_graph("consolidation debt limit reached");
	}
}


Tracker::ipos_t Tracker::walk_forward(size_t &index, bool &is_jump)
{
	if ((graph[index + 1] >> 15) & 1) {
		return fetch_delta(index += FAR_DELTA_SIZE, is_jump);

	} else {
		return fetch_delta(index += 1, is_jump);
	}
}

Tracker::ipos_t Tracker::walk_backward(size_t &index, bool &is_jump)
{
	ipos_t delta = fetch_delta(index, is_jump);

	if ((graph[index] >> 15) & 1) {
		index -= FAR_DELTA_SIZE;

	} else {
		index -= 1;
	}

	return delta;
}


bool Tracker::push_node(node_t &node, int granularity)
{
	if (get_free_graph_space() < sizeof(node_t)) {
		return false;
	}

	float half_accuracy = fast_sqrt(granularity, false);

	// Check if there is already a nearby node that connects roughly the same lines
	for (size_t i = node_count - 1; i > 0; i--) {
		if (check_similarity(node.index1, node.coef1, node_at(i).index1, node_at(i).coef1, half_accuracy))
			if (check_similarity(node.index2, node.coef2, node_at(i).index2, node_at(i).coef2, half_accuracy)) {
				return false;
			}

		if (check_similarity(node.index1, node.coef1, node_at(i).index2, node_at(i).coef2, half_accuracy))
			if (check_similarity(node.index2, node.coef2, node_at(i).index1, node_at(i).coef1, half_accuracy)) {
				return false;
			}
	}

	node_at(node_count++) = node;
	have_dirty_nodes = true;
	return true; // todo: take some action if the buffer becomes full
}


void Tracker::remove_nodes(size_t lower_bound, size_t upper_bound)
{
	for (size_t i = 1; i < node_count; i++) {
		if ((lower_bound < node_at(i).index1 && node_at(i).index1 <= upper_bound) ||
		    (lower_bound < node_at(i).index2 && node_at(i).index2 <= upper_bound) ||
		    node_at(i).obsolete) {
			// move preceding part of the stack
			node_count--;
			memmove(nodes - node_count + 1, nodes - node_count, (node_count - i) * sizeof(node_t));
			i--;
		}
	}
}


bool Tracker::check_similarity(size_t index1, int coef1, size_t index2, int coef2, float max_distance)
{
	if (index1 > index2) {
		myswap(index1, index2);
		myswap(coef1, coef2);
	}

	if (index1 == index2 && coef1 == coef2) {
		return true;
	}

	float fcoef1 = coef_to_float(coef1);
	float fcoef2 = coef_to_float(coef2);

	bool is_jump;
	size_t index2_predecessor = index2;
	float delta_length = fast_sqrt(dot(walk_backward(index2_predecessor, is_jump)), false);

	if (is_jump && coef2 != MAX_COEFFICIENT) {
		return false;
	}

	if (index1 == index2) {
		if (delta_length * std::abs(fcoef1 - fcoef2) <= max_distance) {
			return true;
		}
	}

	float predecessor_delta_length = fast_sqrt(dot(fetch_delta(index2_predecessor, is_jump)), false);

	if (is_jump && coef1) {
		return false;
	}

	if (index1 == index2_predecessor)
		if (delta_length * (1 - fcoef2) + predecessor_delta_length * fcoef1 <= max_distance) {
			return true;
		}

	return false;
}


bool Tracker::is_close_to_graph(ipos_t position, size_t lower_bound, size_t upper_bound, ipos_t pos_at_upper_bound)
{
	int granularity = get_granularity_at(position);

	while (upper_bound > lower_bound && !graph_fault) {
		bool is_jump;
		ipos_t delta = walk_backward(upper_bound, is_jump);

		int coef;
		ipos_t pos_to_line = is_jump ? (position - pos_at_upper_bound) : get_point_to_line_delta(position, delta,
				     pos_at_upper_bound, coef);

		if (dot(pos_to_line) <= granularity) {
			return true;
		}

		pos_at_upper_bound -= delta;
	}

	return false;
}


Tracker::ipos_t Tracker::get_closest_position(ipos_t position, size_t &best_index, unsigned int &best_coef)
{
	int best_dist_squared = INT_MAX;
	ipos_t best_pos = { .x = 0, .y = 0, .z = 0 };
	best_index = 0;
	best_coef = 0;

	if (!graph_next_write) {
		return best_pos;
	}

	ipos_t pos = graph_head_pos;
	size_t index = graph_next_write - 1;
	size_t prev_index;
	int coef;

	do {
		prev_index = index;
		bool is_jump;
		ipos_t delta = walk_backward(index, is_jump);

		int dummy = 0;
		ipos_t line_to_pos = get_line_to_line_delta(delta, pos, { .x = 0, .y = 0, .z = 0 }, position, coef = 0, dummy, is_jump
				     || !prev_index, true);
		int distance_squared = dot(line_to_pos);

		if (best_dist_squared >= distance_squared) {
			best_dist_squared = distance_squared;
			best_pos = position - line_to_pos;
			best_index = prev_index;
			best_coef = coef;
		}

		pos -= delta;
	} while (prev_index && !graph_fault);

	return best_pos;
}


bool Tracker::is_line(ipos_t start_pos, size_t start_index, ipos_t end_pos, size_t end_index, bool should_be_jump)
{
	ipos_t line_delta = end_pos - start_pos;
	ipos_t pos = end_pos;

	// The efficiency gain of prefetching the granularity seems to outweigh the corner-cases there the granularity along the line should be finer than on both ends.
	int granularity = math::min(get_granularity_at(start_pos), get_granularity_at(end_pos));

	// Intuitively, it seems that walking backward allows us to break earlier than walking forward.

	for (;;) {
		bool is_jump;
		pos -= walk_backward(end_index, is_jump);

		if (graph_fault) {
			return false;
		}

		// The line must have a uniform jump property.
		if (is_jump != should_be_jump) {
			return false;
		}

		if (end_index <= start_index) {
			return true;
		}

		int coef;
		ipos_t delta = get_point_to_line_delta(pos, line_delta, end_pos, coef);

		if (dot(delta) > granularity) {
			return false;
		}
	}
}


void Tracker::get_longest_line(ipos_t start_pos, size_t start_index, ipos_t &end_pos, size_t &end_index, bool &is_jump,
			       size_t bound)
{
	ipos_t next_end_pos = start_pos;
	size_t next_end_index = start_index;
	bool next_is_jump = is_jump;

	do {
		end_index = next_end_index;
		end_pos = next_end_pos;
		is_jump = next_is_jump;
		next_end_pos += walk_forward(next_end_index, next_is_jump);
	} while (!graph_fault && next_end_index <= bound
		 && is_line(start_pos, start_index, next_end_pos, next_end_index, next_is_jump));
}


// Consolidates the most recent positions on the graph, by applying some optimizations:
//  1. Consecutive positions that lie roughly on a line are aggregated.
//  2. If multiple consecutive positions lie close to the graph, they are removed and replaced by a jump.
//  3. For lines that pass close to each other, a node is created.
// The entire tracker is guaranteed to remain consistent, however exposed indices may become invalid.
void Tracker::consolidate_graph(const char *reason)
{
	// Abort if the graph is empty
	if (!graph_next_write) {
		return;
	}

	TRACKER_DBG("consolidating graph from %zu to %zu: %s", consolidated_head_index, graph_next_write - 1, reason);
	DUMP_GRAPH();

	// Remove any nodes that refer to the area that will be modified.
	// Any nodes for that area will be newly created anyway.
	remove_nodes(consolidated_head_index, graph_next_write - 1);

	/*** Pass 1: detect lines ***/
	TRACKER_DBG("  line detection...");

	ipos_t pos = consolidated_head_pos;
	size_t read_index = consolidated_head_index;
	size_t write_index = read_index + 1;

	// read_index and (write_index - 1) always correspond to the same position.
	// They start out to be equal, and the gap between them increases as consolidation moves on.

	while (read_index < graph_next_write - 1 && !graph_fault) {
		bool is_jump;
		ipos_t line_end_pos;
		size_t line_end_index;
		get_longest_line(pos, read_index, line_end_pos, line_end_index, is_jump, graph_next_write - 1);

		// Push the line delta ONLY if it uses less space than the original deltas
		if (push_delta(write_index, line_end_pos - pos, is_jump, line_end_index - read_index)) {
			TRACKER_DBG("    aggregated %zu to %zu into line %zu", read_index, line_end_index, write_index - 1);
			//TRACKER_DBG("    from %d %d %d to %d %d %d, %s", pos.x, pos.y, pos.z, line_end_pos.x, line_end_pos.y, line_end_pos.z, is_jump ? "is jump" : "no jump");
			read_index = line_end_index;
		}

		// Copy the deltas that were not aggregated into a line
		while (read_index < line_end_index) {
			push_delta(write_index, walk_forward(read_index, is_jump), is_jump);
		}

		pos = line_end_pos;
	}

	graph_next_write = write_index;


	/*** Pass 2: remove positions that are already in the graph ***/
	TRACKER_DBG("  redundancy removal...");

	pos = consolidated_head_pos;
	read_index = consolidated_head_index;
	write_index = read_index + 1;

	// Keep track of the index/position from where we may overwrite previously copied deltas.
	size_t overwrite_index = write_index;
	ipos_t pos_at_overwrite_index = pos;
	bool was_close = false;

	while (read_index < graph_next_write - 1 && !graph_fault) {
		bool is_jump;
		ipos_t prev_pos = pos;
		ipos_t delta = walk_forward(read_index, is_jump);
		pos += delta;

		// We copy each position optimistically, even if it turns out that it will be deleted later on.
		size_t prev_write_index = write_index;
		push_delta(write_index, delta, is_jump);

		// Scan the graph to see if any line is close to the current position
		bool is_close = is_close_to_graph(pos, overwrite_index > GRAPH_SEARCH_RANGE ? overwrite_index - GRAPH_SEARCH_RANGE : 0,
						  overwrite_index - 1, pos_at_overwrite_index);

		// If the last position wasn't close to the graph, we must definitely keep this one.
		if (!was_close) {
			overwrite_index = write_index;
			pos_at_overwrite_index = pos;
		}

		// As soon as we encounter a position which is not close to the graph, we see if we can remove some preceeding positions
		if (!is_close) {

			// If the last few positions were close to the graph, replace them by a jump
			if (overwrite_index < prev_write_index) {
				size_t max_space = prev_write_index - overwrite_index;

				if (push_delta(overwrite_index, prev_pos - pos_at_overwrite_index, true, max_space)) {
					// We append the last position once again and go on from there
					push_delta(write_index = overwrite_index, delta, is_jump);

					TRACKER_DBG("    replaced redundant path of size %zu by a jump at %zu", max_space, overwrite_index - 1);
				}
			}

			// If this branch was taken, the last important position will be re-initialized once it becomes relevant
		}

		was_close = is_close;
	}

	graph_next_write = write_index;


	/*** Pass 3: detect nodes ***/
	TRACKER_DBG("  node detection...");

	pos = consolidated_head_pos;
	read_index = consolidated_head_index;

	while (read_index < graph_next_write - 1 && !graph_fault) {

		ipos_t inner_pos = pos;
		size_t search_index = read_index;
		bool inhibit_node = true;

		bool is_jump;
		ipos_t delta = walk_forward(read_index, is_jump);
		pos += delta;

		int granularity = get_granularity_at(pos);
		size_t search_bound = read_index > GRAPH_SEARCH_RANGE ? read_index - GRAPH_SEARCH_RANGE : 0;

		while (search_index > search_bound && !graph_fault) {

			// Prepare a node in case we need it
			node_t node = {
				.index1 = (uint16_t)read_index,
				.index2 = (uint16_t)search_index,
				.dirty = 1,
				.coef1 = 0,
				.obsolete = 0,
				.coef2 = 0,
				.delta = pack_compact_delta_item({ .x = 0, .y = 0, .z = 0 }),
				.use_line2 = 0,
				.go_forward = 0,
				.distance = MAX_DISTANCE
			};

			bool is_inner_jump;
			int coef1, coef2;
			//TRACKER_DBG("  index %zu is at %d %d %d", search_index, inner_pos.x, inner_pos.y, inner_pos.z);
			ipos_t inner_delta = walk_backward(search_index, is_inner_jump);

			// We don't want to create a node to the line that directly precedes the current one
			if (!inhibit_node) {
				ipos_t line_to_line = get_line_to_line_delta(delta, pos, inner_delta, inner_pos, coef1, coef2, is_jump, is_inner_jump);

				if (dot(line_to_line) <= granularity) {
					// If the node would be at the beginning of the line, we defer the node creation to the next line (unless we're about to terminate the search)
					if (coef1 < MAX_COEFFICIENT || search_index <= search_bound) {

						if (coef1 < 0 || coef1 > MAX_COEFFICIENT || coef2 < 0 || coef2 > MAX_COEFFICIENT) {
							GRAPH_ERR("delta coefficient out of range: have %d and %d", coef1, coef2);
							return;
						}

						node.coef1 = coef1;
						node.coef2 = coef2;
						node.delta = pack_compact_delta_item(line_to_line);

						//TRACKER_DBG("  line to line delta (%d %d %d)...(%d %d %d) to (%d %d %d)...(%d %d %d) is (%d %d %d), coef is %d, %d",
						//    delta.x, delta.y, delta.z, pos.x, pos.y, pos.z,
						//    inner_delta.x, inner_delta.y, inner_delta.z, inner_pos.x, inner_pos.y, inner_pos.z,
						//    line_to_line.x, line_to_line.y, line_to_line.z,
						//    coef1, coef2);

						TRACKER_DBG("    create node from %zu-%.2f to %zu-%.2f", (size_t)node.index1, (double)coef_to_float(node.coef1),
							    (size_t)node.index2, (double)coef_to_float(node.coef2));

						push_node(node, granularity);
					}
				}
			}

			inhibit_node = false;
			inner_pos -= inner_delta;
		}
	}


	// Finalize the consolidation
	if (consolidated_head_index < pinned_index) {
		graph_version++;
		pinned_index = 0;
	}

	consolidated_head_pos = graph_head_pos;
	consolidated_head_index = graph_next_write - 1;


	// Sanitize home position
	size_t home_index;
	unsigned int home_coef;
	home_on_graph = get_closest_position(home_position, home_index, home_coef);

	if (nodes->index1 != home_index || nodes->coef1 != home_coef) {
		nodes->index1 = home_index;
		nodes->coef1 = home_coef;
		have_dirty_nodes = true;
	}

	TRACKER_DBG("consolidation complete, graph reduced to %zu", graph_next_write - 1);
	DUMP_GRAPH();
}


float Tracker::measure_distance(size_t index1, int coef1, size_t index2, int coef2)
{
	// make sure position 1 is before position 2
	if (index1 > index2 || (index1 == index2 && coef1 < coef2)) {
		myswap(index1, index2);
		myswap(coef1, coef2);
	}


	bool is_jump;
	ipos_t delta = fetch_delta(index1, is_jump);
	float coef1_to_line_end = fast_sqrt(dot(apply_coef(delta, coef1)), false);

	// Walk along the interval to measure distance
	float distance = 0;

	while (index1 < index2) {
		delta = walk_forward(index1, is_jump);
		distance += fast_sqrt(dot(delta), false);

		if (is_jump) {
			return INFINITY;
		}
	}

	float coef2_to_line_end = fast_sqrt(dot(apply_coef(delta, coef2)), false);
	return coef1_to_line_end + distance - coef2_to_line_end;
}


bool Tracker::walk_to_node(size_t &index, int &coef, float &distance, bool forward, size_t search_bound)
{
	// Look through all nodes to find the closest one that lies in the direction of travel
	size_t best_index = search_bound;
	int best_coef = 0;
	bool have_node = false;

	for (size_t i = 0; i < node_count; i++) {
		if (forward) {
			if ((node_at(i).index1 < best_index || (node_at(i).index1 == best_index && node_at(i).coef1 >= best_coef)) &&
			    (node_at(i).index1 > index || (node_at(i).index1 == index && node_at(i).coef1 < coef))) {
				best_index = node_at(i).index1;
				best_coef = node_at(i).coef1;
				have_node = true;
			}

			if (i && (node_at(i).index2 < best_index || (node_at(i).index2 == best_index && node_at(i).coef2 >= best_coef)) &&
			    (node_at(i).index2 > index || (node_at(i).index2 == index && node_at(i).coef2 < coef))) {
				best_index = node_at(i).index2;
				best_coef = node_at(i).coef2;
				have_node = true;
			}

		} else {
			if ((node_at(i).index1 > best_index || (node_at(i).index1 == best_index && node_at(i).coef1 <= best_coef)) &&
			    (node_at(i).index1 < index || (node_at(i).index1 == index && node_at(i).coef1 > coef))) {
				best_index = node_at(i).index1;
				best_coef = node_at(i).coef1;
				have_node = true;
			}

			if (i && (node_at(i).index2 > best_index || (node_at(i).index2 == best_index && node_at(i).coef2 <= best_coef)) &&
			    (node_at(i).index2 < index || (node_at(i).index2 == index && node_at(i).coef2 > coef))) {
				best_index = node_at(i).index2;
				best_coef = node_at(i).coef2;
				have_node = true;
			}
		}
	}

	if (!have_node) {
		return false;
	}

	distance = measure_distance(index, coef, best_index, best_coef);

	index = best_index;
	coef = best_coef;

	return !isinf(distance);
}


float Tracker::apply_node_delta(size_t &index, unsigned int &coef, ipos_t *delta, bool &go_forward)
{
	float best_distance = INFINITY;
	size_t best_index = index;
	int best_coef = coef;

	if (delta)
		*delta = { .x = 0, .y = 0, .z = 0 };

	// Consider all intersections at the specified position and use the one with the smallest distance home.
	for (size_t i = 0; i < node_count; i++) {
		if ((node_at(i).index1 == index && node_at(i).coef1 == coef) || (node_at(i).index2 == index
				&& node_at(i).coef2 == coef)) {
			float distance = node_at(i).distance == MAX_DISTANCE ? FLT_MAX : (float)node_at(i).distance;

			// If the node is far from home, we arbitrarily prefer the line with the lowest index.
			// This is in line with the decision that we prefer to walk backward. Thus, in case of
			// unknown distances, at least we strictly minimize the index and don't get stuck in an infinite loop.
			if (distance >= FLT_MAX) {
				node_at(i).use_line2 = node_at(i).index2 < node_at(i).index1;
			}

			if (best_distance > distance) {
				best_distance = distance;
				best_index = node_at(i).use_line2 ? node_at(i).index2 : node_at(i).index1;
				best_coef = node_at(i).use_line2 ? node_at(i).coef2 : node_at(i).coef1;
				go_forward = node_at(i).go_forward;

				if (delta) {
					// If we don't change lines, the delta is 0
					if (index == (node_at(i).use_line2 ? node_at(i).index2 : node_at(i).index1))
						*delta = { .x = 0, .y = 0, .z = 0 };
					else if (node_at(i).use_line2) {
						*delta = unpack_compact_delta_item(node_at(i).delta);

					} else {
						*delta = -unpack_compact_delta_item(node_at(i).delta);
					}
				}
			}
		}
	}

	index = best_index;
	coef = best_coef;
	return best_distance;
}


float Tracker::get_node_distance(size_t index, unsigned int coef, int &direction)
{
	size_t old_index = index;
	unsigned int old_coef = coef;
	bool go_forward;
	float distance = apply_node_delta(index, coef, NULL, go_forward);
	direction = (index == old_index && coef == old_coef && !isinf(distance)) ? go_forward ? 1 : -1 : 0;
	return distance;
}


bool Tracker::set_node_distance(size_t index, int coef, float distance, bool go_forward)
{
	bool improvement = false;

	// Update all intersections at the specified position (if the new distance is an improvement)
	for (size_t i = 0; i < node_count; i++) {
		if ((node_at(i).index1 == index && node_at(i).coef1 == coef) || (node_at(i).index2 == index
				&& node_at(i).coef2 == coef)) {
			float new_distance = distance + fast_sqrt(dot(unpack_compact_delta_item(node_at(i).delta)), false);
			int int_distance = new_distance > MAX_DISTANCE ? MAX_DISTANCE : ceil(new_distance);

			if (node_at(i).distance > int_distance) {
				TRACKER_DBG("  distance at %zu, improved from %.1f to %d (%.2f) (go %s from %zu-%.2f)", i,
					    node_at(i).distance >= MAX_DISTANCE ? (double)INFINITY : (double)node_at(i).distance, int_distance, (double)distance,
					    go_forward ? "forward" : "backward", index, (double)coef_to_float(coef));

				node_at(i).distance = int_distance;
				node_at(i).use_line2 = index == node_at(i).index2 && coef == node_at(i).coef2;
				node_at(i).go_forward = go_forward;
				node_at(i).dirty = true;
				improvement = true;
				have_dirty_nodes = true;
			}
		}
	}

	return improvement;
}

void Tracker::refresh_distances()
{
	if (!graph_next_write) {
		return;
	}

	// Only log if we're gonna do something
	bool should_log = have_dirty_nodes;

	if (should_log) {
		TRACKER_DBG("calculating distances...");
	}

	// This loop is guaranteed to terminate.
	// Proof outline:
	//  The loop terminates when the set of dirty nodes is empty at the end of an iteration. This set is
	//  cleared at the beginning of every iteration. A node only gets added to this set, if its distance-
	//  to-home improves. A node distance cannot be increased and there are no intervals with negative
	//  length, so every node distance will eventually reach a lower bound. When this state is reached,
	//  the set of dirty nodes remains empty during an entire iteration, thus the loop terminates.

	while (have_dirty_nodes && !graph_fault) {
		have_dirty_nodes = false;

		for (size_t i = 0; i < node_count; i++) {
			// At every dirty node, we look in all four directions of the intersection.
			if (node_at(i).dirty) {
				node_at(i).dirty = 0;

				node_t node = node_at(i);

				size_t index;
				int coef;
				float interval_length;

				int direction_from_here, direction_from_there;
				float distance_here = get_node_distance(node.index1, node.coef1, direction_from_here);
				float distance_there;

				// Note that for instance if the current node has the instruction to walk forward, we certainly don't
				// want it to be the destination of the next node in that direction (this could happen due to
				// discretization of distances). That's why we respect the direction_from_here and direction_from_there.

				// Walk forward from index1
				if (walk_to_node(index = node.index1, coef = node.coef1, interval_length = 0, true, graph_next_write - 1)) {
					distance_there = get_node_distance(index, coef, direction_from_there);

					if (direction_from_here != 1) {
						set_node_distance(index, coef, distance_here + interval_length, false);
					}

					if (direction_from_there != -1) {
						set_node_distance(node.index1, node.coef1, distance_there + interval_length, true);
					}
				}

				// Walk backward from index1
				if (walk_to_node(index = node.index1, coef = node.coef1, interval_length = 0, false, 0)) {
					distance_there = get_node_distance(index, coef, direction_from_there);

					if (direction_from_here != -1) {
						set_node_distance(index, coef, distance_here + interval_length, true);
					}

					if (direction_from_there != 1) {
						set_node_distance(node.index1, node.coef1, distance_there + interval_length, false);
					}
				}

				// Ignore index2 of home node
				if (!i) {
					continue;
				}

				distance_here = get_node_distance(node.index2, node.coef2, direction_from_here);

				// Walk forward from index2
				if (walk_to_node(index = node.index2, coef = node.coef2, interval_length = 0, true, graph_next_write - 1)) {
					distance_there = get_node_distance(index, coef, direction_from_there);

					if (direction_from_here != 1) {
						set_node_distance(index, coef, distance_here + interval_length, false);
					}

					if (direction_from_there != -1) {
						set_node_distance(node.index2, node.coef2, distance_there + interval_length, true);
					}
				}

				// Walk backward from index2
				if (walk_to_node(index = node.index2, coef = node.coef2, interval_length = 0, false, 0)) {
					distance_there = get_node_distance(index, coef, direction_from_there);

					if (direction_from_here != -1) {
						set_node_distance(index, coef, distance_here + interval_length, true);
					}

					if (direction_from_there != 1) {
						set_node_distance(node.index2, node.coef2, distance_there + interval_length, false);
					}
				}
			}
		}
	}

#ifdef DEBUG_TRACKER

	if (should_log) {
		dump_nodes();
	}

#endif
}


bool Tracker::calc_return_path(path_finding_context_t &context, bool &progress)
{
	// Before making any decision, make sure we know the distance of every node to home.
	refresh_distances();

	TRACKER_DBG("calculate return path from %zu-%.2f (%d %d %d) to (%d %d %d)", context.current_index,
		    (double)coef_to_float(context.current_coef),
		    context.current_pos.x, context.current_pos.y, context.current_pos.z,
		    home_on_graph.x, home_on_graph.y, home_on_graph.z);

	progress = false;

	ipos_t delta = { .x = 0, .y = 0, .z = 0 };

	// If we reached the checkpoint, re-evaluate next checkpoint and potentially switch lines.
	if (context.current_index == context.checkpoint_index && context.current_coef == context.checkpoint_coef) {
		TRACKER_DBG("checkpoint!");

		// This check is no longer neccessary
		if (context.checkpoint_index == nodes->index1 && context.checkpoint_coef == nodes->coef1) {
			return false;
		}

		// If we're at a node (which doesn't have to be the case), this will switch to the line that gives the best path home.
		bool go_forward;
		float dist_through_node = apply_node_delta(context.current_index, context.current_coef, &delta, go_forward);

		// If we're at a node, we must proceed in the recommended direction, otherwise we risk going in the
		// wrong direction (in the case of equal cost, due to rounding) and ending up in an infinite loop.
		// Also, we don't want to arrive at a node exactly from the direction where it recommends to go.

		int direction = 0;

		// Determine cost and bound of walking forward (except if the current node recommended to go backward)
		float forward_dist;
		size_t forward_node_index = context.current_index;
		int forward_node_coef = context.current_coef;

		if ((isinf(dist_through_node) || go_forward)
		    && walk_to_node(forward_node_index, forward_node_coef, forward_dist = 0, true, graph_next_write - 1)) {
			forward_dist += get_node_distance(forward_node_index, forward_node_coef, direction);

		} else {
			forward_dist = INFINITY;
		}

		if (direction == -1) {
			forward_dist = INFINITY;
		}

		// Determine cost and bound of walking backward (except if the current node recommended to go forward)
		float backward_dist;
		size_t backward_node_index = context.current_index;
		int backward_node_coef = context.current_coef;

		if ((isinf(dist_through_node) || !go_forward)
		    && walk_to_node(backward_node_index, backward_node_coef, backward_dist = 0, false, 0)) {
			backward_dist += get_node_distance(backward_node_index, backward_node_coef, direction);

		} else {
			backward_dist = INFINITY;
		}

		if (direction == 1) {
			backward_dist = INFINITY;
		}

		TRACKER_DBG("distance from %zu-%.2f is %f forward (to %zu-%.2f) and %f backward (to %zu-%.2f)", context.current_index,
			    (double)coef_to_float(context.current_coef), (double)forward_dist, forward_node_index,
			    (double)coef_to_float(forward_node_coef), (double)backward_dist, backward_node_index,
			    (double)coef_to_float(backward_node_coef));

		// Abort if we don't know what to do
		if (isinf(backward_dist) && forward_dist >= FLT_MAX) {
			PX4_WARN("Could not find any path home from %zu-%.2f.", context.current_index,
				 (double)coef_to_float(context.current_coef));
			return false;
		}

		// Note that we prefer to walk backward
		if (forward_dist < backward_dist) {
			context.checkpoint_index = forward_node_index;
			context.checkpoint_coef = forward_node_coef;

		} else {
			context.checkpoint_index = backward_node_index;
			context.checkpoint_coef = backward_node_coef;
		}

		pin_index(context.current_index);
		pin_index(context.checkpoint_index);
	}

	// If the (potential) node switch above did not yield a position update yet, advance in the direction of the next checkpoint
	if (!(delta.x || delta.y || delta.z)) {
		bool is_jump;
		ipos_t current_delta = fetch_delta(context.current_index, is_jump);
		delta = apply_coef(current_delta, context.current_coef);

		if (context.current_index < context.checkpoint_index) {
			current_delta = walk_forward(context.current_index, is_jump);

			if (is_jump) {
				PX4_WARN("path finding algorithm is moving forward along jump delta");
			}

			context.current_coef = MAX_COEFFICIENT + 1;

		} else if (context.current_index > context.checkpoint_index) {
			delta -= current_delta;

			if (is_jump) {
				PX4_WARN("path finding algorithm is moving backward along jump delta");
			}

			current_delta = walk_backward(context.current_index, is_jump);
			context.current_coef = 0;

		} else {
			if (is_jump) {
				PX4_WARN("path finding algorithm is moving along jump delta");
			}

			delta -= apply_coef(current_delta, context.checkpoint_coef);
			context.current_coef = context.checkpoint_coef;
		}

		pin_index(context.current_index);

		//TRACKER_DBG("changing position by (%d %d %d)", delta.x, delta.y, delta.z);
	}

	context.current_pos += delta;
	progress = delta.x || delta.y || delta.z;

	return true;
}


void Tracker::mark_obsolete(size_t index, int coef, bool forward, bool backward, size_t retain, size_t lower_bound,
			    size_t upper_bound)
{
	size_t start_index = index;
	int start_coef = coef;

	TRACKER_DBG("  mark as obsolete from %zu-%.2f %s", index, (double)coef_to_float(coef),
		    forward ? backward ? "both dirs" : "forward" : backward ? "backward" : "nothing");

	bool is_jump;

	size_t prev_index = index - (((graph[index] >> 15) & 1) ? FAR_DELTA_SIZE : 1);

	while (index < upper_bound && forward && !graph_fault) {
		forward = graph[index] != OBSOLETE_DELTA && index != retain;
		bool keep = index == start_index || !forward;

		// Keep delta if any nodes lie on it.
		// If one is even in the right direction, finish walk.
		for (size_t i = 0; i < node_count && forward; i++) {
			if (!node_at(i).obsolete) {
				keep |= node_at(i).index1 == index || (i && node_at(i).index2 == index);

				if ((node_at(i).index1 == index && node_at(i).coef1 <= coef)
				    || (i && node_at(i).index2 == index && node_at(i).coef2 <= coef)) {
					forward = false;
				}
			}
		}

		while (!keep && prev_index++ < index) {
			graph[prev_index] = OBSOLETE_DELTA;
		}

		prev_index = index;
		walk_forward(index, is_jump);
		coef = MAX_COEFFICIENT + 1;
	}


	index = start_index;
	coef = start_coef;

	while (index > lower_bound && backward && !graph_fault) {
		backward = graph[index] != OBSOLETE_DELTA && index != retain;
		bool keep = index == start_index || !backward;

		// See if any node lies on this delta.
		// If there are nodes, but they don't lie in the considered direction, we just keep this delta but continue the walk.
		for (size_t i = 0; i < node_count && backward; i++) {
			if (!node_at(i).obsolete) {
				keep |= node_at(i).index1 == index || (i && node_at(i).index2 == index);

				if ((node_at(i).index1 == index && node_at(i).coef1 >= coef)
				    || (i && node_at(i).index2 == index && node_at(i).coef2 >= coef)) {
					backward = false;
				}
			}
		}

		prev_index = index;
		walk_backward(index, is_jump);
		coef = 0;

		while (!keep && prev_index-- > index) {
			graph[prev_index + 1] = OBSOLETE_DELTA;
		}
	}
}


void Tracker::rewrite_graph()
{
	if (!graph_next_write) {
		return;
	}

	consolidate_graph("graph rewrite requested");

	TRACKER_DBG("rewriting graph...");

	// The cache helps us in case there is no space at all for the rewrite area
	delta_item_t rewrite_cache[REWRITE_CACHE_SIZE];
	size_t cache_size = 0;

	// Init rewrite area at the end of the node buffer
	size_t next_rewrite_index = GRAPH_LENGTH - node_count * sizeof(node_t) / sizeof(delta_item_t) - 1;

	// Init path finding context, using the graph head as starting point
	path_finding_context_t context = {
		.current_pos = graph_head_pos,
		.current_index = graph_next_write - 1,
		.current_coef = 0,
		.checkpoint_index = 0,
		.checkpoint_coef = 0,
		.graph_version = graph_version
	};
	context.checkpoint_index = context.current_index;
	context.checkpoint_coef = context.current_coef;

	bool not_home = true;

	// Walk the shortest path home and reconstruct it at the end of the graph buffer.
	while (not_home && !graph_fault) {

		size_t prev_checkpoint_index = context.checkpoint_index;
		int prev_checkpoint_coef = context.checkpoint_coef;
		bool at_checkpoint = context.current_index == context.checkpoint_index
				     && context.current_coef == context.checkpoint_coef;

		// Proceed on the return path
		bool progress = false;
		ipos_t prev_pos = context.current_pos;
		not_home = calc_return_path(context, progress);

		// Mark as obsolete the direction in which we're not going
		bool going_forward = context.current_index < context.checkpoint_index
				     || (context.current_index == context.checkpoint_index && context.current_coef > context.checkpoint_coef);
		bool going_backward = context.current_index > context.checkpoint_index
				      || (context.current_index == context.checkpoint_index && context.current_coef < context.checkpoint_coef);
		mark_obsolete(context.current_index, context.current_coef, going_backward, going_forward, 0, 0, graph_next_write);

		// At every checkpoint (usually a node), mark all irrelevant intervals as obsolete
		if (at_checkpoint) {
			for (size_t i = 1; i < node_count; i++) {
				bool come_from_line1 = node_at(i).index1 == prev_checkpoint_index && node_at(i).coef1 == prev_checkpoint_coef;
				bool come_from_line2 = node_at(i).index2 == prev_checkpoint_index && node_at(i).coef2 == prev_checkpoint_coef;

				if (come_from_line1 || come_from_line2) {
					bool use_line2 = node_at(i).use_line2;
					node_at(i).obsolete = 1;

					TRACKER_DBG("  node %zu is obsolete and I'm going %s on line %d", i,
						    going_forward ? "forward" : going_backward ? "backward" : "nowhere", use_line2 ? 2 : 1);

					// Remove intervals from line1
					mark_obsolete(node_at(i).index1, node_at(i).coef1,
						      use_line2 || going_backward,
						      use_line2 || going_forward,
						      node_at(i).index2, 0, graph_next_write);

					// Remove intervals from line2
					mark_obsolete(node_at(i).index2, node_at(i).coef2,
						      !use_line2 || going_backward,
						      !use_line2 || going_forward,
						      node_at(i).index1, 0, graph_next_write);
				}
			}
		}

		bool cache_full = false;

		// Push the negative delta to the cache (when flushing, the deltas will be reversed)
		TRACKER_DBG("  advance from %d %d %d to %d %d %d", prev_pos.x, prev_pos.y, prev_pos.z, context.current_pos.x,
			    context.current_pos.y, context.current_pos.z);

		if (progress) {
			cache_full = !push_delta(cache_size, prev_pos - context.current_pos, false, REWRITE_CACHE_SIZE - cache_size,
						 rewrite_cache);
		}


		// Once in a while, flush the cache to the rewrite area
		// todo: recall the reasoning here: why would multiple cache flushes make sense?
		while ((!not_home || cache_full) && cache_size && !graph_fault) {
			TRACKER_DBG("  flush rewrite cache of size %zu", cache_size);

			// If the cache cannot be flushed, reclaim space used by obsolete stuff
			if (next_rewrite_index < graph_next_write - 1 + cache_size) {
				TRACKER_DBG("  reclaim space");

				// Remove obsolete nodes
				size_t old_node_count = node_count;
				remove_nodes(0, 0);

				// Move rewrite area to the end of the available space
				size_t yield = (old_node_count - node_count) * sizeof(node_t) / sizeof(delta_item_t);
				size_t rewrite_length = GRAPH_LENGTH - old_node_count * sizeof(node_t) / sizeof(delta_item_t) - next_rewrite_index - 1;
				memmove(graph + next_rewrite_index + 1 + yield, graph + next_rewrite_index + 1, rewrite_length * sizeof(delta_item_t));
				next_rewrite_index += yield;

				// Remove obsolete delta elements
				size_t copy_destination = 0;
				size_t copy_origin = 0;
				size_t copy_length = 0;

				for (size_t i = 0; i <= graph_next_write; i++) {
					if (i == graph_next_write ? true : (graph[i] == OBSOLETE_DELTA)) {
						if (copy_length) {
							// Update indices that point to shifted deltas
							context.current_index -= context.current_index < copy_origin ? 0 : copy_origin - copy_destination;
							context.checkpoint_index -= context.checkpoint_index < copy_origin ? 0 : copy_origin - copy_destination;

							for (size_t j = 0; j < node_count; j++) {
								node_at(j).index1 -= node_at(j).index1 < copy_origin ? 0 : copy_origin - copy_destination;
								node_at(j).index2 -= node_at(j).index2 < copy_origin ? 0 : copy_origin - copy_destination;
							}

							memmove(graph + copy_destination, graph + copy_origin, copy_length * sizeof(delta_item_t));
							copy_destination += copy_length;
							copy_length = 0;
						}

						copy_origin = i + 1;

					} else {
						copy_length++;
					}
				}

				TRACKER_DBG("  reclaimed memory of %zu elements", yield + graph_next_write - copy_destination);
				graph_next_write = copy_destination;
			}


			// Flush rewrite cache to rewrite area (as much as possible)
			size_t copy_count = math::min(cache_size, next_rewrite_index - graph_next_write + 1);

			for (size_t i = 0; i < copy_count; i++) {
				if (!next_rewrite_index) {
					GRAPH_ERR("rewrite area exceeded graph buffer size");
					return;
				}

				if ((rewrite_cache[i] >> 15) & 1) {
					graph[next_rewrite_index--] = rewrite_cache[i + 2];
					graph[next_rewrite_index--] = rewrite_cache[i + 1];
					graph[next_rewrite_index--] = rewrite_cache[i];
					i += 2;

				} else {
					graph[next_rewrite_index--] = rewrite_cache[i];
				}
			}

			memmove(rewrite_cache, rewrite_cache + copy_count, (cache_size -= copy_count) * sizeof(delta_item_t));
		}

		// If the delta push failed previously, it should work now that we have flushed the cache
		if (cache_full) {
			if (!push_delta(cache_size, prev_pos - context.current_pos, false, REWRITE_CACHE_SIZE - cache_size, rewrite_cache)) {
				GRAPH_ERR("no space in graph rewrite cache after flushing");
				return;
			}

			not_home = true; // just in case, pretend we're not yet home, so we get get another chance to flush the cache
		}
	}


	// Move rebuilt path to the beginning of the buffer.
	size_t rewrite_length = GRAPH_LENGTH - node_count * sizeof(node_t) / sizeof(delta_item_t) - next_rewrite_index - 1;
	memmove(graph + 1, graph + next_rewrite_index + 1, rewrite_length * sizeof(delta_item_t));

	// Finalize rewrite
	node_count = 1;
	graph_next_write = rewrite_length + 1;
	consolidated_head_index = 0;
	graph_version++;
	pinned_index = 0;

	DUMP_GRAPH();
	TRACKER_DBG("graph rewrite complete!");
}


void Tracker::reset_graph()
{
	TRACKER_DBG("deleting graph of length %zu", graph_next_write);
	node_count = 1;
	graph_next_write = 0;
	consolidated_head_index = 0;
	graph_version++;
	pinned_index = 0;
	memory_pressure = 1;
	graph_fault = false;
}


bool Tracker::init_return_path(path_finding_context_t &context, float &x, float &y, float &z)
{
	// We prefer that the index we're about to expose remains valid, so we consolidate the graph first
	consolidate_graph("init return path");

	if (!graph_next_write) {
		return false;
	}

	// Use graph head as starting point
	context = {
		.current_pos = graph_head_pos,
		.current_index = graph_next_write - 1,
		.current_coef = 0,
		.checkpoint_index = 0,
		.checkpoint_coef = 0,
		.graph_version = graph_version
	};

	context.checkpoint_index = context.current_index;
	context.checkpoint_coef = context.current_coef;

	pin_index(context.current_index);

	x = context.current_pos.x;
	y = context.current_pos.y;
	z = context.current_pos.z;

	return true;
}

bool Tracker::init_return_path_global(path_finding_context_t &context, double &lat, double &lon, float &alt)
{
	float x, y, z;
	bool result = init_return_path(context, x, y, z);
	map_projection_reproject(&_ref_pos, x, y, &lat, &lon);
	alt = _ref_alt - z;
	return result;
}

bool Tracker::advance_return_path(path_finding_context_t &context, float &x, float &y, float &z)
{
	// If the graph version changed, we need to sanitize the path finding context
	if (context.graph_version != graph_version) {
		consolidate_graph("advance return path with invalid index"); // We need to make sure that consolidation is complete before continuing
		context.current_pos = get_closest_position(context.current_pos, context.current_index, context.current_coef);
		context.checkpoint_index = context.current_index;
		context.checkpoint_coef = context.current_coef;
		pin_index(context.current_index);
	}

	bool progress = false;
	bool result = calc_return_path(context, progress);

	// We give a second chance to make progress:
	// It may happen for example that we switch from the end of one line to the beginning of the next line, but the position stays unchanged.
	if (result && !progress) {
		TRACKER_DBG("return path made no progress - second chance");
		result = calc_return_path(context, progress);
	}

	x = context.current_pos.x;
	y = context.current_pos.y;
	z = context.current_pos.z;

	return result;
}

bool Tracker::advance_return_path_global(path_finding_context_t &context, double &lat, double &lon, float &alt)
{
	float x, y, z;
	bool result = advance_return_path(context, x, y, z);
	map_projection_reproject(&_ref_pos, x, y, &lat, &lon);
	alt = _ref_alt - z;
	return result;
}

bool Tracker::is_context_close_to_head(path_finding_context_t &context)
{
	ipos_t delta = context.current_pos - to_ipos(last_known_position);
	/*TRACKER_DBG("context (%d %d %d) to head (%d %d %d) is sqrt(%d)",
	    context.current_pos.x, context.current_pos.y, context.current_pos.z,
	    graph_head_pos.x, graph_head_pos.y, graph_head_pos.z,
	    dot(delta));*/
	return dot(delta) <= get_granularity_at(context.current_pos);
}

bool Tracker::is_context_close_to_home(path_finding_context_t &context)
{
	ipos_t delta = context.current_pos - home_on_graph;
	return dot(delta) <= get_granularity_at(context.current_pos);
}

bool Tracker::is_same_pos(path_finding_context_t &context1, path_finding_context_t &context2)
{
	return context1.current_index == context2.current_index && context1.current_coef == context2.current_coef;
}


void Tracker::dump_recent_path()
{
	if (recent_path_next_read == RECENT_PATH_LENGTH) {
		PX4_INFO("recent path empty");
		return;
	}

	PX4_INFO("recent path:");
	size_t recent_path_length = (recent_path_next_write + RECENT_PATH_LENGTH - recent_path_next_read - 1) %
				    RECENT_PATH_LENGTH + 1;
	PX4_INFO("  length: %zu (read-ptr: %zu, write-ptr: %zu)", recent_path_length, recent_path_next_read,
		 recent_path_next_write);

	size_t index = recent_path_next_write;
	ipos_t head = recent_path_head;

	do {
		PX4_INFO("  (%d, %d, %d)", head.x, head.y, head.z);

		index = (index ? index : RECENT_PATH_LENGTH) - 1;

		if ((recent_path[index] >> 15) & 1) {
			PX4_INFO("ignore the last position");
		}

		head -= unpack_compact_delta_item(recent_path[index]);
	} while (index != recent_path_next_read);
}


void Tracker::dump_graph()
{
	if (!graph_next_write) {
		PX4_INFO("full path empty");
		return;
	}

	PX4_INFO("full path (%zu elements of size %zu bytes, %.2f%% full, pressure %d, tracker overhead is %zu bytes):",
		 graph_next_write, (sizeof(delta_item_t) * CHAR_BIT) / 8,
		 (double)100.0 * ((double)1 - (double)get_free_graph_space() / (GRAPH_LENGTH * sizeof(delta_item_t))),
		 memory_pressure,
		 sizeof(Tracker) - sizeof(graph));
	PX4_INFO("  home: %zu-%.2f (%d %d %d)", nodes->index1, (double)coef_to_float(nodes->coef1), home_on_graph.x,
		 home_on_graph.y, home_on_graph.z);

	ipos_t pos = graph_head_pos;

	size_t prev_index = graph_next_write;
	size_t index = graph_next_write - 1;

	while (prev_index) {
		prev_index = index;
		bool is_jump;
		ipos_t delta = walk_backward(index, is_jump);

		PX4_INFO("  %zu: (%d, %d, %d)%s%s",
			 prev_index, pos.x, pos.y, pos.z,
			 is_jump ? ", jump" : "",
			 nodes->index1 == prev_index ? " <= home" : "");

		pos -= delta;
	};

	dump_nodes();
}

void Tracker::dump_nodes()
{
	PX4_INFO("nodes (%zu elements of size %zu bytes)%s:", node_count, (sizeof(node_t) * CHAR_BIT) / 8,
		 have_dirty_nodes ? " *" : "");

	for (size_t i = 0; i < node_count; i++) {
		PX4_INFO("  node %zu: %s%zu-%.2f%s to %s%zu-%.2f%s, expansion %.3f, dist to home %d %s%s%s", i,
			 node_at(i).use_line2 ? " " : ">", (size_t)node_at(i).index1, (double)coef_to_float(node_at(i).coef1),
			 node_at(i).use_line2 ? " " : "<",
			 node_at(i).use_line2 ? ">" : " ", (size_t)node_at(i).index2, (double)coef_to_float(node_at(i).coef2),
			 node_at(i).use_line2 ? "<" : " ",
			 (double)fast_sqrt(dot(unpack_compact_delta_item(node_at(i).delta)), false), node_at(i).distance,
			 node_at(i).go_forward ? "forward" : "backward",
			 node_at(i).dirty ? " *" : "",
			 i ? "" : " <= home");
	}
}

void Tracker::dump_path_to_home()
{
	path_finding_context_t context;

	float x, y, z;

	if (!init_return_path(context, x, y, z)) {
		PX4_INFO("shortest path to home unavailable");
		return;
	}

	PX4_INFO("shortest path to home:");

	do {
		PX4_INFO("  %zu-%.2f: (%d, %d, %d)", context.current_index, (double)coef_to_float(context.current_coef), (int)x, (int)y,
			 (int)z);
	} while (advance_return_path(context, x, y, z));
}