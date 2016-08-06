
#include <limits.h>     // CHAR_BIT
#include <algorithm>    // std::min
#include <geo/geo.h>
#include "tracker.h"


inline float fast_sqrt(int val, bool fallback_to_infinity) {
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

    if (val < sizeof(values) / sizeof(values[0]))
        return values[val];

    if (fallback_to_infinity)
        return INFINITY;

    float result = sqrt(val);
    if (isnan(result))
        PX4_WARN("fast_sqrt returns NaN for %d", val);
    return result;
}

inline bool Tracker::is_close(fpos_t pos1, fpos_t pos2) {
    float delta_x = pos1.x - pos2.x;
    float delta_y = pos1.y - pos2.y;
    float delta_z = pos1.z - pos2.z;
    return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z < GRID_SIZE * GRID_SIZE + 0.001;
}

inline bool Tracker::is_close(ipos_t pos1, fpos_t pos2) {
    return is_close(to_fpos(pos1), pos2);
}


int Tracker::dot(ipos_t vec1, ipos_t vec2) {
    if (!fits_into_far_delta(vec1) || !fits_into_far_delta(vec2))
        return INT_MAX;
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}


Tracker::ipos_t Tracker::get_point_to_line_delta(ipos_t point, ipos_t line_delta, ipos_t line_end, int &coef) {
    // The negative coefficient is obtained by projecting the point along the line direction
    coef = float_to_coef(dot(line_end - point, line_delta) / (float)dot(line_delta));

    // Constrain coefficient to beginning or end of line
    int max_coefficient = MAX_COEFFICIENT;
    coef = std::max(0, std::min(max_coefficient, coef));

    // Return result
    return line_end - apply_coef(line_delta, coef);
}


Tracker::ipos_t Tracker::get_line_to_line_delta(ipos_t delta1, ipos_t end1, ipos_t delta2, ipos_t end2, int &coef1, int &coef2, bool pin_coef1, bool pin_coef2) {
    
    // If either of the two coefficients is pinned, apply it to the line end
    if (pin_coef1)
        end1 -= apply_coef(delta1, coef1);
    if (pin_coef2)
        end2 -= apply_coef(delta2, coef2);
    
    if (pin_coef1 && pin_coef2)
        return end2 - end1;

    // A point-to-line distance is sufficient if a coefficient is pinned.
    if (pin_coef1)
        return get_point_to_line_delta(end1, delta2, end2, coef2);
    if (pin_coef2)
        return -get_point_to_line_delta(end2, delta1, end1, coef1);

    // None of the coefficients are pinned, so we go on to calculate the true line-to-line delta
    
    // Find the direction that is normal to both lines (using cross product)
    ipos_t normal = {
        .x = delta1.y * delta2.z - delta1.z * delta2.y,
        .y = delta1.z * delta2.x - delta1.x * delta2.z,
        .z = delta1.x * delta2.y - delta1.y * delta2.x
    };

    TRACKER_DBG("line to line normal: %d %d %d", normal.x, normal.y, normal.z);

    // Project a connection between both lines along the direction of the normal.
    ipos_t end_to_end_delta = end2 - end1;
    ipos_t temp_delta = to_ipos(to_fpos(normal) * (dot(end_to_end_delta, normal) / (float)dot(normal)));
    
    TRACKER_DBG("end-to-end factor: %d / %d = %.3f temp delta: %d %d %d", dot(end_to_end_delta, normal), dot(normal), (dot(end_to_end_delta, normal) / (float)dot(normal)), temp_delta.x, temp_delta.y, temp_delta.z);

    // Remove the orthogonal component - the end-to-end delta is now a linear combination of both line directions
    end_to_end_delta -= temp_delta;

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


    TRACKER_DBG("delta1: (%d %d %d), delta2: (%d %d %d), delta3: (%d %d %d), delta4: (%d %d %d)",
        start1_to_line2.x, start1_to_line2.y, start1_to_line2.z,
        end1_to_line2.x, end1_to_line2.y, end1_to_line2.z,
        start2_to_line1.x, start2_to_line1.y, start2_to_line1.z,
        end2_to_line1.x, end2_to_line1.y, end2_to_line1.z);
        

    // Of all the point-to-line deltas, return the smallest one.
    int best_to_line1 = std::min(dot(start2_to_line1), dot(end2_to_line1));
    int best_to_line2 = std::min(dot(start1_to_line2), dot(end1_to_line2));

    if (best_to_line1 < best_to_line2) {
        if (dot(start2_to_line1) < best_to_line1) {
            coef1 = temp_coef1;
            coef2 = MAX_COEFFICIENT;
            return -start2_to_line1;
        } else { // end2_to_line1 is the shortest delta
            coef2 = 0;
            return -end2_to_line1;
        }
    } else {
        if (dot(start1_to_line2) < best_to_line2) {
            coef1 = MAX_COEFFICIENT; 
            coef2 = temp_coef2;
            return start1_to_line2;
        } else { // end1_to_line2 is the shortest delta
            coef1 = 0;
            return end1_to_line2;
        }
    }
}


inline Tracker::delta_item_t Tracker::pack_compact_delta_item(ipos_t delta) {
    return ((delta.x & 0x1F) << 10) | ((delta.y & 0x1F) << 5) | ((delta.z & 0x1F) << 0);
}

inline Tracker::ipos_t Tracker::unpack_compact_delta_item(delta_item_t delta) {
    const int SHIFT_COUNT = CHAR_BIT * sizeof(int) - 5;
    return {
        .x = ((int)(delta << (SHIFT_COUNT - 10))) >> SHIFT_COUNT,
        .y = ((int)(delta << (SHIFT_COUNT - 5))) >> SHIFT_COUNT,
        .z = ((int)(delta << (SHIFT_COUNT - 0))) >> SHIFT_COUNT
    };
}

inline bool Tracker::fits_into_far_delta(ipos_t vec) {
    return  (((vec.x >> 14) == 0) || ((vec.x >> 14) == -1)) &&
            (((vec.y >> 14) == 0) || ((vec.y >> 14) == -1)) &&
            (((vec.z >> 14) == 0) || ((vec.z >> 14) == -1));
}

int Tracker::get_accuracy_at(ipos_t pos) {
    // todo: adapt accuracy to memory pressure
    return GRID_SIZE * GRID_SIZE;
}

void Tracker::set_home(float x, float y, float z) {
    home_position = to_ipos({ .x = x, .y = y, .z = z });

    nodes->dirty = 1;

    // Reset the distance-to-home of all nodes to infinity (except the home node)
    for (size_t i = 1; i < node_count; i++) {
        nodes[i].distance = MAX_DISTANCE;
    }

    consolidate_graph();

    TRACKER_DBG("tracker received new home position %zu-%.2f (%.2f, %.2f, %.2f)", x, y, z, nodes->index1, coef_to_float(nodes->coef1));
}

void Tracker::update(float x, float y, float z) {
    fpos_t local_position = {
        .x = x,
        .y = y,
        .z = z
    };
    
    if (recent_path_tracking_enabled)
        push_recent_path(local_position);

    if (graph_tracking_enabled)
        push_graph(local_position);
}


void Tracker::push_recent_path(fpos_t &position) {
    
    bool rollback = false;
    size_t index = recent_path_next_write;
    ipos_t head = to_ipos(recent_path_head);
    
    if (recent_path_next_read != RECENT_PATH_LENGTH) {
        
        // If we're still close to the most recent position, don't update
        if (is_close(head, position))
            return;
        
        // Scan the buffer for a close position, starting at the latest one
        do {
            index = (index ? index : RECENT_PATH_LENGTH) - 1;
            
            if ((recent_path[index] >> 15) & 1) // If this item is a bumper, we shouldn't roll over this invalid position (todo: use far jump element as bumper)
                break; 

            head -= unpack_compact_delta_item(recent_path[index]);
            rollback = is_close(head, position);
        } while ((index != recent_path_next_read) && !rollback);

        // If the preceding item would be a bumper, we shouldn't roll back to this invalid position
        if (rollback && index != recent_path_next_read)
            if ((recent_path[(index ? index : RECENT_PATH_LENGTH) - 1] >> 15) & 1)
                rollback = false;
    }
    
    
    if (rollback) {
        
        // If there was a close position in the path, roll the path back to that position
        recent_path_head = to_fpos(head);

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
        
        ipos_t old_head = to_ipos(recent_path_head);
        ipos_t new_head = to_ipos(position);
        
        delta_item_t item = pack_compact_delta_item(new_head - old_head);
        
        recent_path_head = position;
        recent_path[recent_path_next_write] = item;
        
        if (recent_path_next_write++ == recent_path_next_read)
            recent_path_next_read = recent_path_next_write;
            
        if (recent_path_next_write == RECENT_PATH_LENGTH)
            recent_path_next_write = 0;
            
        if (recent_path_next_read == RECENT_PATH_LENGTH)
            recent_path_next_read = 0;
    }
}


bool Tracker::pop_recent_path(fpos_t &position) {
    position = recent_path_head;
    
    // Check if path is empty
    if (recent_path_next_read == RECENT_PATH_LENGTH)
        return false;
    
    int last_index = (recent_path_next_write ? recent_path_next_write : RECENT_PATH_LENGTH) - 1;

    // Don't consume bumper item
    if ((recent_path[last_index] >> 15) & 1)
        return false;
    
    // Roll back most recent position
    ipos_t head = to_ipos(recent_path_head);
    head -= unpack_compact_delta_item(recent_path[last_index]);
    recent_path_head = to_fpos(head);
    
    if ((recent_path_next_write = last_index) == recent_path_next_read) {
        recent_path_next_write = 0;
        recent_path_next_read = RECENT_PATH_LENGTH;
    }
    return true;
}


bool Tracker::pop_recent_path(double &lat, double &lon, float &alt) {
    fpos_t position;
    
    if (!pop_recent_path(position))
        return false;
    
    return !globallocalconverter_toglobal(position.x, position.y, position.z, &lat, &lon, &alt);
}


void Tracker::push_graph(fpos_t &position) {
    ipos_t new_pos = to_ipos(position);

    // Don't update if we're still close to the last known line
    if (graph_next_write) {
        int coef;
        bool is_jump;
        ipos_t delta = get_point_to_line_delta(new_pos, fetch_delta(graph_next_write - 1, is_jump), graph_head_pos, coef);
        if (!is_jump && dot(delta) <= get_accuracy_at(new_pos))
            return;
    }

    // Don't update if the graph is full
    if (graph_next_write + FAR_DELTA_SIZE <= GRAPH_LENGTH) {
        if (!graph_next_write)
            consolidated_head_pos = new_pos;

        push_delta(graph_next_write, new_pos - graph_head_pos, false);
        graph_head_pos = new_pos;
        
        if (graph_next_write - 1 - consolidated_head_index >= MAX_CONSOLIDATION_DEPT)
            consolidate_graph();
    } else {
        // todo: cleanup-pass
        consolidate_graph();
    }
}

bool Tracker::push_delta(size_t &index, ipos_t delta, bool jump, size_t max_space) {
    int max_val = std::max(std::max(delta.x, delta.y), delta.z);
    int min_val = std::min(std::min(delta.x, delta.y), delta.z);

    // If the delta is too large for a single compact delta item, we split it into multiple items
    int split_count = 1;
    if (max_val > COMPACT_DELTA_MAX || min_val < COMPACT_DELTA_MIN)
        split_count = (int)ceil(std::max((float)max_val / (float)COMPACT_DELTA_MAX, (float)min_val / (float)COMPACT_DELTA_MIN));

    int far_delta_size = FAR_DELTA_SIZE;
    split_count = std::min(split_count, far_delta_size);
    
    if (max_space < split_count)
        return false;

    if (split_count >= FAR_DELTA_SIZE || jump) {
        if (max_val > FAR_DELTA_MAX || min_val < FAR_DELTA_MIN)
            PX4_ERR("delta overflow in tracker");

        graph[index++] = (delta.x & 0x7FFF) | 0x8000;
        graph[index++] = (delta.y & 0x7FFF) | jump ? 0x8000 : 0;
        graph[index++] = (delta.z & 0x7FFF) | 0x8000;

    } else {

        ipos_t compact_delta_sum = { .x = 0, .y = 0, .z = 0 };
        for (int i = 1; i < split_count; i++) {
            ipos_t compact_delta = to_ipos(to_fpos(delta) * ((float)i / (float)split_count)) - compact_delta_sum;
            graph[index++] = pack_compact_delta_item(compact_delta);
            compact_delta_sum += compact_delta;
        }
        delta -= compact_delta_sum;

        graph[index++] = pack_compact_delta_item(delta);
    }

    return true;
}

Tracker::ipos_t Tracker::fetch_delta(size_t index, bool &is_jump) {
    const int SHIFT_COUNT = CHAR_BIT * sizeof(int) - 15;

    if (!((graph[index] >> 15) & 1)) {
        is_jump = false;
        return unpack_compact_delta_item(graph[index]);
    }

    is_jump = (graph[index - 1] >> 15) & 1;
    return {
        .x = ((int)((graph[index - 2] & 0x7FFF) << SHIFT_COUNT)) >> SHIFT_COUNT,
        .y = ((int)((graph[index - 1] & 0x7FFF) << SHIFT_COUNT)) >> SHIFT_COUNT,
        .z = ((int)((graph[index - 0] & 0x7FFF) << SHIFT_COUNT)) >> SHIFT_COUNT
    };
}

Tracker::ipos_t Tracker::walk_forward(size_t &index, bool &is_jump) {
    if ((graph[index + 1] >> 15) & 1)
        return fetch_delta(index += FAR_DELTA_SIZE, is_jump);
    else
        return fetch_delta(index += 1, is_jump);
}

Tracker::ipos_t Tracker::walk_backward(size_t &index, bool &is_jump) {
    ipos_t delta = fetch_delta(index, is_jump);

    if ((graph[index] >> 15) & 1)
        index -= FAR_DELTA_SIZE;
    else
        index -= 1;

    return delta;
}


bool Tracker::push_node(node_t &node) {
    if (node_count >= NODE_BUFFER_SIZE)
        return false;

    nodes[node_count++] = node;
    have_dirty_nodes = true;
    return true; // todo: take some action if the buffer becomes full
}


void Tracker::remove_nodes(size_t lower_bound, size_t upper_bound) {
    for (size_t i = 1; i < node_count; i++) {
        if ((lower_bound < nodes[i].index1 && nodes[i].index1 <= upper_bound) ||
            (lower_bound < nodes[i].index2 && nodes[i].index2 <= upper_bound)) {
            memcpy(nodes + i, nodes + i + 1, --node_count - i);
            i--;
        }
    }
}


bool Tracker::is_close_to_graph(ipos_t position, size_t lower_bound, size_t upper_bound, ipos_t pos_at_upper_bound) {
    int accuracy_squared = get_accuracy_at(position);

    while (upper_bound > lower_bound) {
        bool is_jump;
        ipos_t delta = walk_backward(upper_bound, is_jump);
        
        int coef;
        delta = is_jump ? (position - pos_at_upper_bound) : get_point_to_line_delta(position, delta, pos_at_upper_bound, coef);
        
        if (dot(delta) <= accuracy_squared)
            return true;

        pos_at_upper_bound -= delta;
    }

    return false;
}


Tracker::ipos_t Tracker::get_closest_position(ipos_t position, size_t &best_index, unsigned int &best_coef) {

    ipos_t pos = graph_head_pos;
    size_t index = graph_next_write ? (graph_next_write - 1) : 0;
    int coef;

    int best_dist_squared = INT_MAX;
    ipos_t best_pos = { .x = 0, .y = 0, .z = 0 };
    best_index = 0;
    best_coef = 0;

    while (index) {
        size_t prev_index = index;
        bool is_jump;
        ipos_t delta = walk_backward(index, is_jump);
        
        int dummy = 0;
        delta = get_line_to_line_delta(delta, pos, { .x = 0, .y = 0, .z = 0 }, position, coef = 0, dummy, is_jump, true);
        int distance_squared = dot(delta);
        
        if (best_dist_squared > distance_squared) {
            best_dist_squared = distance_squared;
            best_pos = position - delta;
            best_index = prev_index;
            best_coef = coef;
        }

        pos -= delta;
    }

    return best_pos;
}


bool Tracker::is_line(ipos_t start_pos, size_t start_index, ipos_t end_pos, size_t end_index, bool should_be_jump) {
    ipos_t line_delta = end_pos - start_pos;
    ipos_t pos = end_pos;

    // The efficiency gain of prefetching the accuracy seems to outweigh the corner-cases there the accuracy along the line should be higher than on both ends.
    int squared_accuracy = std::min(get_accuracy_at(start_pos), get_accuracy_at(end_pos));

    // Intuitively, it seems that walking backward allows us to break earlier than walking forward.

    for (;;) {
        bool is_jump;
        pos -= walk_backward(end_index, is_jump);

        // The line must have a uniform jump property.
        if (is_jump != should_be_jump)
            return false;

        if (end_index <= start_index)
            return true;

        int coef;
        ipos_t delta = get_point_to_line_delta(pos, line_delta, end_pos, coef);
        if (dot(delta) > squared_accuracy)
            return false;
    }
}


void Tracker::get_longest_line(ipos_t start_pos, size_t start_index, ipos_t &end_pos, size_t &end_index, bool &is_jump, size_t bound) {
    ipos_t next_end_pos = start_pos;
    size_t next_end_index = start_index;

    do {
        end_index = next_end_index;
        end_pos = next_end_pos;
        next_end_pos += walk_forward(next_end_index, is_jump);
    } while (is_line(start_pos, start_index, next_end_pos, next_end_index, is_jump) && next_end_index <= bound);
}


// Finds the line on the graph that is closest to the provided line. Jumps are not considered lines.
//  line_delta, line_end: the delta and end of the line against which we want to compare
//  lower_bound, upper_bound: marks the search bounds (lower bound < returned index <= upper bound)
//  pos_at_bound: the position corresponding to upper_bound
//  index: set to the index of the closest line (0 if no line was found)
// Returns: the delta from somewhere (coef1) on the input line to somewhere (coef2) on another line.
/*Tracker::ipos_t Tracker::get_best_line_to_line_delta(ipos_t line_delta, ipos_t line_end, size_t lower_bound, size_t upper_bound, ipos_t pos_at_bound, size_t &index, int &coef1, int &coef2) {
    int best_distance_squared = MAX_INT;
    ipos_t best_index = 0;
    ipos_t best_delta = { .x = FAR_DELTA_MAX, .y = FAR_DELTA_MAX, .z = FAR_DELTA_MAX };

    while (upper_bound > lower_bound) {
        size_t prev_index = upper_bound;
        bool is_jump;
        walk_backward(upper_bound, delta, is_jump);

        // Ignore jumps
        if (is_jump) {
            pos_at_upper_bound -= delta;
            continue;
        }
        
        int temp_coef1, temp_coef2;
        size_t line_to_line_delta = get_line_to_line_delta(delta, pos_at_index, line_delta, line_end, temp_coef1, temp_coef2);

        pos_at_upper_bound -= delta;

        int distance_squared = dot(line_to_line_delta) 
        if (best_distance_squared > distance_squared) {
            best_distance_squared = distance_squared;
            best_delta = line_to_line_delta;
            best_index = prev_index;
            coef1 = temp_coef1;
            coef2 = temp_coef2;
        }
    }

    index = best_index;
    return best_delta;
}*/


// Consolidates the most recent positions on the graph, by applying some optimizations:
//  1. Consecutive positions that lie roughly on a line are aggeregated.
//  2. If multiple consecutive positions lie close to the graph, they are removed and replaced by a jump.
//  3. For lines that pass close to each other, a node is created.
// The entire tracker is guaranteed to remain consistent, however exposed indices may become invalid.
void Tracker::consolidate_graph() {
    TRACKER_DBG("consolidating graph from %zu to %zu", consolidated_head_index, graph_next_write - 1);

    // Abort if the graph is empty
    if (!graph_next_write)
        return;

    // Remove any nodes that refer to the area that will be modified.
    // Any nodes for that area will be newly created anyway.
    remove_nodes(consolidated_head_index, graph_next_write - 1);

    /*** Pass 1: detect lines ***/

    ipos_t pos = consolidated_head_pos;
    size_t read_index = consolidated_head_index;
    size_t write_index = read_index + 1;

    // read_index and (write_index - 1) always correspond to the same position.
    // They start out to be equal, and the gap between them increases as consolidation moves on.

    while (read_index < graph_next_write - 1) {
        bool is_jump;
        ipos_t line_end_pos;
        size_t line_end_index;
        get_longest_line(pos, read_index, line_end_pos, line_end_index, is_jump, graph_next_write - 1);
        
        // Push the line delta ONLY if it uses less space than the original deltas
        if (push_delta(write_index, line_end_pos - pos, is_jump, line_end_index - read_index)) {
            TRACKER_DBG("aggeregated %zu to %zu into line %zu", read_index, line_end_index, write_index - 1);
            read_index = line_end_index;
        }

        // Copy the deltas that were not aggeregated into a line
        while (read_index < line_end_index)
            push_delta(write_index, walk_forward(read_index, is_jump), is_jump);
        
        pos = line_end_pos;
    }
    
    graph_next_write = write_index;


    /*** Pass 2: remove positions that are already in the graph ***/

    pos = consolidated_head_pos;
    read_index = consolidated_head_index;
    write_index = read_index + 1;

    // Keep track of the index/position from where we may overwrite previously copied deltas.
    size_t overwrite_index = write_index;
    ipos_t pos_at_overwrite_index = pos;
    bool is_close = false;

    while (read_index < graph_next_write - 1) {
        bool is_jump;
        ipos_t prev_pos = pos;
        ipos_t delta = walk_forward(read_index, is_jump);
        pos += delta;

        // We copy each position optimistically, even if it turns out that it will be deleted later on.
        size_t prev_write_index = write_index;
        push_delta(write_index, delta, is_jump);

        // If the last position wasn't close to the graph, we must definitely keep this one.
        if (!is_close) {
            overwrite_index = write_index;
            pos_at_overwrite_index = pos;
        }

        // Scan the graph to see if any line is close to the current position
        is_close = is_close_to_graph(pos, overwrite_index > GRAPH_SEARCH_RANGE ? overwrite_index - GRAPH_SEARCH_RANGE : 0, overwrite_index - 1, pos_at_overwrite_index);

        // As soon as we encounter a position which is not close to the graph, we see if we can remove some preceeding positions
        if (!is_close) {
            
            // If the last few positions were close to the graph, replace them by a jump
            if (overwrite_index < prev_write_index) {
                size_t max_space = prev_write_index - overwrite_index;
                if (push_delta(overwrite_index, prev_pos - pos_at_overwrite_index, true, max_space)) {
                    // We append the last position once again and go on from there
                    push_delta(write_index = overwrite_index, delta, is_jump);

                    TRACKER_DBG("replaced redundant path of size %zu by a jump at %zu", max_space, write_index - 1);
                }
            }

            // If this branch was taken, the last important position will be re-initialized once it becomes relevant
        }
    }

    graph_next_write = write_index;


    /*** Pass 3: detect nodes ***/

    pos = consolidated_head_pos;
    read_index = consolidated_head_index;

    while (read_index < graph_next_write - 1) {

        ipos_t inner_pos = pos;
        size_t search_index = read_index;

        bool is_jump;
        ipos_t delta = walk_forward(read_index, is_jump);
        pos += delta;

        int squared_accuracy = get_accuracy_at(pos);
        size_t search_bound = read_index > GRAPH_SEARCH_RANGE ? read_index - GRAPH_SEARCH_RANGE : 0;

        while (search_index > search_bound) {

            // Prepare a node in case we need it
            node_t node = {
                .index1 = (uint16_t)read_index,
                .index2 = (uint16_t)search_index,
                .coef1 = 0,
                .coef2 = 0,
                .use_line2 = 0,
                .dirty = 1,
                .distance = MAX_DISTANCE
            };

            bool is_inner_jump;
            int coef1, coef2;
            ipos_t inner_delta = walk_backward(search_index, is_inner_jump);
            ipos_t line_to_line = get_line_to_line_delta(delta, pos, inner_delta, inner_pos, coef1, coef2, is_jump, is_inner_jump);

            // If the node would be at the beginning of the line, defer the node creation to the next line (unless we're about to terminate the search)
            if (node.coef2 >= MAX_COEFFICIENT && search_index > search_bound)
                continue;

            if (dot(line_to_line) <= squared_accuracy) {
                if (coef1 < 0 || coef1 > MAX_COEFFICIENT || coef2 < 0 || coef2 > MAX_COEFFICIENT)
                    PX4_WARN("delta coefficient out of range: have %d and %d", coef1, coef2);
                node.coef1 = coef1;
                node.coef2 = coef2;
                node.delta = pack_compact_delta_item(line_to_line);                
                
                push_node(node);
            }
            
            inner_pos -= inner_delta;
        }
    }


    // Sanitize home position
    size_t home_index;
    unsigned int home_coef;
    home_on_graph = get_closest_position(home_position, home_index, home_coef);
    if (nodes->index1 != home_index || nodes->coef1 != home_coef) {
        nodes->index1 = home_index;
        nodes->coef1 = home_coef;
        have_dirty_nodes = true;
    }

    // Finalize the consolidation
    consolidated_head_pos = graph_head_pos;
    consolidated_head_index = graph_next_write - 1;
    graph_version++;

    TRACKER_DBG("consolidation complete, graph reduced to %zu", graph_next_write - 1);
}


bool Tracker::walk_to_node(size_t &index, int &coef, float &distance, bool forward, size_t search_bound) {
    bool is_jump;
    ipos_t delta = fetch_delta(index, is_jump);
    float pos_to_line_end_dist = fast_sqrt(dot(apply_coef(delta, coef)), false);

    // Tweak the starting position a bit, so we don't match the exact same position where we left off
    if (!forward)
        coef++;

    for (;;) {
        // Look through all nodes to find one that lies on the current line in the direction of travel
        for (size_t i = 0; i < node_count; i++) {
            if (nodes[i].index1 == index || nodes[i].index2 == index) {
                int node_coef = nodes[i].index1 == index ? nodes[i].coef1 : nodes[i].coef2;
                if (forward ? (node_coef < coef) : (node_coef >= coef)) {
                    float node_to_line_end_dist = fast_sqrt(dot(apply_coef(delta, node_coef)), false);
                    distance += std::abs(pos_to_line_end_dist - node_to_line_end_dist);
                    coef = node_coef;
                    return true;
                }
            }
        }

        // Walk in the direction of travel
        if (forward) {
            distance += pos_to_line_end_dist;
            if (index >= search_bound)
                break;
            delta = walk_forward(index, is_jump);
            if (is_jump)
                break;
            coef = MAX_COEFFICIENT + 1;
            pos_to_line_end_dist = fast_sqrt(dot(delta), false);
        } else {
            if (is_jump)
                break;
            distance += fast_sqrt(dot(delta), false) - pos_to_line_end_dist;
            delta = walk_backward(index, is_jump);
            if (index <= search_bound)
                break;
            coef = 0;
            pos_to_line_end_dist = 0;
        }
    }

    return false;
}


float Tracker::apply_node_delta(size_t &index, unsigned int &coef, ipos_t *delta) {
    float best_distance = INFINITY;
    size_t best_index = index;
    int best_coef = coef;
    
    if (delta)
        *delta = { .x = 0, .y = 0, .z = 0 };

    // Consider all intersections at the specified position and use the one with the smallest distance home.
    for (size_t i = 0; i < node_count; i++) {
        if ((nodes[i].index1 == index && nodes[i].coef1 == coef) || (nodes[i].index2 == index && nodes[i].coef2 == coef)) {
            float distance = nodes[i].distance == MAX_DISTANCE ? FLT_MAX : (float)nodes[i].distance;
            
            // If the node is far from home, we arbitrarily prefer the line with the lowest index.
            // This is in line with the decision that we prefer to walk backward. Thus, in case of
            // unknown distances, at least we strictly minimize the index and don't get stuck in an infinite loop.
            if (distance >= FLT_MAX)
                nodes[i].use_line2 = nodes[i].index2 < nodes[i].index1;

            if (best_distance > distance) {
                best_distance = distance;
                best_index = nodes[i].use_line2 ? nodes[i].index2 : nodes[i].index1;
                best_coef = nodes[i].use_line2 ? nodes[i].coef2 : nodes[i].coef1;
                //move_forward = node[i].move_forward;
                
                if (delta) {
                    // If we don't change lines, the delta is 0
                    if (index == (nodes[i].use_line2 ? nodes[i].index2 : nodes[i].index1))
                        *delta = { .x = 0, .y = 0, .z = 0 };
                    else if (nodes[i].use_line2)
                        *delta = unpack_compact_delta_item(nodes[i].delta);
                    else
                        *delta = -unpack_compact_delta_item(nodes[i].delta);
                }
            }
        }
    }

    index = best_index;
    coef = best_coef;
    return best_distance;
}


bool Tracker::set_node_distance(size_t index, int coef, float distance, bool go_forward) {
    bool improvement = false;

    // Update all intersections at the specified position (if the new distance is an improvement)
    for (size_t i = 0; i < node_count; i++) {
        if ((nodes[i].index1 == index && nodes[i].coef1 == coef) || (nodes[i].index2 == index && nodes[i].coef2 == coef)) {
            float new_distance = distance + fast_sqrt(dot(unpack_compact_delta_item(nodes[i].delta)), false);
            int int_distance = new_distance > MAX_DISTANCE ? MAX_DISTANCE : round(new_distance);

            if (nodes[i].distance > int_distance) {
                nodes[i].distance = int_distance;
                nodes[i].use_line2 = index == nodes[i].index2 && coef == nodes[i].coef2;
                nodes[i].go_forward = go_forward;
                improvement = true;
                have_dirty_nodes = true;
            }
        }
    }

    TRACKER_DBG("  distance at %zu (coef %d), improved to %f + deltalength", index, coef, distance);
    return improvement;
}

void Tracker::refresh_distances() {
    if (!graph_next_write)
        return;

    // Only log if we're gonna do something
    bool should_log = have_dirty_nodes;
    if (should_log)
        TRACKER_DBG("calculating distances...");

    // This loop is guaranteed to terminate.
    // Proof outline:
    //  The loop terminates when the set of dirty nodes is empty at the end of an iteration. This set is
    //  cleared at the beginning of every iteration. A node only gets added to this set, if its distance-
    //  to-home improves. A node distance cannot be increased and there are no intervals with negative
    //  length, so every node distance will eventually reach a lower bound. When this state is reached,
    //  the set of dirty nodes remains empty during an entire iteration, thus the loop terminates.

    while (have_dirty_nodes) {
        have_dirty_nodes = false;

        for (size_t i = 0; i < node_count; i++) {
            node_t node = nodes[i];

            // At every dirty node, we look in all four directions of the intersection.
            if (node.dirty) {
                node.dirty = 0;

                size_t index;
                int coef;
                float interval_length;

                float distance_here = get_node_distance(node.index1, node.coef1);
                float distance_there;

                // Walk forward from index1
                if (walk_to_node(index = node.index1, coef = node.coef1, interval_length = 0, true, graph_next_write - 1)) {
                    distance_there = get_node_distance(index, coef); // may return old distance
                    set_node_distance(index, coef, distance_here + interval_length, false);
                    set_node_distance(node.index1, node.coef1, distance_there + interval_length, true);
                }

                // Walk backward from index1
                if (walk_to_node(index = node.index1, coef = node.coef1, interval_length = 0, false, 0)) {
                    distance_there = get_node_distance(index, coef);
                    set_node_distance(index, coef, distance_here + interval_length, true);
                    set_node_distance(node.index1, node.coef1, distance_there + interval_length, false);
                }

                distance_here = get_node_distance(node.index2, node.coef2);

                // Walk forward from index2
                if (walk_to_node(index = node.index2, coef = node.coef2, interval_length = 0, true, graph_next_write - 1)) {
                    distance_there = get_node_distance(index, coef); // may return old distance
                    set_node_distance(index, coef, distance_here + interval_length, false);
                    set_node_distance(node.index2, node.coef2, distance_there + interval_length, true);
                }

                // Walk backward from index2
                if (walk_to_node(index = node.index2, coef = node.coef2, interval_length = 0, false, 0)) {
                    distance_there = get_node_distance(index, coef);
                    set_node_distance(index, coef, distance_here + interval_length, true);
                    set_node_distance(node.index2, node.coef2, distance_there + interval_length, false);
                }
            }
        }
    }

#ifdef DEBUG_TRACKER
    if (should_log)
        dump_nodes();
#endif
}

/*
void Tracker::get_distance_to_home(size_t index, float bias, float &dist_forward, float &dist_backward) {
    // Calculate the distance if walking forward from index
    dist_forward = bias;
    size_t index_forward = index;
    size_t node_forward = walk_far_forward(index_forward, graph_home_index, dist_forward);
    if (node_forward)
        dist_forward += get_link_dist_to_home(node_forward, get_delta_length(node_forward), index_forward);
    else if (index_forward != graph_home_index)
        dist_forward = INFINITY;

    // Calculate the distance if walking backward from index
    dist_backward = -bias;
    size_t index_backward = index;
    size_t node_backward = walk_far_backward(index_backward, graph_home_index, dist_backward);
    if (node_backward)
        dist_backward += get_link_dist_to_home(node_backward, get_delta_length(node_backward), index_backward);
    else if (index_backward != graph_home_index)
        dist_backward = INFINITY;

    if (isinf(dist_forward) && node_forward)
        dist_forward = FLT_MAX;

    if (isinf(dist_backward) && node_backward)
        dist_backward = FLT_MAX;
}
*/


bool Tracker::calc_return_path(path_finding_context_t context) {
    // Before making any decision, make sure we know the distance of every node to home.
    refresh_distances();

    TRACKER_DBG("calculate return path from %zu-%.2f.", context.current_index, coef_to_float(context.current_coef));

    ipos_t delta = { .x = 0, .y = 0, .z = 0 };
    
    // If we reached the checkpoint, re-evaluate next checkpoint and potentially switch lines.
    if (context.current_index == context.checkpoint_index && context.current_coef == context.checkpoint_coef) {
        TRACKER_DBG("checkpoint!");

        if (context.checkpoint_index == nodes->index1 && context.checkpoint_coef == nodes->coef1)
            return false;
        
        // If we're at a node (which doesn't have to be the case), this will switch to the line that gives the best path home. 
        apply_node_delta(context.current_index, context.current_coef, &delta);
        
        // Determine cost and bound of walking forward
        float forward_dist;
        size_t forward_node_index = context.current_index;
        int forward_node_coef = context.current_coef;
        if (walk_to_node(forward_node_index, forward_node_coef, forward_dist = 0, true, graph_next_write - 1))
            forward_dist += get_node_distance(forward_node_index, forward_node_coef);
        else
            forward_dist = INFINITY;

        // Determine cost and bound of walking backward
        float backward_dist;
        size_t backward_node_index = context.current_index;
        int backward_node_coef = context.current_coef;
        if (walk_to_node(backward_node_index, backward_node_coef, backward_dist = 0, false, 0))
            backward_dist += get_node_distance(backward_node_index, backward_node_coef);
        else
            backward_dist = INFINITY;

        TRACKER_DBG("distance from %zu-%.2f is %f forward (to %zu-%.2f) and %f backward (to %zu-%.2f)", context.current_index, coef_to_float(context.current_coef), forward_dist, forward_node_index, coef_to_float(forward_node_coef), backward_dist, backward_node_index, backward_node_coef);

        // Abort if we don't know what to do
        if (isinf(backward_dist) && forward_dist >= FLT_MAX) {
            PX4_WARN("Could not find any path home from %zu-%.2f.", context.current_index, coef_to_float(context.current_coef));
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
    }

    // If the (potential) node switch above did not yield a position update yet, advance in the direction of the next checkpoint
    if (!(delta.x || delta.y || delta.z)) {
        bool is_jump;
        ipos_t current_delta = fetch_delta(context.current_index, is_jump);
        delta = apply_coef(delta, context.current_coef);

        if (context.current_index < context.checkpoint_index) {
            current_delta = walk_forward(context.current_index, is_jump);
            if (is_jump)
                PX4_WARN("path finding algorithm is moving forward along jump delta");
            context.current_coef = MAX_COEFFICIENT + 1;
        } else if (context.current_index > context.checkpoint_index) {
            delta -= current_delta;
            if (is_jump)
                PX4_WARN("path finding algorithm is moving backward along jump delta");
            current_delta = walk_backward(context.current_index, is_jump);
            context.current_coef = 0;
        } else {
            if (is_jump)
                PX4_WARN("path finding algorithm is moving along jump delta");
            delta -= apply_coef(current_delta, context.checkpoint_coef);
            context.current_coef = context.checkpoint_coef;
        }
    }

    context.current_pos += delta;

    return true;
}


bool Tracker::init_return_path(path_finding_context_t &context, float &x, float &y, float &z) {
    // We prefer that the index we're about to expose remains valid, so we consolidate the graph first
    consolidate_graph();

    if (!graph_next_write)
        return false;

    // Use graph head as starting point
    context = {
        .current_pos = graph_head_pos,
        .current_index = graph_next_write - 1,
        .current_coef = 0,
        .graph_version = graph_version
    };

    context.checkpoint_index = context.current_index;
    context.checkpoint_coef = context.current_coef;

    x = context.current_pos.x;
    y = context.current_pos.y;
    z = context.current_pos.z;
    
    return true;
}

bool Tracker::advance_return_path(path_finding_context_t &context, float &x, float &y, float &z) {
    // If the graph version changed, we need to sanitize the path finding context
    if (context.graph_version != graph_version) {
        context.current_pos = get_closest_position(context.current_pos, context.current_index, context.current_coef);
        context.checkpoint_index = context.current_index;
        context.checkpoint_coef = context.current_coef;
    }

    bool result = calc_return_path(context);

    x = context.current_pos.x;
    y = context.current_pos.y;
    z = context.current_pos.z;

    return result;
}

bool Tracker::is_context_close_to_head(path_finding_context_t &context) {
    ipos_t delta = context.current_pos - graph_head_pos;
    return dot(delta) <= get_accuracy_at(context.current_pos);
}

bool Tracker::is_context_close_to_home(path_finding_context_t &context) {
    ipos_t delta = context.current_pos - home_on_graph;
    return dot(delta) <= get_accuracy_at(context.current_pos);
}

bool Tracker::is_same_pos(path_finding_context_t &context1, path_finding_context_t &context2) {
    return context1.current_index == context2.current_index && context1.current_coef == context2.current_coef;
}


void Tracker::dump_recent_path() {
    if (recent_path_next_read == RECENT_PATH_LENGTH) {
        PX4_INFO("recent path empty");
        return;
    }
    
    PX4_INFO("recent path:");
    size_t recent_path_length = (recent_path_next_write + RECENT_PATH_LENGTH - recent_path_next_read - 1) % RECENT_PATH_LENGTH + 1;
    PX4_INFO("  length: %zu (read-ptr: %zu, write-ptr: %zu)", recent_path_length, recent_path_next_read, recent_path_next_write);
    
    size_t index = recent_path_next_write;
    ipos_t head = to_ipos(recent_path_head);
    
    do {
        PX4_INFO("  (%d, %d, %d)", head.x, head.y, head.z);
        
        index = (index ? index : RECENT_PATH_LENGTH) - 1;

        if ((recent_path[index] >> 15) & 1)
            PX4_INFO("ignore the last position");

        head -= unpack_compact_delta_item(recent_path[index]);
    } while (index != recent_path_next_read);
}


void Tracker::dump_graph() {
    if (!graph_next_write) {
        PX4_INFO("full path empty");
        return;
    }

    PX4_INFO("full path (%zu elements of size %zu bytes):", graph_next_write, (sizeof(delta_item_t) * CHAR_BIT) / 8);
    PX4_INFO("  home: %zu-%.2f", nodes->index1, coef_to_float(nodes->coef1));

    ipos_t pos = graph_head_pos;

    size_t index = graph_next_write - 1;
    while (index) {
        size_t prev_index = index;
        bool is_jump;
        ipos_t delta = walk_backward(index, is_jump);

        PX4_INFO("  %zu: (%d, %d, %d)%s%s",
            prev_index, pos.x, pos.y, pos.z,
            is_jump ? ", jump" : "",
            nodes->index1 == index ? " <= home" : "");

        pos -= delta;
    };

    dump_nodes();
}

void Tracker::dump_nodes() {
    PX4_INFO("nodes (%zu elements of size %zu bytes):", node_count, (sizeof(node_t) * CHAR_BIT) / 8);
    for (size_t i = 0; i < node_count; i++) {
        PX4_INFO("  node %zu: %s%zu-%.2f%s to %s%zu-%.2f%s, expansion %.3f, dist to home %d%s", i,
            nodes[i].use_line2 ? " " : ">", (size_t)nodes[i].index1, coef_to_float(nodes[i].coef1), nodes[i].use_line2 ? " " : "<",
            nodes[i].use_line2 ? ">" : " ", (size_t)nodes[i].index2, coef_to_float(nodes[i].coef2), nodes[i].use_line2 ? "<" : " ",
            fast_sqrt(dot(unpack_compact_delta_item(nodes[i].delta)), false), nodes[i].distance,
            i ? "" : " <= home");
    }
}

void Tracker::dump_path_to_home() {
    path_finding_context_t context;

    float x, y, z;
    if (!init_return_path(context, x, y, z)) {
        PX4_INFO("shortest path to home unavailable");
        return;
    }

    PX4_INFO("shortest path to home:");

    do {
        PX4_INFO("  %zu-%.2f: (%d, %d, %d)", context.current_index, coef_to_float(context.current_coef), (int)x, (int)y, (int)z);
    } while (advance_return_path(context, x, y, z));
}
