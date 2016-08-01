
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

// dist_squared: Set to NULL to save some time if the value is not required
//  If not NULL, it is guaranteed to receive a valid value.
// coefficient: Set to NULL to save some time if the value is not required
//  If not NULL, the output is only guaranteed to be valid if the function returned true OR dist_squared was also requested
bool Tracker::is_close_to_line(ipos_t delta, ipos_t end, ipos_t point, int *dist_squared, float *coefficient) {
    ipos_t point_to_end = end - point;
    int distance_to_end_squared = point_to_end.x * point_to_end.x + point_to_end.y * point_to_end.y + point_to_end.z * point_to_end.z;
    float distance_to_end = fast_sqrt(distance_to_end_squared, true);
    int delta_length_squared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
    int dummy;

    // If the point is not even close to the line, we omit the more expensive check.
    if (distance_to_end > fast_sqrt(delta_length_squared, true) + GRID_SIZE && !dist_squared)
        return false;

    // If the point is beyond the line end, check distance to end
    if (delta.x * point_to_end.x + delta.y * point_to_end.y + delta.z * point_to_end.z <= 0) { // dot(delta, point_to_end) negative?
        if (coefficient)
            *coefficient = 0;
        return (*(dist_squared ? dist_squared : &dummy) = point_to_end.x * point_to_end.x + point_to_end.y * point_to_end.y + point_to_end.z * point_to_end.z) <= GRID_SIZE * GRID_SIZE;
    }

    ipos_t point_to_start = point_to_end - delta;

    // If the point is beyond the line start, check distance to start
    if (delta.x * point_to_start.x + delta.y * point_to_start.y + delta.z * point_to_start.z >= 0) { // dot(delta, point_to_start) positive?
        if (coefficient)
            *coefficient = 1;
        return (*(dist_squared ? dist_squared : &dummy) = point_to_start.x * point_to_start.x + point_to_start.y * point_to_start.y + point_to_start.z * point_to_start.z) <= GRID_SIZE * GRID_SIZE;
    }

    // Calculate the area of the parallelogram that is spanned by line start, line end and the point
    ipos_t cross = {
        .x = delta.y * point_to_end.z - delta.z * point_to_end.y,
        .y = delta.z * point_to_end.x - delta.x * point_to_end.z,
        .z = delta.x * point_to_end.y - delta.y * point_to_end.x
    };
    int area_squared = cross.x * cross.x + cross.y * cross.y + cross.z * cross.z; 

    // The parallelogram's area divided by the length of it's base (the length of delta) is it's height, aka the minimum distance between the point and the line.
    if (dist_squared)
        *dist_squared = area_squared / delta_length_squared;

    bool is_close = area_squared <= GRID_SIZE * GRID_SIZE * delta_length_squared;
    
    // coefficient = length(end - closest-point-on-line) / length(end - start)
    if (coefficient && (dist_squared || is_close))
        *coefficient = fast_sqrt(distance_to_end_squared * delta_length_squared - area_squared, false) / (float)delta_length_squared;

    return is_close;
}


inline Tracker::graph_item_t Tracker::make_delta_item(ipos_t &from_pos, ipos_t &to_pos) {
    int delta_x = to_pos.x - from_pos.x;
    int delta_y = to_pos.y - from_pos.y;
    int delta_z = to_pos.z - from_pos.z;
    return ((delta_x & 0x1F) << 10) | ((delta_y & 0x1F) << 5) | ((delta_z & 0x1F) << 0);
}

inline Tracker::ipos_t Tracker::as_delta_item(graph_item_t &delta) {
    const int SHIFT_COUNT = CHAR_BIT * sizeof(int) - 5;
    return {
        .x = ((int)(delta << (SHIFT_COUNT - 10))) >> SHIFT_COUNT,
        .y = ((int)(delta << (SHIFT_COUNT - 5))) >> SHIFT_COUNT,
        .z = ((int)(delta << (SHIFT_COUNT - 0))) >> SHIFT_COUNT
    };
}

inline int16_t Tracker::as_signed_data_item(graph_item_t &item) {
    const int SHIFT_COUNT = CHAR_BIT * sizeof(int) - 14;
    return ((int)((item & 0x3FFF) << SHIFT_COUNT)) >> SHIFT_COUNT;
}

void Tracker::set_home(float x, float y, float z) {
    home_position.x = x;
    home_position.y = y;
    home_position.z = z;

    // Reset the distance-to-home of all nodes and links to infinity
    for (size_t index = 0; walk_forward(index, NULL, NULL, true);) {
        size_t node = get_node_or_null(index);
 
        if (node == index) {
            *get_node_payload_ptr(node) = make_data_item(UNSIGNED_DATA_ITEM_MAX);
            
            size_t link = node;
            while (walk_node(node, link)) {
                graph_item_t *link_attr_ptr = get_link_attr_ptr(link);
                link_attributes_t attributes(*link_attr_ptr);
                attributes.reset_dist_to_home_coef();
                *link_attr_ptr = attributes.as_attr_item();
            };
        }
    }

    // Determine which index on the graph is now closest to home.
    graph_home_index = get_closest_index(to_ipos(home_position), &graph_home_bias);
    TRACKER_DBG("tracker received new home position: (%.2f, %.2f, %.2f), index %zu, bias %.3f", x, y, z, graph_home_index, graph_home_bias);

    // The home position shall not be consumed by the line detection
    consolidate_index(graph_home_index);

    // Reset dirty nodes to contain only the new home position
    dirty_nodes_count = 0;
    controlled_area_end = graph_next_write - 1;
    controlled_area_end_distance = INFINITY;
    invalidate_node(graph_home_index);
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
            
            if (!is_delta_item(recent_path[index])) // If this item is a bumper, we shouldn't roll over this invalid position
                break; 

            head -= as_delta_item(recent_path[index]);
            rollback = is_close(head, position);
        } while ((index != recent_path_next_read) && !rollback);

        // If the preceding item would be a bumper, we shouldn't roll back to this invalid position
        if (rollback && index != recent_path_next_read)
            if (!is_delta_item(recent_path[(index ? index : RECENT_PATH_LENGTH) - 1]))
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
        
        graph_item_t item = make_delta_item(old_head, new_head);
        
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
    if (!is_delta_item(recent_path[last_index]))
        return false;
    
    // Roll back most recent position
    ipos_t head = to_ipos(recent_path_head);
    head -= as_delta_item(recent_path[last_index]);
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


size_t Tracker::get_cycle_head(size_t index) {
    // Generally, we scan the entire link cycle to find the lowest index, but often we may recognize the cycle head earlier.
    size_t initial_index = index;
    size_t lowest_index = index;
    do {
        if (index + 1 < graph_next_write && !is_data_item(graph[index + 1]))
            return index; // If a link item is not followed by a data item, it MUST be the cycle head 

        index = as_link_item(graph[index]);

        if (index < lowest_index)
            lowest_index = index;
    } while (index != initial_index);
    return lowest_index;
}


size_t Tracker::get_node_or_null(size_t index) {
    if (is_link_item(graph[index])) {
        return index; // Index points to a cycle head
    } else if (is_delta_item(graph[index])) {
        return 0; // Index points to a compact delta item
    }

    if (is_link_item(graph[index - REGULAR_LINK_SIZE + 1])) {
        index -= REGULAR_LINK_SIZE - 1; // Index points to a regular link
    } else if (is_link_item(graph[index - MASTER_LINK_SIZE + 1])) {
        index -= MASTER_LINK_SIZE - 1; // Index points to a master link
    } else {
        return 0; // Index points to a far jump
    }

    return get_cycle_head(index);
}


float Tracker::get_delta_length(size_t index) {
    float result = 0;
    walk_backward(index, NULL, &result, true);
    return result;
}


Tracker::ipos_t Tracker::get_intra_node_delta(size_t link1, size_t link2) {
    size_t node1 = get_node_or_null(link1);
    size_t node2 = get_node_or_null(link2);

    if (node1 != node2)
        PX4_WARN("intra-node distance requested for unrelated links (%zu and %zu)", link1, link2);

    if (!node1)
        PX4_WARN("intra-node distance requested for non-link indices (%zu and %zu)", link1, link2);

    ipos_t delta;
    walk_backward(node1, &delta, NULL, true);
    fpos_t fdelta = to_fpos(delta);

    // note that node1 != node2 by now
    float coef1 = link1 == node2 ? 0 : link_attributes_t{ *get_link_attr_ptr(link1) }.get_dist_to_node_coef();
    float coef2 = link2 == node2 ? 0 : link_attributes_t{ *get_link_attr_ptr(link2) }.get_dist_to_node_coef();

    return to_ipos(fdelta * coef1) - to_ipos(fdelta * coef2);
}


float Tracker::get_link_dist_to_node(size_t node, float node_delta_length, size_t index) {
    if (!node || index == node)
        return 0;
    
    link_attributes_t link_attributes(*get_link_attr_ptr(index));
    return link_attributes.get_dist_to_node_coef() * node_delta_length;
}


float Tracker::get_link_dist_to_home(size_t node, float node_delta_length, size_t index) {
    if (!node)
        return 0;

    if (index == graph_home_index)
        return graph_home_bias;
    
    uint16_t node_to_home = as_unsigned_data_item(*get_node_payload_ptr(node));

    if (node_to_home >= UNSIGNED_DATA_ITEM_MAX && node != graph_home_index)
        return INFINITY;

    if (index == node)
        return node_to_home;
    
    link_attributes_t link_attributes(*get_link_attr_ptr(index));

    if (node == graph_home_index)
        return std::abs(graph_home_bias - link_attributes.get_dist_to_home_coef() * node_delta_length);

    return (float)node_to_home + link_attributes.get_dist_to_home_coef() * node_delta_length;
}


bool Tracker::set_link_dist_to_home(size_t node, float node_delta_length, size_t index, float distance, bool backward) {
    if (!node)
        return false;

    // We consider the base the point on the line where we're just reducing the distance-to-home
    float base_dist_to_node = 0;
    if (index != node) {
        base_dist_to_node = node_delta_length * link_attributes_t{ *get_link_attr_ptr(index) }.get_dist_to_node_coef();
    } else if (backward) {
        base_dist_to_node = node_delta_length;
        distance -= node_delta_length;
    }

    // Update the true node-to-home distance
    graph_item_t *payload_ptr = get_node_payload_ptr(node);
    int old_node_dist_to_home = as_unsigned_data_item(*payload_ptr);
    int new_node_dist_to_home = std::min(round(distance + base_dist_to_node), old_node_dist_to_home);
    *payload_ptr = make_data_item(new_node_dist_to_home);

    // All other link distances are stored relative to the true node distance, so we need to adjust them accordingly
    float node_improvement = (old_node_dist_to_home - new_node_dist_to_home) / node_delta_length;
    bool significant_improvement = std::abs(node_improvement) >= 2;
    bool did_change_anything = false;

    // Update every link in the node (except the cycle head)
    index = node;
    while (walk_node(node, index)) {
        // Calculate the alternative distance-to-home for this link, if we would use the base point that was updated initially
        graph_item_t *link_attr_ptr = get_link_attr_ptr(index);
        link_attributes_t curr_link_attributes(*link_attr_ptr);
        float curr_dist_to_node = node_delta_length * curr_link_attributes.get_dist_to_node_coef();
        float alt_dist_to_home = distance + std::fabs(base_dist_to_node - curr_dist_to_node);
 
        // Handle change in node-to-home distance
        if (old_node_dist_to_home != new_node_dist_to_home) {
            if (significant_improvement)
                curr_link_attributes.reset_dist_to_home_coef();
            else
                curr_link_attributes.update_node_to_home(node_improvement);
        }

        // Update the attribute based on the alternative distance
        // todo: not sure if this is ALWAYS evaluated, even if the left value is already true 
        did_change_anything |= curr_link_attributes.decrease_dist_to_home_coef((alt_dist_to_home - new_node_dist_to_home) / node_delta_length);
        *link_attr_ptr = curr_link_attributes.as_attr_item();
    }

    did_change_anything |= old_node_dist_to_home != new_node_dist_to_home;
    
    // If we just updated a node (potentially inside the controlled area), make sure it is marked dirty
    if (did_change_anything)
        invalidate_node(node);

    return did_change_anything;
}


bool Tracker::is_same_pos(size_t index1, size_t index2) {
    if (index1 == index2)
        return true;

    size_t node1 = get_node_or_null(index1);
    size_t node2 = get_node_or_null(index2);
    return node1 && node1 == node2;
}


bool Tracker::walk_forward(size_t &index, ipos_t *delta, float *distance, bool *compact_delta) {
    if (index + 1 >= graph_next_write)
        return (*compact_delta = false);
    
    ipos_t local_delta;
    if (!delta)
        delta = &local_delta;

    graph_item_t item = graph[++index];

    if (is_data_item(item)) {
        delta->x = as_signed_data_item(item);
        delta->y = as_signed_data_item(graph[++index]);
        delta->z = as_signed_data_item(graph[++index]);
        item = graph[++index]; // fetch link or reserved field (must be a data item)
    }

    if (is_link_item(item))
        item = graph[as_link_item(item) + 2];
    
    if ((*compact_delta = is_delta_item(item))) {
        *delta = as_delta_item(item);
    } else {
        // ignore reserved item for now
    }

    if (distance)
        *distance += fast_sqrt(delta->x * delta->x + delta->y * delta->y + delta->z * delta->z, false);

    // If this element contains a link, make sure we point at the end of it
    if (index + 1 < graph_next_write && is_link_item(graph[index + 1])) {
        size_t link = get_cycle_head(index + 1);
        if (link != index + 1) {
            link = as_link_item(graph[link]);
            if (link == index + 1) {
                index += MASTER_LINK_SIZE; // go to end of master link
            } else {
                index += REGULAR_LINK_SIZE; // go to end of regular link
            }
        }
    }
    
    return true;
}

bool Tracker::walk_backward(size_t &index, ipos_t *delta, float *distance, bool *compact_delta) {
    if (index <= 0)
        return (*compact_delta = false);

    ipos_t local_delta;
    if (!delta)
        delta = &local_delta;

    graph_item_t item = graph[index--];

    // Skip over link, possibly fetching the evicted item
    if (is_link_item(item)) {
        item = graph[as_link_item(item) + 2]; // skip over cycle head
    } else if (is_data_item(item)) {
        if (is_link_item(graph[index])) {
            item = graph[(index -= REGULAR_LINK_SIZE) + 1]; // skip over regular link
        } else if (is_link_item(graph[index - 2])) {
            item = graph[(index -= MASTER_LINK_SIZE) + 1]; // skip over master link
        } // else the item is part of a far jump - don't skip it
    }

    // Fetch compact or far delta
    if ((*compact_delta = is_delta_item(item))) {
        *delta = as_delta_item(item);
    } else {
        index -= FAR_JUMP_SIZE - 1;
        delta->x = as_signed_data_item(graph[index + 1]);
        delta->y = as_signed_data_item(graph[index + 2]);
        delta->z = as_signed_data_item(graph[index + 3]);
        // "item" holds a reserved value - ignore it for now
    }

    if (distance)
        *distance += fast_sqrt(delta->x * delta->x + delta->y * delta->y + delta->z * delta->z, false);

    return true;
}


size_t Tracker::walk_far_forward(size_t &index, size_t checkpoint, float &interval_length) {
    while (walk_forward(index, NULL, &interval_length, false)) {
        uint16_t node = get_node_or_null(index);
        if (node)
            return node;
        else if (index == checkpoint)
            return 0;
    };

    return 0;
}


size_t Tracker::walk_far_backward(size_t &index, size_t checkpoint, float &interval_length) {
    while (walk_backward(index, NULL, &interval_length, false)) {
        uint16_t node = get_node_or_null(index);
        if (node)
            return node;
        else if (index == checkpoint)
            return 0;
    };

    return 0;
}


inline bool Tracker::walk_node(size_t node, size_t &index) {
    if (!node)
        return false;
    
    if (index == as_link_item(graph[node]) + MASTER_LINK_SIZE - 1) {
        index -= MASTER_LINK_SIZE - 1;
    } else if (index != node){
        index -= REGULAR_LINK_SIZE - 1;
    }

    index = as_link_item(graph[index]) + (index == node ? MASTER_LINK_SIZE : REGULAR_LINK_SIZE) - 1;
    return index - 1 != node;
}


void Tracker::push_graph(fpos_t &position) {
    if (graph_next_write > 4) {
        if (is_link_item(graph[4]) && as_link_item(graph[4]) != 9) {
            PX4_WARN("TRACKER INTEGRITY SUFFERED RECENTLY: %zu", as_link_item(graph[4]));
        }
    }

    ipos_t new_pos = to_ipos(position);

    // Skip most checks if the graph is empty.
    if (graph_next_write) {

        // Don't update if we're still close to the last known line
        float coefficient;
        if (is_close_to_line(graph_current_delta, graph_current_line_end, new_pos, NULL, &coefficient)) {
            graph_current_attributes = { coefficient, coefficient };
            if (graph_current_attributes.is_valid())
                return;
        }

        int search_bound = GRAPH_SEARCH_RANGE < graph_next_write ? graph_next_write - GRAPH_SEARCH_RANGE : 0;
        bool compact_delta;
        
        ipos_t head = graph_head;
        ipos_t delta;
        size_t prev_index = graph_next_write - 1;
        size_t next_index = graph_next_write - 1;
        bool inhibit_link = true;

        while (next_index > search_bound && walk_backward(next_index, &delta, NULL, &compact_delta)) {

            // Check if we're close to the line we just found (in case it is valid)
            if (compact_delta && is_close_to_line(delta, head, new_pos, NULL, &coefficient)) {
                graph_current_attributes = { coefficient, coefficient };
                TRACKER_DBG("I'm close to line %zu - %zu, with coefficient %f", next_index, prev_index, coefficient);

                // If the closeness-check tells us that we're at the beginning of the line, ignore it because we can't store a coefficient of 1.
                // If there is an adjacent line, we will find it later anyway.
                // The very first line we handle specially.
                if (!next_index && !graph_current_attributes.is_valid()) {
                    graph_current_attributes = { 0, 0 };
                    graph_current_delta = { .x = 0, .y = 0, .z = 0 };
                    graph_current_line_end = head;
                    graph_current_index = 0;
                    TRACKER_DBG("vehicle moved to the graph start");

                } else if (graph_current_attributes.is_valid()) {
                    graph_current_delta = delta;
                    graph_current_line_end = head;

                    consolidate_index(prev_index);

                    // If we were at the end of the graph, we append a delta and a link.
                    // However, if we just moved back by one position, we don't want to create a link there.
                    if (is_same_pos(graph_current_index, graph_next_write - 1)) {
                        if (!inhibit_link) {
                            push_compact_delta(graph_get_current_pos());
                            push_link(prev_index, graph_current_attributes);
                        }
                    }
 
                    graph_current_index = prev_index;
                    TRACKER_DBG("vehicle moved to already visited index %zu - %.3f * delta", graph_current_index, graph_current_attributes.get_dist_to_node_coef());
                    return;
                }
            }

            inhibit_link = prev_index == graph_next_write - 1;
            prev_index = next_index;
            head -= delta;
        }


        // If we were not at the end if the graph, we need to append a far jump and link it to the index we're coming from
        if (!is_same_pos(graph_current_index, graph_next_write - 1)) {
            ipos_t true_pos = graph_get_current_pos();
            TRACKER_DBG("insert jump after %zu (node %zu), because current index is %zu (node %zu, pos %d, %d, %d)", graph_next_write - 1, get_node_or_null(graph_next_write - 1), graph_current_index, get_node_or_null(graph_current_index), true_pos.x, true_pos.y, true_pos.z);
            push_far_delta(true_pos);
            push_link(graph_current_index, graph_current_attributes);
        }
    }


    // Add compact delta to current position
    push_compact_delta(new_pos);

    // Go to the new position
    graph_current_index = graph_next_write - 1;
    graph_current_delta = as_delta_item(graph[graph_current_index]);
    graph_current_line_end = graph_head;
    graph_current_attributes = { 0, 0 };
}


void Tracker::push_compact_delta(ipos_t destination) {
    aggeregate_line(&destination);

    if (graph_next_write + 1 > GRAPH_LENGTH)
        return;

    graph[graph_next_write++] = make_delta_item(graph_head, destination);
    graph_head = destination;
}


void Tracker::push_far_delta(ipos_t destination) {
    aggeregate_line(NULL);

    if (graph_next_write + 4 > GRAPH_LENGTH)
        return;

    graph[graph_next_write++] = make_data_item(destination.x - graph_head.x);
    graph[graph_next_write++] = make_data_item(destination.y - graph_head.y);
    graph[graph_next_write++] = make_data_item(destination.z - graph_head.z);
    graph[graph_next_write++] = make_data_item(0); // append reserved item
    graph_head = destination;
}


void Tracker::push_link(size_t index, link_attributes_t attributes) {
    if (index == 4)
        PX4_WARN("create node 4 to %zu (if master)", graph_next_write + 3);

    consolidate_index(index);
    aggeregate_line(NULL);

    if (graph_next_write + 4 > GRAPH_LENGTH)
        return;

	size_t node;

	if (is_data_item(graph[index])) {
	    if (is_link_item(graph[index - REGULAR_LINK_SIZE + 1]))
            index -= REGULAR_LINK_SIZE - 1;
        else if (is_link_item(graph[index - MASTER_LINK_SIZE + 1]))
	        index -= MASTER_LINK_SIZE - 1;
    }

	// Evict existing item by inserting a link to the current position (which is where the back-link will be placed)
	graph_item_t item = graph[index];
    TRACKER_DBG("evict item at %zu by putting link to %zu", index, graph_next_write);
	graph[index] = make_link_item(graph_next_write);

	if (!is_link_item(item)) {
        TRACKER_DBG("append master link");
        // Append master link
	    graph[graph_next_write++] = make_link_item(node = index); // new link item pointing to the cycle head
        graph[graph_next_write++] = attributes.as_attr_item(); // attributes
	    graph[graph_next_write++] = item; // evicted delta item
	    graph[graph_next_write++] = make_data_item(UNSIGNED_DATA_ITEM_MAX); // node payload
    } else {
        TRACKER_DBG("append regular link to head %zu", as_link_item(item));
        // Append regular link
	    graph[graph_next_write++] = item; // evicted link item
        graph[graph_next_write++] = attributes.as_attr_item(); // attributes
	    node = get_cycle_head(index);
    }
    
	// Make sure the node will be updated when required.
	invalidate_node(node);
}


void Tracker::aggeregate_line(ipos_t *new_pos) {
    ipos_t new_line_delta = new_pos ? (*new_pos - graph_head) : ipos_t{ .x = 0, .y = 0, .z = 0 };
    bool should_aggeregate = !new_pos;

    bool current_index_is_at_head = is_same_pos(graph_current_index, graph_next_write - 1);
    if (!current_index_is_at_head)
        consolidate_index(graph_current_index);

    // Don't proceed if there graph is empty
    if (!graph_next_write)
        return;

    ipos_t delta;
    ipos_t head = graph_head;
    size_t index = graph_next_write - 1;

    // Determine where the line (if any) should start from.
    while (index > line_detection_bound) {

        // Don't aggeregate beyond a node
        if (get_node_or_null(index))
            break;

        // Don't aggeregate beyond a far jump
        size_t prev_index = index;
        if (!walk_backward(index, &delta, NULL, false)) {
            index = prev_index;
            break;
        }

        head -= delta;
        new_line_delta += delta;

        // If expanding to the new point would make the delta too long, we need to aggeregate before it is too late
        if (is_too_large(new_line_delta)) {
            should_aggeregate = true;
            break;
        }
    }

    ipos_t line_start = head;
    line_detection_bound = index;
    // head does now coincide with line_detection_bound

    if (!should_aggeregate) {
        // Check if expanding the line to the new point would violate the recording accuracy for any point 
        while (walk_forward(index, &delta, NULL, false)) {
            head += delta;
            if (!is_close_to_line(new_line_delta, *new_pos, head, NULL, NULL)) {
                should_aggeregate = true;
                break;
            }
        }
    }

    if (line_detection_bound == graph_next_write - 1)
        should_aggeregate = false;

    // If we decided to aggeregate the recent deltas to a line, append the line delta item and go on from there
    if (should_aggeregate) {
        if (line_detection_bound + 1 < graph_next_write - 1)
            TRACKER_DBG("aggeregated %zu deltas into a line at %zu", graph_next_write - 1 - line_detection_bound, line_detection_bound + 1);
        
        graph_next_write = line_detection_bound + 1;
        graph[graph_next_write++] = make_delta_item(line_start, graph_head);
        //if (current_index_is_at_head)
        //    graph_current_index = graph_next_write - 1;
        line_detection_bound++; // any later line detection must not remove the end of this line, so we move the search bound there
    }
}


void Tracker::consolidate_index(size_t index) {
    if (line_detection_bound < index)
        line_detection_bound = index;
}


void Tracker::invalidate_node(size_t node) {
    // No need to cache the node if it's invalid (0) or outside of the controlled area
    if (!node || node >= controlled_area_end)
        return;

    TRACKER_DBG("  did update node inside controlled area: %zu", node);
    bool unique = true;
    uint16_t insert_index;
    for (insert_index = 0; insert_index < dirty_nodes_count; insert_index++) {
        if (dirty_nodes[insert_index] == node)
            unique = false;
        if (dirty_nodes[insert_index] >= node)
            break;
    }

    // No need to cache the same node more than once
    if (!unique)
        return;

    // If we evict a node from the dirty list, make sure that the controlled area gets cropped accordingly 
    if (dirty_nodes_count == DIRTY_NODES_BUFFER_SIZE) {
        uint16_t evicted_node = dirty_nodes[DIRTY_NODES_BUFFER_SIZE - 2];
        if (evicted_node < controlled_area_end) {
            controlled_area_end = evicted_node;
            controlled_area_end_distance = as_unsigned_data_item(*get_node_payload_ptr(evicted_node));
            TRACKER_DBG("  crop controlled area to %zu", controlled_area_end);
        }
    } else {
        dirty_nodes_count++;
    }

    memcpy(dirty_nodes + insert_index + 1, dirty_nodes + insert_index, (dirty_nodes_count - insert_index - 1) * sizeof(dirty_nodes[0]));
    dirty_nodes[insert_index] = node;
}


void Tracker::refresh_distances() {
    size_t walk_origin = 0;
    size_t walk_origin_node = 0;
    float walk_origin_node_delta_length = 0;
    bool walking_forward = true;

    // Only log if we're gonna do something
    bool should_log = controlled_area_end + 1 < graph_next_write || dirty_nodes_count;
    if (should_log)
        TRACKER_DBG("calculating distances...");

    // This loop is guaranteed to terminate.
    // Proof outline:
    //  The loop terminates when the set of dirty nodes becomes empty. This set is only expanded, if there
    //  is a reduction in some node distance. Also, at the beginning of every iteration, the set is reduced
    //  by one node. A node distance cannot be increased. There are no intervals with negative length, so
    //  every node distance will eventually reach a lower bound.
    //  When this happens, the set of dirty nodes is no longer expanded and thus reduced in every iteration.
    //  It follows that the set will eventually become empty, which means that the loop terminates. 
    for (;;) {

        // Determine which interval we want to walk next
        if (!walking_forward) {
            walking_forward = true;
        } else {
            walking_forward = false;

            // If we're walking a node, walk_node takes care of everything, else we need to determine where to look next
            if (!walk_node(walk_origin_node, walk_origin)) {
                if (dirty_nodes_count) {
                    walk_origin = dirty_nodes[0];
                    memcpy(dirty_nodes, dirty_nodes + 1, --dirty_nodes_count * sizeof(dirty_nodes[0]));
                } else if (controlled_area_end + 1 < graph_next_write) {
                    walk_origin = controlled_area_end;
                } else {
                    break; // nothing more to be updated - we're done!
                }

                walk_origin_node = get_node_or_null(walk_origin);
                walk_origin_node_delta_length = get_delta_length(walk_origin_node);

                // If we're not looking at a node head, don't walk the node.
                // If we moreover continue from the controlled area end, we don't have to walk backwards anymore.
                if (walk_origin != walk_origin_node) {
                    walk_origin_node = 0;
                    walk_origin_node_delta_length = 0;
                    walking_forward = walk_origin == controlled_area_end;
                }
            }
        }

        // Walk along the interval until we encounter the next node or an end
        TRACKER_DBG("  walking interval from %zu (node: %zu) %s...", walk_origin, walk_origin_node, walking_forward ? "forward" : "backward");
        if (!walk_origin && controlled_area_end) {
            PX4_WARN("tracker integrity became questionable");
            dump_graph();
        }

        // Walk forward or backward until we reach the next interesting index (node, jump, home or end-of-graph).
        size_t walk_end = walk_origin;
        size_t walk_end_node = 0;
        float interval_length = 0;
        if (walking_forward)
            walk_end_node = walk_far_forward(walk_end, graph_home_index, interval_length);
        else
            walk_end_node = walk_far_backward(walk_end, graph_home_index, interval_length);
        
        float walk_end_node_delta_length = get_delta_length(walk_end_node);
        TRACKER_DBG("  interval from %zu (node: %zu, l: %.3f) to %zu (node: %zu, l: %.3f): length %f", walk_origin, walk_origin_node, walk_origin_node_delta_length, walk_end, walk_end_node, walk_end_node_delta_length, interval_length);

        // Retrieve the distance to home at the beginning and end of the interval we just visited
        float dist_at_interval_start = walk_origin_node ? get_link_dist_to_home(walk_origin_node, walk_origin_node_delta_length, walk_origin) : controlled_area_end_distance;
        if (walk_origin == graph_home_index)
            dist_at_interval_start = walking_forward ? graph_home_bias : -graph_home_bias;

        float dist_at_interval_end = walk_end_node ? get_link_dist_to_home(walk_end_node, walk_end_node_delta_length, walk_end) : (dist_at_interval_start + interval_length);
        if (walk_end == graph_home_index)
            dist_at_interval_end = walking_forward ? -graph_home_bias : graph_home_bias;

        // Register expansion of the controlled area if applicable
        if (walking_forward && walk_origin == controlled_area_end) {
            controlled_area_end = walk_end;
            controlled_area_end_distance = dist_at_interval_end;
            TRACKER_DBG("  expand controlled area to %zu", controlled_area_end);
        }

        // Update node at beginning or end of the current interval, if the interval enables a shorter path
        float alt_distance = std::min(dist_at_interval_start, dist_at_interval_end) + interval_length;
        if (walk_origin_node && dist_at_interval_start > alt_distance && walk_origin_node != graph_home_index) {
            if (set_link_dist_to_home(walk_origin_node, walk_origin_node_delta_length, walk_origin, alt_distance, !walking_forward)) {
                TRACKER_DBG("  distance at interval start improved to %f", alt_distance);
                dump_nodes();
            }
        } else if (walk_end_node && dist_at_interval_end > alt_distance && walk_end_node != graph_home_index) {
            if (set_link_dist_to_home(walk_end_node, walk_end_node_delta_length, walk_end, alt_distance, walking_forward)) {
                TRACKER_DBG("  distance at interval end improved to %f", alt_distance);
                dump_nodes();
            }
        } else {
            TRACKER_DBG("  nothing is improving through this interval");
        }
    }

#ifdef DEBUG_TRACKER
    if (should_log)
        dump_nodes();
#endif
}


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


size_t Tracker::get_closest_index(ipos_t position, float *bias) {
    bool compact_delta;
    ipos_t head = graph_head;
    ipos_t delta;
    size_t prev_index = graph_next_write - 1;
    size_t next_index = graph_next_write - 1;

    *bias = 0;
    size_t best_index = 0;
    int best_distance = INT_MAX;

    while (walk_backward(next_index, &delta, NULL, &compact_delta)) {
        if (compact_delta) {
            int distance;
            float coefficient;

            is_close_to_line(delta, head, position, &distance, &coefficient);
            
            if (best_distance > distance) {
                best_distance = distance;
                best_index = prev_index;
                *bias = fast_sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z, false) * coefficient;
            }
        }

        prev_index = next_index;
        head -= delta;
    }

    return best_index;
}


size_t Tracker::calc_return_path(size_t index, float bias, bool &move_forward, bool &move_backward, bool &did_select_link) {
    // Special handling if we're already at the home node
    if (index == graph_home_index) { // todo: consider that graph_home_index may be on a node
        move_forward = move_backward = false;
        if (std::abs(bias - graph_home_bias) > FLT_EPSILON) {
            move_backward = true;
            TRACKER_DBG("calc return path, I'm almost home");
        } else {  
            TRACKER_DBG("calc return path, but I'm already home");
        }
        return index;
    }

    // Detect if the current position is at a node (in which case we'd have to walk the cycle)
    size_t walk_origin_node = get_node_or_null(index); // will be 0 if there is no node
    TRACKER_DBG("calc return path from %zu (node %zu)", index, walk_origin_node);

    // If have a movement direction, don't change it unless we're at a node and haven't yet selected the best link
    if ((!walk_origin_node || did_select_link) && (move_forward || move_backward)) {
        did_select_link = false;
        return index;
    }

    // Make sure all nodes and links have a valid distance-to-home
    refresh_distances();

    float dist_forward = 0, dist_backward = 0;

    if (walk_origin_node && !did_select_link) {
        // Select a link

        // By default, we assume that it's best to walk backwards on the line of the node head
        float best_distance = INFINITY;
        size_t best_link = index;

        index = walk_origin_node;
        float walk_origin_node_delta_length = get_delta_length(walk_origin_node);

        // Look at all the links in the current node and select the one that is closest to home.
        do {
            get_distance_to_home(index, index == walk_origin_node ? bias : 0, dist_forward, dist_backward);

            if (index != walk_origin_node) {
                float dist_to_link = std::abs(bias - get_link_dist_to_node(walk_origin_node, walk_origin_node_delta_length, index));
                dist_forward += dist_to_link;
                dist_backward += dist_to_link;
            }

            float link_distance = std::min(dist_forward, dist_backward);
            TRACKER_DBG("link %zu has distance %f to home", index, link_distance);
            if (best_distance > link_distance) {
                best_distance = link_distance;
                best_link = index;
                move_backward = !(move_forward = dist_forward < dist_backward);
            }
        } while (walk_node(walk_origin_node, index));

        // If no link has a known path to home, try the earliest link (i.e. the cycle head), or abort.
        if (best_distance == INFINITY) {
            move_forward = move_backward = false;
            if (best_link == walk_origin_node) {
                dump_graph();
                PX4_WARN("I don't see any path home from node %zu", walk_origin_node);
            }
        }

        did_select_link = true;
        return best_link;
    }


    // Select a direction

    did_select_link = false;

    // Calculate the distances for both directions
    get_distance_to_home(index, bias, dist_forward, dist_backward);
    TRACKER_DBG("distance from %zu is: %f forward, %f backward", index, dist_forward, dist_backward);
    
    // Note that we prefer to move backward in case of equal cost
    move_backward = !(move_forward = dist_forward < dist_backward);

    // Only if both directions have unknown cost AND walking backwards wouldn't take us to a node, we either move forward (if there's a node there) or give up.
    if (isinf(dist_forward) && isinf(dist_backward)) {
        move_backward = move_forward = false;
        PX4_WARN("I don't see any path home from graph index %zu.", index);
    }

    return index;
}


bool Tracker::get_current_pos(pos_handle_t &pos, float &x, float &y, float &z) {
    ipos_t true_pos = graph_get_current_pos();
    x = true_pos.x;
    y = true_pos.y;
    z = true_pos.z;

    ipos_t delta = graph_current_line_end - true_pos;

    pos = {
        .index = graph_current_index,
        .bias = fast_sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z, false),
        .position = graph_current_line_end,
        .did_move_forward = false,
        .did_move_backward = false,
        .did_select_link = false
    };

    consolidate_index(pos.index);
    return true;
}


bool Tracker::get_path_to_home(pos_handle_t &pos, float &x, float &y, float &z) {
    // Calculate where to go next
    size_t new_index = calc_return_path(pos.index, pos.bias, pos.did_move_forward, pos.did_move_backward, pos.did_select_link);

    // If we change the link within a node we need to move along the line.
    if (pos.did_select_link) {
        ipos_t delta = get_intra_node_delta(pos.index, new_index);
        TRACKER_DBG("intra-node delta: (%d, %d, %d) + (%d, %d, %d)", pos.position.x, pos.position.y, pos.position.z, delta.x, delta.y, delta.z);
        pos.index = new_index;
        pos.position += delta;


        if (get_node_or_null(new_index) == new_index && pos.did_move_backward) {
            // If we need to walk to the beginning of the line, we act as if we're not at a node.
            TRACKER_DBG("I will move to the beginning of node %zu (which ends at %d, %d, %d)", new_index, pos.position.x, pos.position.y, pos.position.z);
            pos.did_select_link = false;
        } else if (delta == ipos_t{ .x = 0, .y = 0, .z = 0 }) {
            // If the link change does not result in a new position, we try another update.
            TRACKER_DBG("incomplete position change... second chance");
            calc_return_path(pos.index, 0, pos.did_move_forward, pos.did_move_backward, pos.did_select_link);
        } else {
            TRACKER_DBG("link change resulted in position change (%d, %d, %d)", delta.x, delta.y, delta.z);
        }
    }

    pos.bias = 0;
    ipos_t delta = { .x = 0, .y = 0, .z = 0 };

    // Apply move-forward-or-backward-instruction
    if (pos.did_select_link) {
        // already handled
    } else if (pos.did_move_forward) {
        walk_forward(pos.index, &delta, NULL, false);
        pos.position += delta;

        if (pos.index == graph_home_index) 
            pos.bias = graph_home_bias;
    } else if (pos.did_move_backward) {
        if (pos.index == graph_home_index) {
            size_t dummy = pos.index;
            walk_backward(dummy, &delta, NULL, false);
            pos.bias = graph_home_bias;
        } else {
            walk_backward(pos.index, &delta, NULL, false);
            pos.position -= delta;
        }
    } else {
        return false;
    }

    x = pos.position.x;
    y = pos.position.y;
    z = pos.position.z;

    if (pos.bias) {
        float coef = pos.bias / fast_sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z, false);
        if (!isinf(coef)) {
            x -= delta.x * coef;
            y -= delta.y * coef;
            z -= delta.z * coef;
        }
    }

    consolidate_index(pos.index);
    return true;
}


bool Tracker::is_same_pos(pos_handle_t &pos1, pos_handle_t &pos2) {
    return is_same_pos(pos1.index, pos2.index);
}


bool Tracker::is_close_to_pos(pos_handle_t &pos) {
    return is_close(pos.position, to_fpos(graph_get_current_pos()));
}


bool Tracker::is_close_to_home(pos_handle_t &pos) {
    fpos_t adjusted_home_pos = home_position;
    adjusted_home_pos.z = pos.position.z;
    return is_close(pos.position, adjusted_home_pos);
}


void Tracker::dump_recent_path() {
    if (recent_path_next_read == RECENT_PATH_LENGTH) {
        PX4_INFO("recent path empty");
        return;
    }
    
    PX4_INFO("recent path (element size: %zu):", sizeof(graph_item_t));
    size_t recent_path_length = (recent_path_next_write + RECENT_PATH_LENGTH - recent_path_next_read - 1) % RECENT_PATH_LENGTH + 1;
    PX4_INFO("  length: %zu (read-ptr: %zu, write-ptr: %zu)", recent_path_length, recent_path_next_read, recent_path_next_write);
    
    size_t index = recent_path_next_write;
    ipos_t head = to_ipos(recent_path_head);
    
    do {
        PX4_INFO("  (%d, %d, %d)", head.x, head.y, head.z);
        
        index = (index ? index : RECENT_PATH_LENGTH) - 1;

        if (!is_delta_item(recent_path[index]))
            PX4_INFO("ignore the last position");

        head -= as_delta_item(recent_path[index]);
    } while (index != recent_path_next_read);
}


void Tracker::dump_graph() {
    if (!graph_next_write) {
        PX4_INFO("full path empty");
        return;
    }

    PX4_INFO("full path (length: %zu):", graph_next_write);

    ipos_t delta = { .x = 0, .y = 0, .z = 0 };
    ipos_t head = graph_head;

    size_t index = graph_next_write - 1;
    do {
        head -= delta;

        size_t node = get_node_or_null(index);
        bool is_cycle_head = node && index == node;
        bool is_cycle_master = is_data_item(graph[index]) && !is_link_item(graph[index - REGULAR_LINK_SIZE + 1]) && is_link_item(graph[index - MASTER_LINK_SIZE + 1]);
        bool is_regular_link = is_data_item(graph[index]) && is_link_item(graph[index - REGULAR_LINK_SIZE + 1]);
        
        size_t delta_item_index;
        if (is_cycle_head)
            delta_item_index = as_link_item(graph[node]) + 2;
        else if (is_cycle_master)
            delta_item_index = index - MASTER_LINK_SIZE;
        else if (is_regular_link)
            delta_item_index = index - REGULAR_LINK_SIZE;
        else
            delta_item_index = index;

        PX4_INFO("  %zu: (%d, %d, %d)%s%s%s%s",
            index, head.x, head.y, head.z,
            is_cycle_head ? ", head" : is_cycle_master ? ", master" : is_regular_link ? ", link" : "",
            is_data_item(graph[delta_item_index]) ? ", jump" : "",
            graph_current_index == index ? " <= current index" : "",
            graph_home_index == index ? " <= home" : "");
    } while (walk_backward(index, &delta, NULL, true));

    dump_nodes();
}

void Tracker::dump_nodes() {
    PX4_INFO("nodes:");
    for (size_t node = 0; walk_forward(node, NULL, NULL, true);) {
        if (get_node_or_null(node) != node)
            continue;
        
        PX4_INFO("  node %zu", node);
        size_t link = node;
        bool is_cycle_master = false;
        do {
            PX4_INFO("    link %zu: coef %.3f, dist to home %.3f%s",
                link,
                link == node ? 0 : link_attributes_t{ *get_link_attr_ptr(link) }.get_dist_to_node_coef(),
                get_link_dist_to_home(node, get_delta_length(node), link),
                is_cycle_master ? " (master)" : "");
            is_cycle_master = link == node;
        } while (walk_node(node, link));
    }
}

void Tracker::dump_path_to_home() {
    pos_handle_t pos;

    float x, y, z;
    if (!get_current_pos(pos, x, y, z)) {
        PX4_INFO("shortest path to home unavailable");
        return;
    }

    PX4_INFO("shortest path to home:");

    do {
        PX4_INFO("  %zu: (%d, %d, %d)", pos.index, (int)x, (int)y, (int)z);
    } while (get_path_to_home(pos, x, y, z));
}
