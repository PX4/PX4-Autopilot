
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
    return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z < ACCURACY * ACCURACY + 0.001;
}

inline bool Tracker::is_close(ipos_t pos1, fpos_t pos2) {
    return is_close(to_fpos(pos1), pos2);
}


bool Tracker::is_close_to_line(ipos_t delta, ipos_t end, ipos_t point, float *coefficient) {
    ipos_t point_to_end = end - point;
    int distance_to_end_squared = point_to_end.x * point_to_end.x + point_to_end.y * point_to_end.y + point_to_end.z * point_to_end.z;
    float distance_to_end = fast_sqrt(distance_to_end_squared, true);
    int delta_length_squared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;

    // If the point is not even close to the line, we omit the more expensive check.
    if (distance_to_end > fast_sqrt(delta_length_squared, true) + ACCURACY)
        return false;

    // If the point is beyond the line end, check distance to end
    if (delta.x * point_to_end.x + delta.y * point_to_end.y + delta.z * point_to_end.z <= 0) { // dot(delta, point_to_end) negative?
        if (coefficient)
            *coefficient = 0;
        return point_to_end.x * point_to_end.x + point_to_end.y * point_to_end.y + point_to_end.z * point_to_end.z <= ACCURACY * ACCURACY;
    }

    ipos_t point_to_start = point_to_end - delta;

    // If the point is beyond the line start, check distance to start
    if (delta.x * point_to_start.x + delta.y * point_to_start.y + delta.z * point_to_start.z >= 0) { // dot(delta, point_to_start) positive?
        if (coefficient)
            *coefficient = 1;
        return point_to_start.x * point_to_start.x + point_to_start.y * point_to_start.y + point_to_start.z * point_to_start.z <= ACCURACY * ACCURACY;
    }

    // Calculate the area of the parallelogram that is spanned by line start, line end and the point
    ipos_t cross = {
        .x = delta.y * point_to_end.z - delta.z * point_to_end.y,
        .y = delta.z * point_to_end.x - delta.x * point_to_end.z,
        .z = delta.x * point_to_end.y - delta.y * point_to_end.x
    };
    int area_squared = cross.x * cross.x + cross.y * cross.y + cross.z * cross.z; 

    // The parallelogram's area divided by the length of it's base (the length of delta) is it's height, aka the minimum distance between the point and the line.
    if (area_squared <= ACCURACY * ACCURACY * delta_length_squared) {
        // coefficient = | end - closest-point-on-line | / | end - start |
        if (coefficient)
            *coefficient = fast_sqrt(distance_to_end_squared * delta_length_squared - area_squared, false) / (float)delta_length_squared;
        return true;
    }
    return false;
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

void Tracker::set_home(home_position_s *position) {
    home_position.x = position->x;
    home_position.y = position->y;
    home_position.z = position->z;

    // Reset all distances to max value
    for (size_t index = 1; index < graph_next_write; index++)
        if (get_node_or_null(index) == index)
            graph[as_link_item(graph[index]) + 2] = make_data_item(0xFFFF);

    // Force fresh evaluation of all node-to-home distances
    visited_area_end = 0;
    unvisited_nodes_count = 0;
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

    ipos_t delta;
    walk_backward(node1, &delta, NULL, true);
    fpos_t fdelta = to_fpos(delta);

    float coef1 = link1 == node1 ? 0 : link_attributes_t{ *get_link_attr_ptr(link1) }.get_dist_to_node_coef();
    float coef2 = link2 == node1 ? 0 : link_attributes_t{ *get_link_attr_ptr(link2) }.get_dist_to_node_coef();
    return to_ipos(fdelta * coef1) - to_ipos(fdelta * coef2);
}


float Tracker::get_link_dist_to_home(size_t node, float node_delta_length, size_t index) {
    if (!node)
        return 0;
    
    uint16_t node_to_home = as_unsigned_data_item(*get_node_payload_ptr(node));

    if (node_to_home >= UNSIGNED_DATA_ITEM_MAX)
        return INFINITY;

    if (index == node)
        return node_to_home;
    
    link_attributes_t link_attributes(*get_link_attr_ptr(index));

    return (float)node_to_home + link_attributes.get_dist_to_home_coef() * node_delta_length;
}


bool Tracker::set_link_dist_to_home(size_t node, float node_delta_length, size_t index, float distance) {
    if (!node)
        return false;

    float base_dist_to_node = 0;
    if (index != node)
        base_dist_to_node = node_delta_length * link_attributes_t{ *get_link_attr_ptr(index) }.get_dist_to_node_coef();

    // The node cycle head gets special handling
    graph_item_t *payload_ptr = get_node_payload_ptr(node);
    int old_node_dist_to_home = as_unsigned_data_item(*payload_ptr);
    int new_node_dist_to_home = std::min(round(distance + base_dist_to_node), old_node_dist_to_home);
    *payload_ptr = make_data_item(new_node_dist_to_home);

    float node_improvement = (old_node_dist_to_home - new_node_dist_to_home) / node_delta_length;
    bool significant_improvement = std::abs(node_improvement) >= 2;
    bool did_change_anything = false;

    // Update every link in the node (except the cycle head)
    index = node;
    while (walk_node(node, index)) {
        // Calculate the alternative distance-to-home for this link, if we would use the link that was updated initially
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
    
    // If we just updated a node (potentially inside the already visited area), make sure it is marked unvisited
    if (did_change_anything)
        unvisit_node(node);

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
    ipos_t new_pos = to_ipos(position);

    // Skip most checks if the graph is empty.
    if (graph_next_write) {

        // Don't update if we're still close to the last known line
        float coefficient;
        if (is_close_to_line(graph_current_delta, graph_current_line_end, new_pos, &coefficient)) {
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
            if (compact_delta && is_close_to_line(delta, head, new_pos, &coefficient)) {
                graph_current_attributes = { coefficient, coefficient };

                // If the closeness-check tells us that we're at the beginning of the line, ignore it because we can't store a coefficient of 1.
                // If there is an adjacent line, we will find it later anyway. 
                if (graph_current_attributes.is_valid()) {
                    graph_current_delta = delta;
                    graph_current_line_end = head;

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
    if (graph_next_write + 1 > GRAPH_LENGTH)
        return;

    graph[graph_next_write++] = make_delta_item(graph_head, destination);
    graph_head = destination;
}


void Tracker::push_far_delta(ipos_t destination) {
    if (graph_next_write + 4 > GRAPH_LENGTH)
        return;

    graph[graph_next_write++] = make_data_item(destination.x - graph_head.x);
    graph[graph_next_write++] = make_data_item(destination.y - graph_head.y);
    graph[graph_next_write++] = make_data_item(destination.z - graph_head.z);
    graph[graph_next_write++] = make_data_item(0); // append reserved item
    graph_head = destination;
}


void Tracker::push_link(size_t index, link_attributes_t attributes) {
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
	graph[index] = make_link_item(graph_next_write);

	if (!is_link_item(item)) {
        // Append master link
	    graph[graph_next_write++] = make_link_item(node = index); // new link item pointing to the cycle head
        graph[graph_next_write++] = attributes.as_attr_item(); // attributes
	    graph[graph_next_write++] = item; // evicted delta item
	    graph[graph_next_write++] = make_data_item(UNSIGNED_DATA_ITEM_MAX); // node payload
    } else {
        // Append regular link
	    graph[graph_next_write++] = item; // evicted link item
        graph[graph_next_write++] = attributes.as_attr_item(); // attributes
	    node = get_cycle_head(index);
    }
    
	// Make sure the node will be visited when required.
	unvisit_node(node);
}


void Tracker::unvisit_node(size_t node) {
    // No need to cache the node if it's invalid or outside of the controlled area
    if (!node || node >= visited_area_end)
        return;

    TRACKER_DBG("  did update node inside controlled area: %zu", node);
    bool unique = true;
    uint16_t insert_index;
    for (insert_index = 0; insert_index < unvisited_nodes_count; insert_index++) {
        if (unvisited_nodes[insert_index] == node)
            unique = false;
        if (unvisited_nodes[insert_index] >= node)
            break;
    }

    // No need to cache the same node more than once
    if (!unique)
        return;

    // If we evict a node from the unvisited list, make sure that the visited area gets cropped accordingly 
    if (unvisited_nodes_count == UNVISITED_NODES_BUFFER_SIZE) {
        uint16_t evicted_node = unvisited_nodes[UNVISITED_NODES_BUFFER_SIZE - 2];
        if (evicted_node < visited_area_end) {
            visited_area_end = evicted_node;
            visited_area_end_distance = as_unsigned_data_item(*get_node_payload_ptr(evicted_node));
            TRACKER_DBG("  crop controlled area to %zu", visited_area_end);
        }
    } else {
        unvisited_nodes_count++;
    }

    memcpy(unvisited_nodes + insert_index + 1, unvisited_nodes + insert_index, (unvisited_nodes_count - insert_index - 1) * sizeof(unvisited_nodes[0]));
    unvisited_nodes[insert_index] = node;
}


void Tracker::calc_distances() {
    // todo: respect home index != 0
    size_t walk_origin = 0;
    size_t walk_origin_node = 0;
    float walk_origin_node_delta_length = 0;
    bool walking_forward = true;

    bool should_log = visited_area_end + 1 < graph_next_write || unvisited_nodes_count;
    if (should_log)
        TRACKER_DBG("calculating distances...");

    // Continue as long as there's an unvisited area or unvisited nodes
    while (visited_area_end + 1 < graph_next_write || unvisited_nodes_count) {
        // Determine which interval we want to walk next
        if (!walking_forward) {
            walking_forward = true;
        } else {
            walking_forward = false;
            if (walk_node(walk_origin_node, walk_origin)) {
                // nothing more todo: walk_node takes care of everything
            } else if (unvisited_nodes_count) {
                walk_origin = walk_origin_node = unvisited_nodes[0];
                walk_origin_node_delta_length = get_delta_length(walk_origin_node);
                memcpy(unvisited_nodes, unvisited_nodes + 1, --unvisited_nodes_count * sizeof(unvisited_nodes[0]));
            } else {
                // If we continue at the end of the visited area, it's only neccessary to walk the node if we continue at a cycle head.
                // (because this can happen if it was evicted from the unvisited nodes buffer)
                // Else, it is sufficient to walk forward and not look into the node
                walk_origin = walk_origin_node = get_node_or_null(visited_area_end);
                walk_origin_node_delta_length = get_delta_length(walk_origin_node);
                if (!walk_origin_node || walk_origin_node != visited_area_end) {
                    walk_origin = visited_area_end;
                    walk_origin_node = 0;
                    walk_origin_node_delta_length = 0;
                    walking_forward = true;
                }
            }
        }

        // Walk along the interval until we encounter the next node or an end
        TRACKER_DBG("  walking interval from %zu (node: %zu) %s...", walk_origin, walk_origin_node, walking_forward ? "forward" : "backward");
        if (!walk_origin && visited_area_end) {
            PX4_WARN("tracker integrity became questionable");
            dump_graph();
        }

        // Walk forward or backward until we reach the next interesting index (node, jump, home or end-of-graph).
        size_t index = walk_origin;
        uint16_t node = 0;
        float interval_length = 0;
        if (walking_forward)
            node = walk_far_forward(index, graph_home_index, interval_length);
        else
            node = walk_far_backward(index, graph_home_index, interval_length);
        
        float node_delta_length = get_delta_length(node);
        TRACKER_DBG("  interval from %zu (node: %zu, l: %.3f) to %zu (node: %zu, l: %.3f): length %f", walk_origin, walk_origin_node, walk_origin_node_delta_length, index, node, node_delta_length, interval_length);

        // Retrieve the distance to home at the beginning and end of the interval we just visited
        float dist_at_interval_start = walk_origin_node ? get_link_dist_to_home(walk_origin_node, walk_origin_node_delta_length, walk_origin) : visited_area_end_distance;
        float dist_at_interval_end = node ? get_link_dist_to_home(node, node_delta_length, index) : (index == graph_home_index ? 0 : (dist_at_interval_start + interval_length));

        // Register expansion of the visited area if applicable
        if (walking_forward && walk_origin == visited_area_end) {
            visited_area_end = index;
            visited_area_end_distance = dist_at_interval_end;
            TRACKER_DBG("  expand controlled area to %zu", visited_area_end);
        }

        // Update node at beginning or end of the current interval, if the interval enables a shorter path
        float alt_distance = std::min(dist_at_interval_start, dist_at_interval_end) + interval_length;
        if (walk_origin_node && dist_at_interval_start > alt_distance) {
            if (set_link_dist_to_home(walk_origin_node, walk_origin_node_delta_length, walk_origin, alt_distance)) {
                TRACKER_DBG("  distance at interval start improved to %f", alt_distance);
                dump_nodes();
            }
        } else if (node && dist_at_interval_end > alt_distance) {
            if (set_link_dist_to_home(node, node_delta_length, index, alt_distance)) {
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


size_t Tracker::calc_return_path(size_t index, float bias, bool &move_forward, bool &move_backward) {
    if (index == graph_home_index) {
        // todo: consider that home_index may be on a node
        move_forward = move_backward = false;
        return index;
    }

    // Detect if the current position is at a node (in which case we'd have to walk the cycle)
    size_t walk_origin_node = get_node_or_null(index); // will be 0 if there is no node
    TRACKER_DBG("calc return path from %zu (node %zu)", index, walk_origin_node);

    // If we're in the middle of an interval and already have a movement direction, don't change it
    if (!walk_origin_node && (move_forward || move_backward))
        return index;

    // Make sure all nodes and links have a valid distance-to-home
    calc_distances();

    // Depending on the circumstances, either select a link or a direction.
    if (!walk_origin_node || (move_forward && move_backward)) {
        // Select a direction

        // Calculate the distance if walking forward from index
        float dist_forward = bias;
        size_t index_forward = index;
        size_t node_forward = walk_far_forward(index_forward, graph_home_index, dist_forward);
        if (node_forward)
            dist_forward += get_link_dist_to_home(node_forward, get_delta_length(node_forward), index_forward);
        else if (index_forward != graph_home_index)
            dist_forward = INFINITY;
        
        // Calculate the distance if walking backward from index
        float dist_backward = -bias;
        size_t index_backward = index;
        size_t node_backward = walk_far_backward(index_backward, graph_home_index, dist_backward);
        if (node_backward)
            dist_backward += get_link_dist_to_home(node_backward, get_delta_length(node_backward), index_backward);
        else if (index_backward != graph_home_index)
            dist_backward = INFINITY;

        TRACKER_DBG("distance from %zu is: %f forward, %f backward", index, dist_forward, dist_backward);

        // Note that we prefer to move backward in case of equal cost
        move_backward = !(move_forward = dist_forward < dist_backward);
        
        // Only if both directions have unknown cost AND walking backwards wouldn't take us to a node, we either move forward (if there's a node there) or give up.
        if (isinf(dist_forward) && isinf(dist_backward) && !node_backward) {
            if (node_forward) {
                move_backward = !(move_forward = true);
            } else {
                move_backward = move_forward = false;
                PX4_WARN("I don't see any path home from graph index %zu.", index);
            }
        }

        return index;

    } else {
        // Select a link

        float best_distance = INFINITY;
        size_t best_link = index;

        index = walk_origin_node;
        float walk_origin_node_delta_length = get_delta_length(walk_origin_node);

        // Look at all the links in the current node and select the one that is closest to home.
        do {
            float link_distance = get_link_dist_to_home(walk_origin_node, walk_origin_node_delta_length, index);
            if (best_distance > link_distance) {
                best_distance = link_distance;
                best_link = index;
            }
        } while (walk_node(walk_origin_node, index));

        move_forward = move_backward = true;

        // If no link is a known path to home, try the earliest link (e.g. the cycle head), or abort.
        if (best_distance == INFINITY && best_link == walk_origin_node) {
            move_forward = move_backward = false;

            dump_graph();
            PX4_WARN("I don't see any path home from node %zu", walk_origin_node);
        }

        return best_link;
    }
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
        .did_move_backward = false
    };

    return true;
}


bool Tracker::get_path_to_home(pos_handle_t &pos, float &x, float &y, float &z) {
    // Calculate where to go next
    size_t new_index = calc_return_path(pos.index, pos.bias, pos.did_move_forward, pos.did_move_backward);

    // If we change the link without changing the position, we get a second chance to make progress.
    // The second attempt is guaranteed to yield something else than change-link.
    if (pos.did_move_forward && pos.did_move_backward) {
        pos.index = new_index;
        ipos_t delta = get_intra_node_delta(pos.index, new_index);
        if (delta == ipos_t{ .x = 0, .y = 0, .z = 0 }) {
            TRACKER_DBG("link change didn't result in position change... second chance");
            calc_return_path(pos.index, 0, pos.did_move_forward, pos.did_move_backward);
        } else {
            TRACKER_DBG("link change resulted in position change (%d, %d, %d)", delta.x, delta.y, delta.z);
            pos.position += delta;
        }
    }

    // Apply move-forward-or-backward-instruction
    if (pos.did_move_forward && pos.did_move_backward) {
        // already handled
    } else if (pos.did_move_forward) {
        ipos_t delta;
        walk_forward(pos.index, &delta, NULL, false);
        pos.position += delta;
    } else if (pos.did_move_backward) {
        ipos_t delta; 
        walk_backward(pos.index, &delta, NULL, false);
        pos.position -= delta;
    } else {
        return false;
    }

    pos.bias = 0;
    x = pos.position.x;
    y = pos.position.y;
    z = pos.position.z;
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

        PX4_INFO("  %zu: (%d, %d, %d)%s%s%s",
            index, head.x, head.y, head.z,
            is_cycle_head ? ", head" : is_cycle_master ? ", master" : is_regular_link ? ", link" : "",
            is_data_item(graph[delta_item_index]) ? ", jump" : "",
            graph_current_index == index ? " <= current index" : "");
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
