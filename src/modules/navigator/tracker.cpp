
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


bool Tracker::walk_forward(size_t &index, ipos_t *position, float *distance, bool allow_jump) {
    if (index + 1 >= graph_next_write)
        return false;
    
    bool result = allow_jump;
    ipos_t delta;
    graph_item_t item = graph[++index];

    if (is_data_item(item)) {
        delta.x = as_signed_data_item(item);
        delta.y = as_signed_data_item(graph[++index]);
        delta.z = as_signed_data_item(graph[++index]);
        item = graph[++index]; // fetch link or reserved field (must be a data item)
    }

    if (is_link_item(item))
        item = graph[as_link_item(item) + 2];
    
    if (is_delta_item(item)) {
        delta = as_delta_item(item);
        result = true;
    } else {
        // ignore reserved item for now 
    }

    if (position)
        *position += delta;
    if (distance)
        *distance += fast_sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z, false);

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
    
    return result;
}

bool Tracker::walk_backward(size_t &index, ipos_t *position, float *distance, bool allow_jump) {
    if (index <= 0)
        return false;

    bool result = allow_jump;
    ipos_t delta;
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
    if (is_delta_item(item)) {
        delta = as_delta_item(item);
        result = true;
    } else {
        index -= FAR_JUMP_SIZE - 1;
        delta.x = as_signed_data_item(graph[index + 1]);
        delta.y = as_signed_data_item(graph[index + 2]);
        delta.z = as_signed_data_item(graph[index + 3]);
        // "item" holds a reserved value - ignore it for now
    }

    if (position)
        *position -= delta;
    if (distance)
        *distance += fast_sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z, false);

    return result;
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


void Tracker::goto_graph_index(size_t index, ipos_t position) {
    TRACKER_DBG("vehicle moved to already visited index %zu", index);
    graph_current_index = index;
    graph_current_position = position;
}


void Tracker::push_graph(fpos_t &position) {
    // Don't update if we're still close to the last known position
    //if (is_close(graph_current_position, position))
    // Until we have proper line handling, we record every single position that is unique within the tracker resolution
    if (graph_current_position == to_ipos(position))
        return;


    // Detect if the current position is at a node (in which case we'd have to walk the cycle) 
    size_t walk_origin_node = get_node_or_null(graph_current_index); // will be 0 if there is no node
    size_t walk_origin = walk_origin_node ? walk_origin_node : graph_current_index;

    // Look at all positions that are adjacent to the current position.
    // If neccessary, walk the cycle that represents the node at this position.  
    do {
        size_t index;
        ipos_t head;

        head = graph_current_position;
        if (walk_forward(index = walk_origin, &head, NULL, false)) {
            //if (is_close(head, position)) {
            if (head == to_ipos(position)) { // temporary hack
                goto_graph_index(index, head);
                return;
            }
        }

        head = graph_current_position;
        if (walk_backward(index = walk_origin, &head, NULL, false)) {
            //if (is_close(head, position)) {
            if (head == to_ipos(position)) { // temporary hack
                goto_graph_index(index, head);
                return;
            }
        }
    } while (walk_node(walk_origin_node, walk_origin));


    // Don't proceed if the path is almost full.
    // The most we can append in one go is a far jump, a master link, a compact delta and another master link.
    if (graph_next_write > GRAPH_LENGTH - FAR_JUMP_SIZE - 2 * MASTER_LINK_SIZE - 1)
        return; // todo: consider a clean-up pass

    
    // todo: handle large distances

    ipos_t old_head = to_ipos(graph_head);
    size_t current_index_node = get_node_or_null(graph_current_index);

    // If the new interval is detached, we insert a far jump to the last known position and link it to the last known index.
    // From there, we add a compact delta item as usual.
    if (graph_current_index != graph_next_write - 1 // The current graph position is not at the end of the graph
        && (!current_index_node || current_index_node != get_node_or_null(graph_next_write - 1)) // The current position is not on the node at the end of the graph
        && graph_next_write) // The graph is not empty
    {
        TRACKER_DBG("insert jump after %zu (node %zu), because current index is %zu (node %zu)", graph_next_write - 1, get_node_or_null(graph_next_write - 1), graph_current_index, current_index_node);
        graph[graph_next_write++] = make_data_item(graph_current_position.x - old_head.x);
        graph[graph_next_write++] = make_data_item(graph_current_position.y - old_head.y);
        graph[graph_next_write++] = make_data_item(graph_current_position.z - old_head.z);
        graph[graph_next_write++] = make_data_item(0); // append reserved item
        old_head = graph_current_position;
        push_link(graph_current_index, 0);
    }

    // Add compact delta to current position
    ipos_t new_head = to_ipos(position);
    graph[graph_next_write++] = make_delta_item(old_head, new_head);
    graph_head = position;
    

    // Scan through the graph to see if we've been here already

    int search_bound = GRAPH_SEARCH_RANGE < graph_next_write ? graph_next_write - GRAPH_SEARCH_RANGE : 0;
    ipos_t head = new_head;

    for (size_t index = graph_next_write - 1; index > search_bound;) {
        if (!walk_backward(index, &head, NULL, true))
            break;

        // Check for intersection
        if (head == new_head) {
            push_link(index, 0);
            break;
        }
    }


    // Go to the new position
    graph_current_position = new_head;
    graph_current_index = graph_next_write - 1;
}


void Tracker::push_link(size_t index, uint16_t attribute) {
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
        graph[graph_next_write++] = make_data_item(attribute); // attribute
	    graph[graph_next_write++] = item; // evicted delta item
	    graph[graph_next_write++] = make_data_item(0xFFFF); // node payload
    } else {
        // Append regular link
	    graph[graph_next_write++] = item; // evicted link item
        graph[graph_next_write++] = make_data_item(attribute); // attribute
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
            visited_area_end_distance = as_unsigned_data_item(graph[as_link_item(graph[evicted_node]) + 2]);
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
                memcpy(unvisited_nodes, unvisited_nodes + 1, --unvisited_nodes_count * sizeof(unvisited_nodes[0]));
            } else {
                // If we continue at the end of the visited area, it's only neccessary to walk the node if we continue at a cycle head.
                // (because this can happen if it was evicted from the unvisited nodes buffer)
                // Else, it is sufficient to walk forward and not look into the node
                walk_origin = walk_origin_node = get_node_or_null(visited_area_end);
                if (!walk_origin_node || walk_origin_node != visited_area_end) {
                    walk_origin = visited_area_end;
                    walk_origin_node = 0;
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

        size_t index = walk_origin;
        uint16_t node = 0;
        float interval_length = 0;
        if (walking_forward)
            node = walk_far_forward(index, graph_home_index, interval_length);
        else
            node = walk_far_backward(index, graph_home_index, interval_length);
        TRACKER_DBG("  interval from %zu (node: %zu) to %zu (node: %zu): length %f", walk_origin, walk_origin_node, index, node, interval_length);

        // Calculate distance to home at the beginning and end of the interval we just visited
        graph_item_t *dist_at_interval_start_ptr = get_node_payload_ptr(walk_origin_node);
        graph_item_t *dist_at_interval_end_ptr = get_node_payload_ptr(node);
        uint16_t dist_at_interval_start = dist_at_interval_start_ptr ? as_unsigned_data_item(*dist_at_interval_start_ptr) : visited_area_end_distance;
        uint16_t dist_at_interval_end = dist_at_interval_end_ptr ? as_unsigned_data_item(*dist_at_interval_end_ptr) : (index == graph_home_index ? 0 : (dist_at_interval_start + interval_length));

        // Register expansion of the visited area if applicable
        if (walking_forward && walk_origin == visited_area_end) {
            visited_area_end = index;
            visited_area_end_distance = dist_at_interval_end;
            TRACKER_DBG("  expand controlled area to %zu", visited_area_end);
        }

        // Update node at beginning or end of the current interval, if the interval enables a shorter path
        uint16_t alt_distance = std::min(dist_at_interval_start, dist_at_interval_end) + interval_length;
        if (dist_at_interval_start_ptr && dist_at_interval_start > alt_distance) {
            TRACKER_DBG("  distance at interval start improving to %f", alt_distance);
            *dist_at_interval_start_ptr = make_data_item(alt_distance);
            node = walk_origin_node; // walk_origin_node is non-zero in this case
        } else if (dist_at_interval_end_ptr && dist_at_interval_end > alt_distance) {
            TRACKER_DBG("  distance at interval end improving to %f", alt_distance);
            *dist_at_interval_end_ptr = make_data_item(alt_distance);
            // node is non-zero in this case
        } else {
            TRACKER_DBG("  nothing is improving through this interval");
            node = 0;
        }

        // If we just updated a node (inside the already visited area), make sure it is marked unvisited
        // (if we didn't update the node, it will be 0 by now)
        unvisit_node(node);
    }

#ifdef DEBUG_TRACKER
    if (should_log)
        dump_nodes();
#endif
}


void Tracker::calc_return_path(size_t &index, bool &move_forward, bool &move_backward) {
    if (index == graph_home_index) {
        // todo: consider that home_index may be on a node
        move_forward = move_backward = false;
        return;
    }

    // Detect if the current position is at a node (in which case we'd have to walk the cycle)
    size_t walk_origin_node = get_node_or_null(index); // will be 0 if there is no node
    TRACKER_DBG("calc return path from %zu (node %zu)", index, walk_origin_node);

    // If we're in the middle of an interval and already have a movement direction, don't change it
    if (!walk_origin_node && (move_forward || move_backward))
        return;

    // Make sure all nodes have a valid distance-to-home
    calc_distances();

    float best_distance = INFINITY;
    float best_origin = index;

    size_t walk_origin = walk_origin_node ? walk_origin_node : index;

    // Look at all intervals that originate at the current position to find the shortest path home.
    // If neccessary, walk the cycle that represents the node at this position.
    do {
        uint16_t node;
        float interval_length;
        float dist_forward = INFINITY;
        float dist_backward = INFINITY;
        
        // Calculate the distance if walking forward from walk_origin
        node = walk_far_forward(index = walk_origin, graph_home_index, interval_length = 0);
        if (index == graph_home_index)
            dist_forward = interval_length; // The home position is on this interval
        else if (node)
            dist_forward = as_unsigned_data_item(*get_node_payload_ptr(node)) + interval_length;
        if (dist_forward >= 0x3FFF)
            dist_forward = INFINITY;
        
        // Calculate the distance if walking backward from walk_origin
        node = walk_far_backward(index = walk_origin, graph_home_index, interval_length = 0);
        if (index == graph_home_index)
            dist_backward = interval_length;
        else if (node)
            dist_backward = as_unsigned_data_item(*get_node_payload_ptr(node)) + interval_length;
        if (dist_backward >= 0x3FFF)
            dist_backward = INFINITY;

        TRACKER_DBG("distance from %zu is: %f forward, %f backward", walk_origin, dist_forward, dist_backward);

        // If we found an interval that leads home more directly, keep track of it
        if (dist_forward < best_distance || dist_backward < best_distance) {
            move_backward = !(move_forward = dist_forward < dist_backward);
            best_origin = walk_origin;
            best_distance = move_forward ? dist_forward : dist_backward;
        }
    } while (walk_node(walk_origin_node, walk_origin));

    // Move index within the current node
    index = best_origin;

    if (best_distance == INFINITY) {
        dump_graph();
        PX4_WARN("I'm at graph index %zu and don't see any path home.", index);
    }
}


bool Tracker::get_current_pos(pos_handle_t &pos, float &x, float &y, float &z) {
    pos = {
        .index = graph_current_index,
        .position = graph_current_position,
        .did_move_forward = false,
        .did_move_backward = false
    };

    x = pos.position.x;
    y = pos.position.y;
    z = pos.position.z;

    return true;
}


bool Tracker::get_path_to_home(pos_handle_t &pos, float &x, float &y, float &z) {
    // Calculate where to go next
    calc_return_path(pos.index, pos.did_move_forward, pos.did_move_backward);

    // Apply instruction
    if (pos.did_move_forward)
        walk_forward(pos.index, &pos.position, NULL, false);
    else if (pos.did_move_backward)
        walk_backward(pos.index, &pos.position, NULL, false);
    else
        return false;

    x = pos.position.x;
    y = pos.position.y;
    z = pos.position.z;
    return true;
}


bool Tracker::is_same_pos(pos_handle_t &pos1, pos_handle_t &pos2) {
    if (pos1.index == pos2.index)
        return true;

    size_t node1 = get_node_or_null(pos1.index);
    size_t node2 = get_node_or_null(pos2.index);
    return node1 && node1 == node2;
}


bool Tracker::is_close_to_pos(pos_handle_t &pos) {
    return is_close(to_fpos(graph_current_position), to_fpos(pos.position));
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

    ipos_t head = to_ipos(graph_head);

    size_t index = graph_next_write - 1;
    do {
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
    } while (walk_backward(index, &head, NULL, true));


    PX4_INFO("  nodes:");
    for (index = 0; walk_forward(index, NULL, NULL, true);) {
        if (get_node_or_null(index) != index)
            continue;
        
        PX4_INFO("    node with data %d:", as_unsigned_data_item(*get_node_payload_ptr(index)));
        size_t link = index;
        bool is_cycle_master = false;
        do {
            PX4_INFO("      idx %zu%s", link + (link == index ? 0 : is_cycle_master ? MASTER_LINK_SIZE - 1 : 1), is_cycle_master ? " m" : "");
            is_cycle_master = link == index;
        } while ((link = as_link_item(graph[link])) != index);
    }
}

void Tracker::dump_nodes() {
    PX4_INFO("nodes:");
    for (size_t index = 0; walk_forward(index, NULL, NULL, true);)
        if (get_node_or_null(index) == index)
            PX4_INFO("    node %zu: distance %d", index, as_unsigned_data_item(*get_node_payload_ptr(index)));
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
