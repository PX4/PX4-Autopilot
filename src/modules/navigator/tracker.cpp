
#include <limits.h>
#include <geo/geo.h>
#include "tracker.h"


inline bool Tracker::is_close(fpos_t pos1, fpos_t pos2) {
    float delta_x = pos1.x - pos2.x;
    float delta_y = pos1.y - pos2.y;
    float delta_z = pos1.z - pos2.z;
    return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z < (ACCURACY * ACCURACY);
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

inline void Tracker::subtract_delta_item(ipos_t &position, graph_item_t &delta) {
    const int SHIFT_COUNT = CHAR_BIT * sizeof(int) - 5; 
    position.x -= ((int)(delta << (SHIFT_COUNT - 10))) >> SHIFT_COUNT;
    position.y -= ((int)(delta << (SHIFT_COUNT - 5))) >> SHIFT_COUNT;
    position.z -= ((int)(delta << (SHIFT_COUNT - 0))) >> SHIFT_COUNT;
}

void Tracker::reset(home_position_s *position) {
    home_position.x = position->x;
    home_position.y = position->y;
    home_position.z = position->z;
}


void Tracker::update(vehicle_local_position_s *position) {
    if (position->xy_global) {
        ref_lat = position->ref_lat;
        ref_lon = position->ref_lon;
    }
    
    if (position->z_global)
        ref_alt = position->ref_alt;
    
    if (!position->xy_valid || !position->z_valid)
        return;
        
    fpos_t local_position = {
        .x = position->x,
        .y = position->y,
        .z = position->z
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
        
            if (index == recent_path_next_write)
                break; // Note that the initial position is always invalid, so we musn't use that
                    
            subtract_delta_item(head, recent_path[index]);
            
            rollback = is_close(head, position);
        } while ((index != recent_path_next_read) && !rollback);
    }
    
    
    if (rollback) {
        
        // If there was a close position in the path, roll the path back to that position
        recent_path_head = to_fpos(head);
        recent_path_next_write = index;
        PX4_INFO("recent path rollback to %zu", index);
        
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
    
    if (recent_path_next_read == RECENT_PATH_LENGTH)
        return false;
    
    int last_index = (recent_path_next_write ? recent_path_next_write : RECENT_PATH_LENGTH) - 1;
    
    // Roll back most recent position
    ipos_t head = to_ipos(recent_path_head);
    subtract_delta_item(head, recent_path[last_index]);
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


Tracker::graph_item_t Tracker::get_delta_item(size_t &index) {
    graph_item_t item = full_path[index--];

    if (is_delta_item(item))
        return item;

    if (is_data_item(item)) {
        index -= 2;
    } else {
        size_t link = get_link_item(item);
        if (is_data_item(full_path[link + 2]))
            return full_path[link + 1];
    }

    return full_path[index--];
}


void Tracker::push_graph(fpos_t &position) {
    ipos_t old_head = to_ipos(full_path_head);

    // Don't update if we're still close to the last position
    if (is_close(old_head, position))
        return;

    // Don't update if the path is almost full (we might append up to 4 elements)
    if (full_path_next_write > FULL_PATH_LENGTH - 4)
        return; // todo: consider a clean-up pass
    
    
    // Add the current position to the graph
    // todo: handle large distances
        
    ipos_t new_head = to_ipos(position);
    
    full_path_head = position;
    full_path[full_path_next_write++] = make_delta_item(old_head, new_head);
    

    // Scan through the existing path to see if we've been here already

    ipos_t head = new_head;

    int search_bound = FULL_PATH_SEARCH_RANGE < full_path_next_write ? full_path_next_write - FULL_PATH_SEARCH_RANGE : 0;

    for (size_t index = full_path_next_write - 1; index > search_bound;) {
        graph_item_t item = get_delta_item(index);
        subtract_delta_item(head, item);

        // Check for intersection
        if (head == new_head) {
            if (is_data_item(full_path[index]))
                index -= 2;

            // Evict existing item by inserting a link to the current position (which is where the back-link will be placed)
            item = full_path[index];
            full_path[index] = make_link_item(full_path_next_write);

            // If the evicted item was not already a link, we need to initialize the link cycle
            if (is_delta_item(item))
                full_path[full_path_next_write++] = make_link_item(index);
            
            // Insert the evicted item (which may be a link)
            full_path[full_path_next_write++] = item;
            
            // If we just created a new cycle, we need to attach a reserved field
            if (is_delta_item(item))
                full_path[full_path_next_write++] = make_data_item(0);

            break;
        }
    }
}


void Tracker::dump_recent_path() {
    if (recent_path_next_read == RECENT_PATH_LENGTH) {
        PX4_INFO("recent path empty");
        PX4_INFO("element size is %lu bytes", sizeof(graph_item_t));
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
        subtract_delta_item(head, recent_path[index]);
    } while (index != recent_path_next_read);
}


void Tracker::dump_full_path() {
    if (!full_path_next_write) {
        PX4_INFO("full path empty");
        return;
    }
    
    PX4_INFO("full path (%zu elements):", full_path_next_write);

    ipos_t head = to_ipos(full_path_head);

    for (size_t index = full_path_next_write - 1; index > 0;) {
        if (is_data_item(full_path[index]))
            index -= 2;

        PX4_INFO("  %zu: (%d, %d, %d)", index, head.x, head.y, head.z);

        graph_item_t item = get_delta_item(index);
        subtract_delta_item(head, item);
    }

    PX4_INFO("full path nodes:");
    for (size_t index = 2; index < full_path_next_write; index++) {
        graph_item_t item = full_path[index];
        if (is_data_item(item)) {
            PX4_INFO("  node with data %d:", get_data_item(item));
            size_t link = index - 2;
            do {
                PX4_INFO("  pos %zu", link);
            } while ((link = get_link_item(full_path[link])) != index - 2);
        }
    }
}
