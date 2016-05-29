
#include <geo/geo.h>
#include "tracker.h"

// todo: check if relative position size is 2 bytes on constrained systems, if not, we have to discard the use of bitfields
//#define MAKE_REL_POS(delta_x, delta_y, delta_z) ((delta_x & 0x1F) << 10) & ((delta_y & 0x1F) << 5) & ((delta_z & 0x1F) << 0)
//#define REL_POS_DELTA_X(rel_pos) (rel_pos >> 10 & 0x1F)
//#define REL_POS_DELTA_Y(rel_pos) (rel_pos >> 5 & 0x1F)
//#define REL_POS_DELTA_Z(rel_pos) (rel_pos >> 0 & 0x1F)


inline bool Tracker::is_close(pos_t pos1, pos_t pos2) {
    float delta_x = pos1.x - pos2.x;
    float delta_y = pos1.y - pos2.y;
    float delta_z = pos1.z - pos2.z;
    return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z < (ACCURACY * ACCURACY);
}


void Tracker::reset(home_position_s *position) {
    home_position.x = position->x;
    home_position.y = position->y;
    home_position.z = position->z;
    // todo: shift flyable volume
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
        
    pos_t local_position = {
        .x = position->x,
        .y = position->y,
        .z = position->z
    };
    
    if (recent_path_enabled)
        push_recent_path(local_position);
        
        
    // todo: set voxel in flyable volume
    local_position.x -= home_position.x;
    local_position.y -= home_position.y;
    local_position.z -= home_position.z;
}


void Tracker::push_recent_path(pos_t &position) {
    
    bool rollback = false;
    int index = recent_path_next_write;
    pos_t head = recent_path_head;
     
    if (recent_path_next_read != RECENT_PATH_LENGTH) {
        
        // If we're still close to the most recent position, don't update
        if (is_close(recent_path_head, position))
            return;
        
        // Scan the buffer for a close position, starting at the latest one
        do {
            index = (index ? index : RECENT_PATH_LENGTH) - 1;
        
            if (index == recent_path_next_write)
                break; // Note that the initial position is always invalid, so we musn't use that
                    
            rel_pos_t rel_pos = recent_path[index];
            head.x -= rel_pos.delta_x;
            head.y -= rel_pos.delta_y;
            head.z -= rel_pos.delta_z;
            
            rollback = is_close(head, position);
        } while ((index != recent_path_next_read) && !rollback);
    }
    
    
    if (rollback) {
        
        // If there was a close position in the path, roll the path back to that position
        recent_path_head = head;
        recent_path_next_write = index;
        
    } else {
        
        // If there was no close position in the path, add the current position to the buffer
        // todo: handle large distances
        rel_pos_t rel_pos = {
            .valid = 1,
            .delta_x = (int)(position.x - recent_path_head.x),
            .delta_y = (int)(position.y - recent_path_head.y),
            .delta_z = (int)(position.z - recent_path_head.z)
        };
        
        recent_path_head = position;
        recent_path[recent_path_next_write] = rel_pos;
        
        if (recent_path_next_write++ == recent_path_next_read)
            recent_path_next_read = recent_path_next_write;
            
        if (recent_path_next_write == RECENT_PATH_LENGTH)
            recent_path_next_write = 0;
            
        if (recent_path_next_read == RECENT_PATH_LENGTH)
            recent_path_next_read = 0;
    }
}


bool Tracker::pop_recent_path(pos_t &position) {
    position = recent_path_head;
    
    if (recent_path_next_read == RECENT_PATH_LENGTH)
        return false;
    
    int last_index = (recent_path_next_write ? recent_path_next_write : RECENT_PATH_LENGTH) - 1;
    
    // Roll back most recent position
    rel_pos_t rel_pos = recent_path[last_index];
    recent_path_head.x -= rel_pos.delta_x;
    recent_path_head.y -= rel_pos.delta_y;
    recent_path_head.z -= rel_pos.delta_z;
    
    if ((recent_path_next_write = last_index) == recent_path_next_read) {
        recent_path_next_write = 0;
        recent_path_next_read = RECENT_PATH_LENGTH;
    }
    return true;
}


bool Tracker::pop_recent_path(double &lat, double &lon, float &alt) {
    pos_t position;
    
    if (!pop_recent_path(position))
        return false;
    
    return !globallocalconverter_toglobal(position.x, position.y, position.z, &lat, &lon, &alt);
}

void Tracker::dump_recent_path() {
    if (recent_path_next_read == RECENT_PATH_LENGTH) {
        PX4_INFO("recent path empty");
        PX4_INFO("element size is %lu bytes", sizeof(rel_pos_t));
        return;
    }
    
    PX4_INFO("recent path:");
    
    int index = recent_path_next_write;
    pos_t head = recent_path_head;
    
    do {
        PX4_INFO("  x: %f, y: %f, z: %f", (double)head.x, (double)head.y, (double)head.z);
        
        index = (index ? index : RECENT_PATH_LENGTH) - 1;
        
        rel_pos_t rel_pos = recent_path[index];
        head.x -= rel_pos.delta_x;
        head.y -= rel_pos.delta_y;
        head.z -= rel_pos.delta_z;
    } while (index != recent_path_next_read);
}
