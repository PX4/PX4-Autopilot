
#include <geo/geo.h>
#include "tracker.h"


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
    
    // Scan the buffer for a close position, starting at the latest one
    // In most cases, the very last position will match and the buffer will remain unaltered.
    
    bool rollback = false;
    int index = recent_path_next_write;
    
    do {
        if (!index--) {
            index = RECENT_PATH_LENGTH;
        } else {
            rollback = is_close(recent_path[index], position);
        }
    } while ((index != recent_path_next_read) && !rollback);
    
    
    if (rollback) {
        
        // If there was a close position in the path, roll the path back to that position
        recent_path_next_write = (index + 1) % RECENT_PATH_LENGTH;
        
    } else {
        
        // If there was no close position in the path, add the current position to the buffer
        recent_path[recent_path_next_write] = position;
        
        if (recent_path_next_write++ == recent_path_next_read)
            recent_path_next_read = recent_path_next_write;
            
        if (recent_path_next_write == RECENT_PATH_LENGTH)
            recent_path_next_write = 0;
            
        if (recent_path_next_read == RECENT_PATH_LENGTH)
            recent_path_next_read = 0;
    }
}


bool Tracker::pop_recent_path(pos_t &position) {
    if (recent_path_next_read == RECENT_PATH_LENGTH)
        return false;
        
    int last_index = recent_path_next_write ? (recent_path_next_write - 1) : (RECENT_PATH_LENGTH - 1);
    
    position = recent_path[last_index];
    
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
    PX4_INFO("recent path:");
    int index = recent_path_next_write;
    do {
        if (!index--) {
            index = RECENT_PATH_LENGTH;
        } else {
            PX4_INFO("  x: %f, y: %f, z: %f", recent_path[index].x, recent_path[index].y, recent_path[index].z);
        }
    } while (index != recent_path_next_read);
}
