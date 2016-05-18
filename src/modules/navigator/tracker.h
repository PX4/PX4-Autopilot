
#ifndef NAVIGATOR_TRACKER_H
#define NAVIGATOR_TRACKER_H


#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_local_position.h>


class Tracker
{
    
public:
    // Resets the home position used by the tracker.
    void reset(home_position_s *position);
    
    // Informs the tracker about a new current position.
    void update(vehicle_local_position_s *position);
    
    // Pops the last position from the recent path. Returns false if the path is empty.
    bool pop_recent_path(double &lat, double &lon, float &alt);
    
    // Enables or disables tracking of the recent path.
    void set_recent_path_enabled(bool enabled) { recent_path_enabled = enabled; }
        
    // Dumps the points in the recent path to the log output
    void dump_recent_path(void);
    
private:
	static constexpr float ACCURACY = 2; // tracking accuracy in meters
    
    /*
    * Number of positions that are retained in the recent path buffer.
    * This must be a multiple of 16. 
    * The effective recent path length is RECENT_PATH_LENGTH * ACCURACY
    * Memory usage in bytes is RECENT_PATH_LENGTH * sizeof(pos_t) + c
    */
    static constexpr int RECENT_PATH_LENGTH = 20;
    
    /*
    * Number voxels along each dimension to keep in the flyable region volume.
    * This must be a multiple of 16.
    * The effective tracked volume is (HOME_VOLUME_LENGTH * ACCURACY) ^ 3
    */
    static constexpr int HOME_VOLUME_LENGTH = 32;
    
    typedef struct {
        float x, y, z;
    } pos_t;
    
    static inline bool is_close(pos_t pos1, pos_t pos2);
    
    // Pushes a new current position to the recent path. This works even while the recent path is disabled.
    void push_recent_path(pos_t &position);
    
    // Pops the last position from the recent path. Returns false if the path is empty.
    bool pop_recent_path(pos_t &position);
    
    double ref_lat;
    double ref_lon;
    float ref_alt;
    
    pos_t home_position;
    
    // The recent path respects the following invariant:
    // No two points are closer than ACCURACY.
    pos_t recent_path[RECENT_PATH_LENGTH];
    
    size_t recent_path_next_write = 0; // always valid, 0 if empty, equal to next_read if full
    size_t recent_path_next_read = RECENT_PATH_LENGTH; // LENGTH if empty, valid if non-empty
    
    bool recent_path_enabled = true;
    
    //int16_t flyable_volume[HOME_VOLUME_LENGTH][HOME_VOLUME_LENGTH][HOME_VOLUME_LENGTH >> 4];
};

#endif // NAVIGATOR_TRACKER_H
