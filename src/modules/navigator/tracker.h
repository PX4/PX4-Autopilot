
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
    void set_recent_path_tracking_enabled(bool enabled) { recent_path_tracking_enabled = enabled; }
    
    // Enables or disables tracking of the flight path as a graph.
    void set_graph_tracking_enabled(bool enabled) { graph_tracking_enabled = enabled; }
    
    // Dumps the points in the recent path to the log output
    void dump_recent_path(void);
    
    // Dumps the content of the full flight graph to the log output
    void dump_full_path(void);
    
private:
    
    // Tracking accuracy in meters
	static constexpr float ACCURACY = 2;
    
    // Number of positions that are retained in the recent path buffer.
    // This must be a multiple of 16 (?)
    static constexpr int RECENT_PATH_LENGTH = 64;
    
    // Number of positions that can be stored in the full flight path.
    // The actual number of position depends on how many intersections are recorded.
    static constexpr int FULL_PATH_LENGTH = 256;

    // Number of position to scan at each update to find intersections.
    // If the entire buffer is scanned, CPU usage may be too high.
    static constexpr int FULL_PATH_SEARCH_RANGE = FULL_PATH_LENGTH;
    

    struct fpos_t {
        float x, y, z;
    };
    
    struct ipos_t {
        int x, y, z;
        
        inline bool operator==(const ipos_t &pos2) const {
            return this->x == pos2.x && this->y == pos2.y && this->z == pos2.z;
        }
    };
    
    static inline bool is_close(fpos_t pos1, fpos_t pos2);
    static inline bool is_close(ipos_t pos1, fpos_t pos2);
    static inline fpos_t to_fpos(ipos_t &pos) { return { .x = (float)pos.x, .y = (float)pos.y, .z = (float)pos.z }; }
    static inline ipos_t to_ipos(fpos_t &pos) { return { .x = (int)pos.x, .y = (int)pos.y, .z = (int)pos.z }; }


    typedef uint16_t graph_item_t;

    static inline graph_item_t make_delta_item(ipos_t &from_pos, ipos_t &to_pos);
    static inline graph_item_t make_link_item(uint16_t index) { return (1 << 15) | (index & 0x3FFF); }
    static inline graph_item_t make_data_item(uint16_t data) { return (3 << 14) | (data & 0x3FFF); }
    static inline bool is_delta_item(graph_item_t &item) { return !(item >> 15); }
    static inline bool is_data_item(graph_item_t &item) { return (item >> 14) == 3; }

    // Subtracts the delta from the provided position. The caller must ensure that the item is really a delta item.
    static inline void subtract_delta_item(ipos_t &position, graph_item_t &delta);

    // Returns the index contained by a link item. The caller must ensure that the item is really a link item.
    static inline uint16_t get_link_item(graph_item_t &item) { return item & 0x3FFF; }

    // Returns the payload contained by a data item. The caller must ensure that the item is really a data item.
    static inline uint16_t get_data_item(graph_item_t &item) { return item & 0x3FFF; }

    
    // Pushes a new current position to the recent path. This works even while the recent path is disabled.
    void push_recent_path(fpos_t &position);
    
    // Pops the last position from the recent path. Returns false if the path is empty.
    bool pop_recent_path(fpos_t &position);

    // Pushes a new current position to the flight graph.
    void push_graph(fpos_t &position);

    // Fetches the delta item at the specified index from the graph.
    // If the index points to a special item, this is handled appropriately.
    // The index is decremented to point to the next item that should be consumed.
    // The index must be larger than 0.
    graph_item_t get_delta_item(size_t &index);

    
    double ref_lat;
    double ref_lon;
    float ref_alt;
    
    fpos_t home_position;


    bool recent_path_tracking_enabled = true;
    bool graph_tracking_enabled = true;

    
    // The most recent position in the recent path. The absolute positions in the path can be calculated based on this. 
    // If tracking has not yet started (that is, if the path is empty), this is invalid.
    fpos_t recent_path_head = { .x = 0, .y = 0, .z = 0 };
    
    // Stores the (potentially shortened) recent flight path as a ring buffer.
    // The recent path respects the following invariant: No two points are closer than ACCURACY.
    // This buffer contains only delta items. Each item stores a position relative to the previous position in the path.
    // Note that the first item carries no valid information other than that the path is non-empty.
    graph_item_t recent_path[RECENT_PATH_LENGTH];
    
    size_t recent_path_next_write = 0; // always valid, 0 if empty, equal to next_read if full
    size_t recent_path_next_read = RECENT_PATH_LENGTH; // LENGTH if empty, valid if non-empty


    // The most recent position in the full path.
    fpos_t full_path_head = { .x = 0, .y = 0, .z = 0 };

    // Stores the entire flight path (until the buffer is full).
    // Each delta item stores a position relative to the previous position in the path.
    // If an intersection is detected, a node is created by linking the intersecting positions and attaching some payload data to the node.
    // Note that the first item carries no valid information other than that the path is non-empty.
    graph_item_t full_path[FULL_PATH_LENGTH];
    size_t full_path_next_write = 0;
};

#endif // NAVIGATOR_TRACKER_H
