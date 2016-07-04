
#ifndef NAVIGATOR_TRACKER_H
#define NAVIGATOR_TRACKER_H


#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_local_position.h>

// Enables verbose debug messages by the flight path tracker
#define DEBUG_TRACKER


#ifdef DEBUG_TRACKER
#define TRACKER_DBG(...) PX4_INFO(__VA_ARGS__)
#else
#define TRACKER_DBG(...)
#endif


class Tracker
{
    friend class TrackerTest;
    
public:

    struct pos_handle_t;

    // Informs the tracker about a new home position.
    void set_home(home_position_s *position);
    
    // Informs the tracker about a new current vehicle position.
    void update(vehicle_local_position_s *position) { if (position->xy_valid && position->z_valid) update(position->x, position->y, position->z); }
    
    // Pops the last position from the recent path. Returns false if the path is empty.
    bool pop_recent_path(double &lat, double &lon, float &alt);
    
    // Enables or disables tracking of the recent path.
    void set_recent_path_tracking_enabled(bool enabled) { recent_path_tracking_enabled = enabled; }

    // Returns a handle that refers to the current position in the graph, and the corresponding coordinates.
    // The position handle must be viewed as an opaque value that can for instance be passed to the get_path_to_home function.
    // Returns true on success.
    bool get_current_pos(pos_handle_t &pos, float &x, float &y, float &z);

    // Fetches the next position on the shortest path back home (starting at, but excluding the provided position).
    // The position handle is updated accordingly.
    // Returns true if the returned position is valid.
    bool get_path_to_home(pos_handle_t &pos, float &x, float &y, float &z);

    // Returns true if the two position handles refer to the same position.
    bool is_same_pos(pos_handle_t &pos1, pos_handle_t &pos2);

    // Returns true if the vehicle is close to the position represented by the specified handle.
    bool is_close_to_pos(pos_handle_t &pos);

    // Returns true if the home position is close to the position represented by the specified handle.
    // This ignores the altitude information.
    bool is_close_to_home(pos_handle_t &pos);
    
    // Dumps the points in the recent path to the log output
    void dump_recent_path(void);
    
    // Dumps the content of the full flight graph to the log output
    void dump_graph(void);
    
    // Dumps the payload value of each node in the graph
    void dump_nodes(void);
    
    // Dumps the shortest path from the current position to the home position
    void dump_path_to_home(void);

    
private:
    
    // Tracking accuracy in meters
	static constexpr float ACCURACY = 2;
    
    // Number of positions that are retained in the recent path buffer.
    // This must be a multiple of 16 (?)
    static constexpr int RECENT_PATH_LENGTH = 64;
    
    // Number of positions that can be stored in the full flight graph.
    // The actual number of position depends on how many intersections are recorded.
    static constexpr int GRAPH_LENGTH = 256;

    // Number of position to scan at each update to find intersections.
    // If the entire buffer is scanned, CPU usage may be too high.
    static constexpr int GRAPH_SEARCH_RANGE = GRAPH_LENGTH;

    // A larger value increases RAM usage, while a smaller value increases CPU usage
    static constexpr int UNVISITED_NODES_BUFFER_SIZE = 5;

    // The number of path elements to store in the return path. This should be sized according to what the client needs.
    static constexpr int RETURN_PATH_SIZE = 6;
    

    struct fpos_t {
        float x, y, z;
    };
    
    struct ipos_t {
        int x, y, z;
        
        inline bool operator==(const ipos_t &pos2) const {
            return this->x == pos2.x && this->y == pos2.y && this->z == pos2.z;
        }

        inline ipos_t operator+(const ipos_t &pos2) const {
            return {
                .x = this->x + pos2.x,
                .y = this->y + pos2.y,
                .z = this->z + pos2.z
            };
        }

        inline ipos_t operator-(const ipos_t &pos2) const {
            return {
                .x = this->x - pos2.x,
                .y = this->y - pos2.y,
                .z = this->z - pos2.z
            };
        }

        inline ipos_t operator+=(const ipos_t &pos2) {
            this->x += pos2.x;
            this->y += pos2.y;
            this->z += pos2.z;
            return *this;
        }

        inline ipos_t operator-=(const ipos_t &pos2) {
            this->x -= pos2.x;
            this->y -= pos2.y;
            this->z -= pos2.z;
            return *this;
        }
    };
    
    static inline int round(float f) { return (int)(f + (f < 0 ? -0.5f : 0.5f)); };
    static inline bool is_close(fpos_t pos1, fpos_t pos2);
    static inline bool is_close(ipos_t pos1, fpos_t pos2);
    static inline fpos_t to_fpos(ipos_t &pos) { return { .x = (float)pos.x, .y = (float)pos.y, .z = (float)pos.z }; }
    static inline ipos_t to_ipos(fpos_t &pos) { return { .x = round(pos.x), .y = round(pos.y), .z = round(pos.z) }; }


    typedef uint16_t graph_item_t;

    // Large elements have 4 playload items:
    //  Node cycle master: delta coefficient, evicted item, distance to home
    //  Far jump: x, y, z, reserved
    static constexpr int MASTER_LINK_SIZE = 4;
    static constexpr int REGULAR_LINK_SIZE = 2;
    static constexpr int FAR_JUMP_SIZE = MASTER_LINK_SIZE;

    static inline graph_item_t make_delta_item(ipos_t &from_pos, ipos_t &to_pos);
    static inline graph_item_t make_link_item(uint16_t index) { return (1 << 15) | (index & 0x3FFF); }
    static inline graph_item_t make_data_item(uint16_t data) { return (3 << 14) | (data & 0x3FFF); }
    static inline bool is_delta_item(graph_item_t &item) { return !(item >> 15); }
    static inline bool is_link_item(graph_item_t &item) { return (item >> 14) == 2; }
    static inline bool is_data_item(graph_item_t &item) { return (item >> 14) == 3; }

    // Returns the delta position stored in a delta item. The caller must ensure that the item is really a delta item.
    static inline ipos_t as_delta_item(graph_item_t &delta);

    // Returns the index contained by a link item. The caller must ensure that the item is really a link item.
    static inline uint16_t as_link_item(graph_item_t &item) { return item & 0x3FFF; }

    // Returns the (unsigned) payload contained by a data item. The caller must ensure that the item is really a data item.
    static inline uint16_t as_unsigned_data_item(graph_item_t &item) { return item & 0x3FFF; }

    // Returns the (signed) payload contained by a data item. The caller must ensure that the item is really a data item.
    static inline int16_t as_signed_data_item(graph_item_t &item);


    // Informs the tracker about a new current vehicle position.
    void update(float x, float y, float z);

    // Registers that the vehicle moved to another position that already exists in the graph.
    void goto_graph_index(size_t index, ipos_t position);
    
    // Pushes a new current position to the recent path. This works even while the recent path is disabled.
    void push_recent_path(fpos_t &position);
    
    // Pops the last position from the recent path. Returns false if the path is empty.
    bool pop_recent_path(fpos_t &position);

    // Pushes a new current position to the flight graph.
    void push_graph(fpos_t &position);

    // Adds a link to the end of the graph that links to the specified index.
    // If there is already a node at that index, the new link is integrated in the node cycle.
    // The end of the graph must be a pure delta (near or far) without a link.
    void push_link(size_t index, uint16_t attribute);

    // Returns the index of the head of the cycle that the specified link item belongs to.
    // The caller must ensure that index points to a valid link item.
    size_t get_cycle_head(size_t index);

    // Returns the index of the node that the specified element belongs to.
    // The node index is defined as the index of the first index in the link cycle. 
    // Returns 0 it the element is not part of a node.
    size_t get_node_or_null(size_t index);

    // Returns the payload of the specified node.
    // If node is invalid (0), this returns a NULL pointer.
    graph_item_t* get_node_payload_ptr(size_t node) { return node ? graph + as_link_item(graph[node]) + 3 : NULL; }

    // Walks forward on the graph by one position.
    // The provided index must be a valid graph-index and will be updated to another valid value.
    // allow_jump: If false, if a jump element (far delta) is encountered, the function returns false (but the parameters are still updated)
    bool walk_forward(size_t &index, ipos_t *position, float *distance, bool allow_jump);

    // Walks backward on the graph by one position.
    // The provided index must be a valid graph-index and will be updated to another valid value.
    // allow_jump: If false, if a jump element (far delta) is encountered, the function returns false (but the parameters are still updated)
    bool walk_backward(size_t &index, ipos_t *position, float *distance, bool allow_jump);

    // Walks forward on the graph until either an element with a link, the checkpoint or a jump is consumed (or the end of the graph is reached).
    // index: in: The index before the first element to be consumed.
    //        out: An index pointing to a link element, the checkpoint or a far-delta element.
    // checkpoint: An index that should not be crossed (but can be started from). Set to 0 to disable.
    // interval_length: increased by the length of the interval that was consumed.
    // Returns: The cycle head of the link that was found or zero.
    size_t walk_far_forward(size_t &index, size_t checkpoint, float &interval_length);

    // Walks backward on the graph until either an element with a link, the checkpoint or a jump is reached (or the beginning of the graph is reached).
    // index: in: The index of the first element to be consumed.
    //        out: An index pointing to a link element, the checkpoint or before a far-delta element.
    // checkpoint: An index that should not be crossed (but can be started from). Set to 0 to disable.
    // interval_length: increased by the length of the interval that was consumed.
    // Returns: The cycle head of the link that was found or zero.
    size_t walk_far_backward(size_t &index, size_t checkpoint, float &interval_length);

    // Walks through a node cycle by advancing index by one link.
    // Returns false if node is invalid (0) or when the cycle head is reached.
    // Hence, when iterating through the links of a node, start at the node head and call walk_node AFTER each iteration.
    inline bool walk_node(size_t node, size_t &index);

    // Registers that a node just received a new value (or was newly created).
    // This ensures that when the distance-to-home values are demanded, the new/updated node is respected.
    void unvisit_node(size_t node);

    // Ensures, that each node has a valid distance-to-home value.
    void calc_distances(void);

    // Calculates the next instruction to get back home, starting at the specified index.
    // index: in: the index from which to return home. If this is a node, all adjacent intervals are considered.
    //        out: the index from which to actually move. This is either equal to the input or an index of the same node.
    // move_forward/move_backward:
    //  in: Indicates the direction of the previous instruction. Both false if the previous direction is unknown.
    //      If the index doesn't point to a node, we can just go on in this direction.
    //  out: Indicates the direction that leads home. If the index already points at the home index, both directions are false.
    void calc_return_path(size_t &index, bool &move_forward, bool &move_backward);

    
    
    fpos_t home_position;


    bool recent_path_tracking_enabled = true;
    bool graph_tracking_enabled = true;

    
    // The most recent position in the recent path. The absolute positions in the path can be calculated based on this. 
    // If tracking has not yet started (that is, if the path is empty), this is invalid.
    fpos_t recent_path_head = { .x = 0, .y = 0, .z = 0 };
    
    // Stores the (potentially shortened) recent flight path as a ring buffer.
    // The recent path respects the following invariant: No two points are closer than ACCURACY.
    // This buffer contains only delta items. Each item stores a position relative to the previous position in the path.
    // The very first item is a bumper that indicates that the initial position (0,0,0) is not valid. This bumper will disappear once the ring buffer overflows.
    graph_item_t recent_path[RECENT_PATH_LENGTH] = { make_data_item(0) };
    
    size_t recent_path_next_write = 1; // always valid, 0 if empty, equal to next_read if full
    size_t recent_path_next_read = 0; // LENGTH if empty, valid if non-empty


    // The last position in the graph (corresponding to the end of the buffer). The absolute positions in the path can be calculated based on this.
    fpos_t graph_head = { .x = 0, .y = 0, .z = 0 };

    // The next free slot in the graph.
    size_t graph_next_write = 0;

    // The current vehicle position.
    ipos_t graph_current_position = { .x = 0, .y = 0, .z = 0 };

    // An index into the graph buffer corresponding to the current vehicle position.
    // A valid graph index obeys these rules:
    // For multi-item path elements, it points to the last item of the element (including the link that belongs to it).
    // Post-decrement (when walking backwards, the element being pointed to is consumed).
    // Pre-increment (when walking forwards, the element after the item being pointed to is consumed).
    size_t graph_current_index = 0;

    // Stores the entire flight path (until the buffer is full).
    // Each delta item stores a position relative to the previous position in the path.
    // If an intersection is detected, a node is created by linking the intersecting positions and attaching some payload data to the node.
    // Note that the first item carries no valid information other than that the graph is non-empty.
    graph_item_t graph[GRAPH_LENGTH];

    // The index in the graph that represents the position that is closest to the true home position.
    uint16_t graph_home_index = 0;

    // A buffer to hold the smallest nodes that have a new distance-to-home.
    // This contains only (unique) cycle heads and is sorted by size.
    uint16_t unvisited_nodes[UNVISITED_NODES_BUFFER_SIZE];
    uint16_t unvisited_nodes_count = 0;

    // The end of the linearily visited area.
    // Up to this index, all nodes have a consistent and up-to-date distance-to-home, except the ones mentioned in unvisited_nodes.
    // This is always larger than the greatest unvisited node.
    uint16_t visited_area_end = 0;
    
    // The distance-to-home at the end of the visited area
    uint16_t visited_area_end_distance = 0;
    
};

struct Tracker::pos_handle_t {
    size_t index; // Indicates the index of the position represented by this handle.
    ipos_t position; // Holds the position represented by this handle.
    bool did_move_backward; // Tells us from which direction this position was reached.
    bool did_move_forward; // Tells us from which direction this position was reached.
};

#endif // NAVIGATOR_TRACKER_H
