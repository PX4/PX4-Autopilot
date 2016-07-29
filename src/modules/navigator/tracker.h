
#ifndef NAVIGATOR_TRACKER_H
#define NAVIGATOR_TRACKER_H

#include <limits.h>
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
    void set_home(home_position_s *position) { set_home(position->x, position->y, position->z); }
    
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
    static constexpr int DIRTY_NODES_BUFFER_SIZE = 5;

    // The number of path elements to store in the return path. This should be sized according to what the client needs.
    static constexpr int RETURN_PATH_SIZE = 6;
    

    struct fpos_t {
        float x, y, z;
    
        inline fpos_t operator*(const float scalar) const {
            return {
                .x = this->x * scalar,
                .y = this->y * scalar,
                .z = this->z * scalar
            };
        }

        inline fpos_t operator-(const fpos_t &pos2) const {
            return {
                .x = this->x - pos2.x,
                .y = this->y - pos2.y,
                .z = this->z - pos2.z
            };
        }
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
    bool is_close_to_line(ipos_t delta, ipos_t end, ipos_t point, int *dist_squared, float *coefficient);
    static inline fpos_t to_fpos(ipos_t pos) { return { .x = (float)pos.x, .y = (float)pos.y, .z = (float)pos.z }; }
    static inline ipos_t to_ipos(fpos_t pos) { return { .x = round(pos.x), .y = round(pos.y), .z = round(pos.z) }; }

    typedef uint16_t graph_item_t;

    // Large elements have 4 playload items:
    //  Node cycle master: delta coefficient, evicted item, distance to home
    //  Far jump: x, y, z, reserved
    static constexpr int MASTER_LINK_SIZE = 4;
    static constexpr int REGULAR_LINK_SIZE = 2;
    static constexpr int FAR_JUMP_SIZE = MASTER_LINK_SIZE;
    static constexpr uint16_t UNSIGNED_DATA_ITEM_MAX = 0x3FFF;

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


    struct link_attributes_t {
        int8_t _dist_to_node_coef;
        int8_t _dist_to_home_coef;

        link_attributes_t(graph_item_t attributes) {
            const int SHIFT_COUNT = CHAR_BIT * sizeof(int) - 7;
            _dist_to_node_coef = ((int)(attributes << (SHIFT_COUNT - 7))) >> SHIFT_COUNT;
            _dist_to_home_coef = ((int)(attributes << (SHIFT_COUNT - 0))) >> SHIFT_COUNT;
        }

        link_attributes_t(float dist_to_node_coef, float dist_to_home_coef) {
            _dist_to_node_coef = (int8_t)(dist_to_node_coef * 0x40);
            _dist_to_home_coef = (int8_t)(dist_to_home_coef * 0x40);
        }

        inline float get_dist_to_node_coef() { return (float)_dist_to_node_coef / 0x40; }
        inline float get_dist_to_home_coef() { return (float)_dist_to_home_coef / 0x40; }

        bool is_valid() {
            return _dist_to_node_coef < 0x40 && _dist_to_node_coef >= (int8_t)0xC0
                && _dist_to_home_coef < 0x40 && _dist_to_home_coef >= (int8_t)0xC0;
        }
        
        graph_item_t as_attr_item() {
            return make_data_item(((_dist_to_node_coef & 0x7F) << 7) | ((_dist_to_home_coef & 0x7F) << 0));
        }

        inline void reset_dist_to_home_coef() { _dist_to_home_coef = _dist_to_node_coef; }

        // improvement: should be in [-2, +2] to prevent overflow. +1 means the node-to-home was reduced by the delta-length.
        void update_node_to_home(float improvement) {
            _dist_to_home_coef += (int8_t)(improvement * 0x40);
            if (!is_valid())
                _dist_to_home_coef = improvement >= 0 ? _dist_to_node_coef : -_dist_to_node_coef;
        }

        bool decrease_dist_to_home_coef(float alt_dist_to_home_coef) {
            int8_t alt_val = (int8_t)(alt_dist_to_home_coef * 0x40);
            if (_dist_to_home_coef > alt_val) { // A smaller distance attribute also means a smaller actual distance-to-home
                _dist_to_home_coef = alt_val;
                return true;
            }
            return false;
        }
    };


    // Informs the tracker about a new home position.
    void set_home(float x, float y, float z);

    // Informs the tracker about a new current vehicle position.
    void update(float x, float y, float z);
    
    // Pushes a new current position to the recent path. This works even while the recent path is disabled.
    void push_recent_path(fpos_t &position);
    
    // Pops the last position from the recent path. Returns false if the path is empty.
    bool pop_recent_path(fpos_t &position);

    // Pushes a new current position to the flight graph.
    void push_graph(fpos_t &position);

    // Adds a compact delta element to the end of the graph.
    void push_compact_delta(ipos_t destination);

    // Adds a far jump element to the end of the graph.
    void push_far_delta(ipos_t destination);

    // Adds a link to the end of the graph that links to the specified index.
    // If there is already a node at that index, the new link is integrated in the node cycle.
    // The end of the graph must be a pure delta (near or far) without a link.
    void push_link(size_t index, link_attributes_t attributes);

    // Returns the index of the head of the cycle that the specified link item belongs to.
    // The caller must ensure that index points to a valid link item.
    size_t get_cycle_head(size_t index);

    // Returns the index of the node that the specified element belongs to.
    // The node index is defined as the index of the first index in the link cycle. 
    // Returns 0 it the element is not part of a node.
    size_t get_node_or_null(size_t index);

    // Returns the length of the delta item at the specified index.
    float get_delta_length(size_t index);

    // Returns a pointer to the payload item of the specified node.
    // If node is invalid (0), this returns a NULL pointer.
    graph_item_t* get_node_payload_ptr(size_t node) { return node ? &graph[as_link_item(graph[node]) + 3] : NULL; }

    // Returns a pointer to the attributes item of the specified link.
    // The caller must make sure that the index actually points to an element with a link that is NOT the cycle head.
    graph_item_t* get_link_attr_ptr(size_t index) { return is_link_item(graph[index - 1]) ? &graph[index] : &graph[index - 2]; }
    
    // Returns the position difference between two links of the same node.
    // The caller must ensure that both indices point to links in the same node.
    ipos_t get_intra_node_delta(size_t link1, size_t link2);

    // Retrieves the distance of a particular link element to the node head.
    // Even though the node and node_delta_length could be inferred from the index, they are expected as a parameter for improved efficiency.
    // If node is invalid (0), the function returns 0.
    float get_link_dist_to_node(size_t node, float node_delta_length, size_t index);

    // Retrieves the distance of a particular link element to home, based on the precalculated value stored in the node.
    // Even though the node and node_delta_length could be inferred from the index, they are expected as a parameter for improved efficiency.
    // If node is invalid (0), the function returns 0.
    float get_link_dist_to_home(size_t node, float node_delta_length, size_t index);

    // Updates the distance of a particular link element to home.
    // This may influence the distance-to-home of the cycle head, or of any other link in the node.
    // No distances can be increased through this function.
    // Even though the node and node_delta_length could be inferred from the index, they are expected as a parameter for improved efficiency.
    // If node is invalid (0), no action is taken.
    // Returns false iif nothing was changed.
    bool set_link_dist_to_home(size_t node, float node_delta_length, size_t index, float distance, bool backward);
    
    // Returns true iif the two indices are equal or belong to the same node.
    bool is_same_pos(size_t index1, size_t index2);

    // Walks forward on the graph by one position. Returns false if the end of the graph is reached.
    // The provided index must be a valid graph-index and will be updated to another valid value.
    // *compact_delta: Receives the information on whether the consumed delta was a compact delta or a far jump.
    bool walk_forward(size_t &index, ipos_t *delta, float *distance, bool *compact_delta);

    // Walks forward on the graph by one position. Returns false if the end of the graph is reached.
    // If jumps are disallowed but a far delta element was consumed, the funtion also returns false.
    inline bool walk_forward(size_t &index, ipos_t *delta, float *distance, bool allow_jump) {
        bool compact_delta;
        return walk_forward(index, delta, distance, &compact_delta) ? allow_jump || compact_delta : false;
    }

    // Walks backward on the graph by one position. Returns false if the beginning of the graph is reached.
    // The provided index must be a valid graph-index and will be updated to another valid value.
    // *compact_delta: Receives the information on whether the consumed delta was a compact delta or a far jump.
    bool walk_backward(size_t &index, ipos_t *delta, float *distance, bool *compact_delta);

    // Walks backward on the graph by one position. Returns false if the beginning of the graph is reached.
    // If jumps are disallowed but a far delta element was consumed, the funtion also returns false.
    bool walk_backward(size_t &index, ipos_t *delta, float *distance, bool allow_jump) {
        bool compact_delta;
        return walk_backward(index, delta, distance, &compact_delta) ? allow_jump || compact_delta : false;
    } 

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

    // Registers that a node just received a new value or wants to receive a new value.
    // This ensures that when the distance-to-home values are demanded, this node and all it's neighbors are updated.
    void invalidate_node(size_t node);

    // Ensures that each node has a valid distance-to-home value.
    void refresh_distances(void);

    // Calculates the distance to home from a specific index in both forward and backward directions.
    void get_distance_to_home(size_t index, float bias, float &dist_forward, float &dist_backward);

    // Returns the index (and bias) which is closest to the specified position.
    size_t get_closest_index(ipos_t position, float *bias);

    // Calculates the next best move to get back home, starting at the specified index.
    // index: The index from which to return home. If this is a node, all adjacent intervals are considered.
    // bias: The actual starting position may be anywhere on the line specified by index. This bias specifies the distance to the end of this line.
    // move_forward/move_backward:
    //  in: Indicates the direction of the previous instruction. If the index doesn't point to a node, we can just go on in this direction.
    //      Both false indicates that the previous direction is unknown.
    //      Both true indicates that the previous instruction was a relocation within the same node.
    //  out: Indicates the direction that leads home.
    //      If a relocation within the same node is recommended, both values are set to false.
    //      If the index already points at the home index, both values are set to false.
    // Returns: In case of an intra-node relocation, the index of the destination link, otherwise the input index.
    size_t calc_return_path(size_t index, float bias, bool &move_forward, bool &move_backward, bool &did_select_link);

    
    
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
    ipos_t graph_head = { .x = 0, .y = 0, .z = 0 };

    // The next free slot in the graph.
    size_t graph_next_write = 0;

    // An index into the graph buffer corresponding to the current vehicle position.
    // A valid graph index obeys these rules:
    // For multi-item path elements, it points to the last item of the element (including the link that belongs to it).
    // Post-decrement (when walking backwards, the element being pointed to is consumed).
    // Pre-increment (when walking forwards, the element after the item being pointed to is consumed).
    size_t graph_current_index = 0;

    // The delta of the line on which the vehicle currently resides.
    ipos_t graph_current_delta = { .x = 0, .y = 0, .z = 0 };

    // The end of the line on which the vehicle currently resides.
    ipos_t graph_current_line_end = { .x = 0, .y = 0, .z = 0 };

    // The delta coefficient to specify the point on the current line that corresponds to the vehicle position
    link_attributes_t graph_current_attributes{0, 0};

    // Returns a point on the graph that corresponds to the current vehicle position
    ipos_t graph_get_current_pos() { return to_ipos(to_fpos(graph_current_line_end) - to_fpos(graph_current_delta) * graph_current_attributes.get_dist_to_node_coef()); }

    // Stores the entire flight path (until the buffer is full).
    // Each delta item stores a position relative to the previous position in the path.
    // If an intersection is detected, a node is created by linking the intersecting positions and attaching some payload data to the node.
    // Note that the first item carries no valid information other than that the graph is non-empty.
    graph_item_t graph[GRAPH_LENGTH];

    // The index in the graph that represents the position that is closest to the true home position.
    size_t graph_home_index = 0;

    // This bias reflects the possibility, that the closest position to home does not exactly coincide with a line end
    float graph_home_bias = 0;

    // A buffer to hold the smallest nodes that have or require a new distance-to-home.
    // This contains only (unique) node indices and is sorted by size.
    size_t dirty_nodes[DIRTY_NODES_BUFFER_SIZE];
    size_t dirty_nodes_count = 0;

    // The end of the controlled area. All nodes beyond this area shall be considered dirty.
    // This is always larger than the greatest dirty node.
    size_t controlled_area_end = 0;
    
    // The distance-to-home at the end of the controlled area
    float controlled_area_end_distance = 0;
    
};

struct Tracker::pos_handle_t {
    size_t index; // Indicates the index of the position represented by this handle.
    ipos_t position; // Holds the position represented by this handle.
    float bias; // Indicates how far the actual position is from the one stored in the position field. 
    bool did_move_backward; // Tells us from which direction this position was reached.
    bool did_move_forward; // Tells us from which direction this position was reached.
    bool did_select_link; // Tells us if we already decided for a specific link in case we're at a node.
};

#endif // NAVIGATOR_TRACKER_H
