/**
 * @file tracker.h
 *
 * Tracks the flight graph in a memory optimized format.
 *
 * The flight graph is stored as a series of delta elements at `GRID_SIZE`
 * resolution and a list of nodes. Both lists share the same buffer `graph`.
 * Each delta element (`delta_item_t`) takes up 2 bytes or sometimes 6 bytes if
 * the delta is large. The path can consist of multiple disconnected segments,
 * in which case the gaps are stored as delta elements with a jump-bit set.
 *
 * Once in a while or when required `consolidate_graph` is called, which means:
 *  - Points that lie roughly in a line are replaced by a single line
 *  - Points that lie close to previously recorded lines are pruned
 *  - For lines that pass close to each other a node element is created
 *
 * Furthermore:
 *  - The graph is recorded at a higher precision closer to home
 *  - If the graph becomes full, the overall precision is reduced and the whole
 *    graph is re-consolidated.
 *  - If the graph becomes full once more, all data is removed except for the
 *    shortest path home at that moment (`rewrite_graph`). One of these actions
 *    is repeated at each subsequent fill-up.
 *
 * Path finding information is generated/refreshed on demand (`refresh_distances`)
 * and stored in the nodes. During return-to-home, the best direction to home
 * is continuously evaluated (`calc_return_path`) by using the information stored
 * in the nodes.
 *
 * @author Samuel Sadok <samuel.sadok@bluewin.ch>
 */

#ifndef NAVIGATOR_TRACKER_H
#define NAVIGATOR_TRACKER_H

#include <limits.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>


// Enables verbose debug messages by the flight path tracker
//#define DEBUG_TRACKER


#ifdef DEBUG_TRACKER
#define TRACKER_DBG(...)    PX4_INFO(__VA_ARGS__)
#define DUMP_GRAPH()        dump_graph()
#else
#define TRACKER_DBG(...)    (void)0
#define DUMP_GRAPH()        (void)0
#endif

#define GRAPH_ERR(...) do { \
		/*PX4_ERR("Unexpected exception in flight path tracker: the flight graph may now be inconsistent must be reset.");*/ \
		PX4_ERR(__VA_ARGS__); \
		graph_fault = true; \
	} while (0)


class Tracker
{
	friend class TrackerTest;

public:

	struct path_finding_context_t;

	// Informs the tracker about a new home position.
	void set_home(home_position_s *position) { set_home(position->x, position->y, position->z); }

	// Informs the tracker about a new current vehicle position.
	void update(vehicle_local_position_s *position);

	// Returns true if a graph fault occurred.
	void get_origin(struct map_projection_reference_s &ref_pos, float &ref_alt)
	{
		ref_pos = _ref_pos;
		ref_alt = _ref_alt;
	}

	// Pops the last position from the recent path. Returns false if the path is empty.
	bool pop_recent_path(double &lat, double &lon, float &alt);

	// Enables or disables tracking of the recent path.
	void set_recent_path_tracking_enabled(bool enabled) { recent_path_tracking_enabled = enabled; }

	// Returns true if a graph fault occurred.
	bool get_graph_fault() { return graph_fault; }

	// Returns the guaranteed tracking precision at the specified position.
	float get_accuracy_at(float x, float y, float z) { return 2 * GRID_SIZE * fast_sqrt(get_granularity_at(to_ipos({ .x = x, .y = y, .z = z })), false); }

	// Forces graph consolidation
	void consolidate_graph() { consolidate_graph("external request"); }

	// Rewrites the graph such that it contains nothing but the shortest path to home.
	void rewrite_graph();

	// Deletes the flight graph.
	void reset_graph();

	// Initializes a path finding context starting at the current position.
	// The context must be viewed as an opaque value that can for instance be passed to the advance_return_path function.
	// Returns true on success.
	bool init_return_path(path_finding_context_t &context, float &x, float &y, float &z);

	// Same as `init_return_path`, but returns the global position
	bool init_return_path_global(path_finding_context_t &context, double &lat, double &lon, float &alt);

	// Fetches the next position on the shortest path back home (starting at, but excluding the position stored by the provided context).
	// The context is updated accordingly.
	// Returns true if the returned position is valid.
	bool advance_return_path(path_finding_context_t &context, float &x, float &y, float &z);

	// Same as `advance_return_path`, but returns the global position
	bool advance_return_path_global(path_finding_context_t &context, double &lat, double &lon, float &alt);

	// Checks if the provided context is currently close to the latest position that the tracker recorded.
	bool is_context_close_to_head(path_finding_context_t &context);

	// Checks if the provided context is currently close to the home position of the tracker.
	bool is_context_close_to_home(path_finding_context_t &context);

	// Checks if both of the provided contexts refer to the same position.
	bool is_same_pos(path_finding_context_t &context1, path_finding_context_t &context2);

	// Dumps the points in the recent path to the log output
	void dump_recent_path(void);

	// Dumps the content of the full flight graph to the log output
	void dump_graph(void);

	// Dumps the payload value of each node in the graph
	void dump_nodes(void);

	// Dumps the shortest path from the current position to the home position
	void dump_path_to_home(void);


private:

	/*** configuration ***/

	// Tracking accuracy in meters
	// todo: test GRID_SIZE values other than 1
	static constexpr float GRID_SIZE = 1;

	// Number of positions that are retained in the recent path buffer.
	// This must be a multiple of 16 (?)
	static constexpr int RECENT_PATH_LENGTH = 32;

	// Number of small lines that can be stored in the full flight graph.
	// The graph size in bytes is usually twice this number.
	// The actual number of position depends on how how well the path can be optimized and how many nodes are stored.
	static constexpr int GRAPH_LENGTH = 256;

	// Scan range used in the consolidation algorithm.
	// If the entire buffer is scanned, CPU usage may be too high.
	// If two close lines are stored further appart than this range, their closeness will not be detected.
	static constexpr int GRAPH_SEARCH_RANGE = GRAPH_LENGTH;

	// The maximum number of positions that should be recorded until the new positions are consolidated.
	// At the beginning and end of each consolidation pass, usually some optimization opportunities are missed.
	// For this reason, a larger number is preferred here.
	// The only drawback of a larger number should be a high workload once consolidation kicks in.
	static constexpr int MAX_CONSOLIDATION_DEBT = 64;

	// Controls the maximum distance from home where the graph is retained at maximum precision.
	// Everything at distance 2^(HIGH_PRECISION_RANGE/2+1) or closer falls into this regime.
	// Note that even within this range, precision will be reduced if neccessary.
	static constexpr int HIGH_PRECISION_RANGE = 10; // corresponds to 64m

	// The size of the cache used by the graph rewrite algorithm.
	// This must be at least FAR_DELTA_SIZE.
	// A smaller number results in higher CPU load when a rewrite is invoked.
	// A larger number leads to a higher stack, so be careful.
	static constexpr int REWRITE_CACHE_SIZE = 9; // 3 * FAR_DELTA_SIZE

	// Maximum number of graph compression performance measurements (this can be 0 unless you're analyzing the graph performance)
	static constexpr int MAX_PERF_MEASUREMENTS = 10;


	// Limitations and properties inherent to the graph representation (should not be changed)
	static constexpr int COMPACT_DELTA_MIN = -16;
	static constexpr int COMPACT_DELTA_MAX = 15;
	static constexpr int FAR_DELTA_MIN = -16383; // 0xC001
	static constexpr int FAR_DELTA_MAX = 16383; // 0x3FFF
	static constexpr int FAR_DELTA_SIZE = 3;
	static constexpr int MAX_INDEX = 0xFFFF;
	static constexpr int MAX_COEFFICIENT = 0x7FFF;
	static constexpr int MAX_DISTANCE = 0x3FFF;
	static constexpr int OBSOLETE_DELTA = 0xC000;

#if GRAPH_LENGTH > MAX_INDEX
#error "GRAPH_LENGTH too large"
#endif


	/*** internal type definitions ***/

	typedef uint16_t delta_item_t;

	// sizeof(node_t) should be 12 bytes (verify this!)
	struct node_t {
		uint16_t index1;            // identifies the first line of the intersection
		uint16_t index2;            // identifies the second line of the intersection
		unsigned int dirty : 1;     // 1 if the node received a new distance value or needs a new value.
unsigned int coef1 :
		15;    // (coef1 / (MAX_COEFFICIENT + 1)) specifies where on line 1 the intersection lies. 0 if it is at the end, 1 if it is at the beginning.
		unsigned int obsolete : 1;  // set to 1 to mark this node for deletion
unsigned int coef2 :
		15;    // (coef2 / (MAX_COEFFICIENT + 1)) specifies where on line 2 the intersection lies. 0 if it is at the end, 1 if it is at the beginning.

		delta_item_t
		delta;         // The intersection points on line1 and line2 are usually not precisely equal. This field specifies the difference between them.

		// Note that an intersection is often not exact but consists of two distinct points.
		// In this case, the distance of either point to home is upper bounded by the node distance.
		// In some cases, both points are a bit closer than this upper bound.

		// Currently, a node only stores routing information for one destination (home).
		// Conceptually, we could support multiple destinations, by replicating the following fields for each destination.
		// When the set of destinations changes, the node size would change, so we need to delete all nodes and regenerate them.
		unsigned int use_line2 : 1; // true if the specified distance can be achieved by walking on line 2
		unsigned int go_forward : 1; // true if the specified distance can be achieved by walking forward (NOT USED CURRENTLY)
unsigned int distance :
		14; // Specifies the distance of this intersection to home. A value of MAX_DISTANCE means infinity.
	};

	struct fpos_t {
		float x, y, z;

		inline fpos_t operator*(const float scalar) const
		{
			return {
				.x = this->x * scalar,
				.y = this->y * scalar,
				.z = this->z * scalar
			};
		}

		inline fpos_t operator-(const fpos_t &pos2) const
		{
			return {
				.x = this->x - pos2.x,
				.y = this->y - pos2.y,
				.z = this->z - pos2.z
			};
		}
	};

	struct ipos_t {
		int x, y, z;

		inline bool operator==(const ipos_t &pos2) const
		{
			return this->x == pos2.x && this->y == pos2.y && this->z == pos2.z;
		}

		inline ipos_t operator+(const ipos_t &pos2) const
		{
			return {
				.x = this->x + pos2.x,
				.y = this->y + pos2.y,
				.z = this->z + pos2.z
			};
		}

		inline ipos_t operator-(const ipos_t &pos2) const
		{
			return {
				.x = this->x - pos2.x,
				.y = this->y - pos2.y,
				.z = this->z - pos2.z
			};
		}

		inline ipos_t operator+=(const ipos_t &pos2)
		{
			this->x += pos2.x;
			this->y += pos2.y;
			this->z += pos2.z;
			return *this;
		}

		inline ipos_t operator-=(const ipos_t &pos2)
		{
			this->x -= pos2.x;
			this->y -= pos2.y;
			this->z -= pos2.z;
			return *this;
		}

		inline ipos_t operator-() const
		{
			return {
				.x = -this->x,
				.y = -this->y,
				.z = -this->z
			};
		}
	};


	/*** utility functions ***/

	static inline int round(float f) { return (int)(f + (f < 0 ? -0.5f : 0.5f)); };
	static inline fpos_t to_fpos(ipos_t pos) { return { .x = (float)pos.x * GRID_SIZE, .y = (float)pos.y * GRID_SIZE, .z = (float)pos.z * GRID_SIZE }; }
	static inline ipos_t to_ipos(fpos_t pos) { return { .x = round(pos.x / GRID_SIZE), .y = round(pos.y / GRID_SIZE), .z = round(pos.z / GRID_SIZE) }; }

	// Calculates the square root of an integer, based on a look-up table.
	// If the range of the look-up table is exceeded, the following action is taken:
	//  fallback_to_infinity: true: INFINITY is returned, false: the usual sqrt function is called
	static float fast_sqrt(int val, bool fallback_to_infinity);

	// Calculates the scalar product of two vectors. If any of the input vectors is very long (> ~16'000), this returns INT_MAX to prevent overflow.
	static int dot(ipos_t vec1, ipos_t vec2);

	// Calculates the squared length of a vector (the scalar product of the vector with itself).
	static inline int dot(ipos_t vec) { return dot(vec, vec); }

	// Determines the shortest from a point to a lines, as well as the point on the line that correspond to this distance.
	static ipos_t get_point_to_line_delta(ipos_t point, ipos_t line_delta, ipos_t line_end, int &coef);

	// Determines the shortest distance between two lines, as well as the points on the lines that correspond to this distance.
	// Either of the two coefficients can be pinned, in which case a point-to-line or point-to-point delta is returned.
	static ipos_t get_line_to_line_delta(ipos_t delta1, ipos_t end1, ipos_t delta2, ipos_t end2, int &coef1, int &coef2,
					     bool pin_coef1, bool pin_coef2);

	// Converts a floating point number (on a scale of 0 to 1) to a discrete coefficient representation (on a scale of 0 to MAX_COEFFICIENT + 1)
	static inline int float_to_coef(float coef) { return coef * (MAX_COEFFICIENT + 1); }

	// Converts a discrete coefficient representation (on a scale of 0 to MAX_COEFFICIENT + 1) to a floating point number (on a scale of 0 to 1)
	static inline float coef_to_float(int coef) { return coef / (float)(MAX_COEFFICIENT + 1); }

	// Applies a coefficient to a delta and discretizes the result
	static ipos_t apply_coef(ipos_t delta, int coef) { return to_ipos(to_fpos(delta) * coef_to_float(coef)); }

	static inline delta_item_t pack_compact_delta_item(ipos_t delta);
	static inline ipos_t unpack_compact_delta_item(delta_item_t delta);

	// Returns true if the vector would be too large for a far delta element
	static inline bool fits_into_far_delta(ipos_t vec);

	// Pushes the specified delta item to a delta list (usually the graph buffer).
	// If possible, the delta is stored as a compact delta item.
	// Else, it is split into multiple items or stored as a far delta element, whichever is smaller.
	// The caller must ensure that there are at least max_space free slots at the specified index.
	// index:
	//  in: the delta will be stored starting at this index
	//  out: set to one index after the delta that was stored
	// max_space: The maximum number of slots that may be used.
	// Returns: true if the delta was pushed, false if the space was insufficient. If max_space >= FAR_DELTA_SIZE, the function always succeeds.
	static bool push_delta(size_t &index, ipos_t delta, bool is_jump, size_t max_space, delta_item_t *buffer);
	inline bool push_delta(size_t &index, ipos_t delta, bool is_jump, size_t max_space = FAR_DELTA_SIZE) { return push_delta(index, delta, is_jump, max_space, graph); };

	// Fetches a delta item from the specified index.
	// If the delta spans multiple indices, it is identified by its last index.
	static ipos_t fetch_delta(size_t index, bool &is_jump, delta_item_t *buffer);
	inline ipos_t fetch_delta(size_t index, bool &is_jump) { return fetch_delta(index, is_jump, graph); };


	/*** private functions ***/

	// Returns a granularity measure of the graph at the specified position.
	// This is a function of the distance to home and memory pressure.
	// The granularity is the square of half of the accuracy guarantee.
	int get_granularity_at(ipos_t pos);

	// Informs the tracker about a new home position.
	void set_home(float x, float y, float z);

	// Informs the tracker about a new current vehicle position.
	void update(float x, float y, float z);

	// Pushes a new current position to the recent path. This works even while the recent path is disabled.
	void push_recent_path(fpos_t &position);

	// Pops the last position from the recent path. Returns false if the path is empty.
	bool pop_recent_path(fpos_t &position);

	// Returns the size of the free area in the graph.
	size_t get_free_graph_space() { return (GRAPH_LENGTH - graph_next_write) * sizeof(delta_item_t) - node_count * sizeof(node_t); };

	// Pushes a new current position to the flight graph.
	void push_graph(fpos_t &position);

	// Reads one delta element from the graph in the forward direction.
	// The index must point to the end of a delta element and is pre-incremented.
	ipos_t walk_forward(size_t &index, bool &is_jump);

	// Reads one delta element from the graph in the backward direction.
	// The index must point to the end of a delta element and is post-decremented.
	ipos_t walk_backward(size_t &index, bool &is_jump);

	// Returns the node at the specified index.
	node_t &node_at(size_t index) { return *(nodes - index); };

	// Pushes a node to the node stack if there is enough space.
	// Returns true if the operation succeeds.
	// Returns false if there is not enough space of if there is already a nearby node.
	bool push_node(node_t &node, int granularity);

	// Removes all nodes (except the home node) that reference lines within the specified range.
	// The lower bound is exclusive, the upper bound is inclusive.
	void remove_nodes(size_t lower_bound, size_t upper_bound);

	// Checks if two positions are similar.
	// This means there's at most one vertex between them, there's no jump between them and their distance along the line is not too large.
	// Returns true if these conditions are satisfied.
	bool check_similarity(size_t index1, int coef1, size_t index2, int coef2, float max_distance);

	// Returns true if the specified position is close to any line in the graph.
	//  lower_bound: exclusive lower search bound
	//  upper_bound: inclusive upper search bound
	bool is_close_to_graph(ipos_t position, size_t lower_bound, size_t upper_bound, ipos_t pos_at_bound);

	// Searches the entire graph for the position which is closest to the specified one.
	//  best_index, best_coef: specifies the index and coefficient of the closest position on the graph. Both are 0 if the graph is empty.
	ipos_t get_closest_position(ipos_t position, size_t &best_index, unsigned int &best_coef);

	// Determines if all points between start and end lie on a straight line.
	// The points are allowed to have the distance max_deviation to the line.
	bool is_line(ipos_t start_pos, size_t start_index, ipos_t end_pos, size_t end_index, bool should_be_jump);

	// Returns the latest index to which a straight line can be drawn from start_index
	//  start_pos: the position corresponding to start_index
	//  start_index: the index at which the line should start
	//  end_pos:
	//      in: ignored
	//      out: the position corresponding to end_index
	//  end_index:
	//      in: ignored
	//      out: the index at which the longest possible line ends (start_index < end_index <= bound)
	//  is_jump: set to the jump property of the line under consideration (jump and non-jump deltas are never aggregated)
	//  bound: the line is constrained not to go beyond this index, but it may end there.
	void get_longest_line(ipos_t start_pos, size_t start_index, ipos_t &end_pos, size_t &end_index, bool &is_jump,
			      size_t bound);

	// Optimizes memory usage of the most recent positions in the graph.
	void consolidate_graph(const char *reason);

	// Makes sure that the meaning of the specified index does not change unless the graph version is incremented.
	inline size_t pin_index(size_t index) { pinned_index = index > pinned_index ? index : pinned_index; return index; }

	// Measures the direct distance between two positions on the graph.
	// Returns infinity if the positions are separated by a jump.
	float measure_distance(size_t index1, int coef1, size_t index2, int coef2);

	// Walks the graph from the specified position up to the next node, while tracking the covered distance.
	// If the search bound or a jump is encountered before a node, the function returns false.
	// Note that jumps can have nodes as well, but only at the end. Therefore, walking backwards may end at a jump node and walking forward may start at one.
	//  index, coef:
	//      in: The index/coefficient at which to start the search.
	//      out: The index/coefficient at which a node was encountered (invalid if the function returns false).
	//  distance: The distance that was covered from the start up to the node (invalid if the function returns false).
	//  forward: true if the function should walk forward, false if it should walk backward.
	//  search_bound: bounds the search (inclusive in both directions)
	bool walk_to_node(size_t &index, int &coef, float &distance, bool forward, size_t search_bound);

	// Uses the best node at the specified position to switch to the line which represents the shortest path home.
	// The distance to home is also returned.
	//  index, coef:
	//      in: the position at which the nodes should be examined
	//      out: the position that leads home
	//  delta: if not NULL, set to the position difference between the input and output positions
	//  go_forward: set to the best direction to home from the new line (valid iif the position was at a node, i.e. if the result is finite)
	float apply_node_delta(size_t &index, unsigned int &coef, ipos_t *delta, bool &go_forward);

	// Returns the distance to home of any position on the graph which coincides with a node,
	// including an indication on how this distance can be achieved.
	//  index, coef: the position for which the distance-to-home should be retrieved
	//  direction: 0: switch line, 1: go forward from here, -1: go backward from here
	float get_node_distance(size_t index, unsigned int coef, int &direction);

	// Sets the distance to home of any position on the graph which corresponds to one or multiple nodes.
	// The function takes only improving action (i.e. only reduces distances) and returns true if did so.
	//  go_forward: indicates if the distance can be achieved by walking forward
	bool set_node_distance(size_t index, int coef, float distance, bool go_forward);

	// Ensures that each node has a valid distance-to-home value.
	void refresh_distances(void);

	// Calculates the distance to home from a specific index in both forward and backward directions.
	//void get_distance_to_home(size_t index, float bias, float &dist_forward, float &dist_backward);

	// Returns the index (and bias) which is closest to the specified position.
	//size_t get_closest_index(ipos_t position, float *bias);

	// Calculates the next best move to get back home, using the provided path finding context.
	bool calc_return_path(path_finding_context_t &context, bool &progress);

	// Marks all delta elements in the forward or backward direction obsolete, until a node is encountered.
	//  index, coef: where to start marking deltas as obsolete. The starting index is always kept.
	//  forward, backward: which direction to go to mark deltas as obsolete
	//  retain: if this item is reached, the walk is aborted
	//  lower_bound, upper_bound: the exclusive bounds of the walk
	void mark_obsolete(size_t index, int coef, bool forward, bool backward, size_t retain, size_t lower_bound,
			   size_t upper_bound);


	/*** internal variables ***/

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	uint64_t _ref_timestamp;

	ipos_t home_position;
	ipos_t home_on_graph;
	bool did_set_home = false;


	bool recent_path_tracking_enabled = true;
	bool graph_tracking_enabled = true;

	fpos_t last_known_position = { .x = 0, .y = 0, .z = 0 };

	// The most recent position in the recent path. The absolute positions in the path can be calculated based on this.
	// If tracking has not yet started (that is, if the path is empty), this is invalid.
	ipos_t recent_path_head = { .x = 0, .y = 0, .z = 0 };

	// Stores the (potentially shortened) recent flight path as a ring buffer.
	// The recent path respects the following invariant: No two points are closer than the grid size.
	// This buffer contains only delta items. Each item stores a position relative to the previous position in the path.
	// The very first item is a bumper that indicates that the initial position (0,0,0) is not valid. This bumper will disappear once the ring buffer overflows.
	delta_item_t recent_path[RECENT_PATH_LENGTH] = { 0xFFFF };

	size_t recent_path_next_write = 1; // always valid, 0 if empty, equal to next_read if full
	size_t recent_path_next_read = 0; // RECENT_PATH_LENGTH if empty, valid if non-empty


	// Indicates that the graph may have become inconsistent.
	// If this occurs, it must be reset at the next opportunity.
	bool graph_fault = false;

	// Keeps track of the memory pressure to allow for adaptive graph precision.
	// Each time the graph becomes full, the memory pressure is incremented by one.
	// Each even time, the graph precision is reduced, each odd time, the graph is rewritten.
	int memory_pressure = 1;

	// The first valid position in the graph, corresponding to the first position in the buffer.
	ipos_t graph_start = { .x = 0, .y = 0, .z = 0 };

	// The last position in the graph, corresponding to the end of the buffer.
	// This roughly corresponds to the current vehicle position.
	// The absolute positions in the path can be calculated based on this.
	ipos_t graph_head_pos = { .x = 0, .y = 0, .z = 0 };

	// The next free index in the graph.
	size_t graph_next_write = 0;

	// Stores the entire flight path (until the buffer is full).
	// Each delta item represents a line from the previous position to a new position.
	// Sometimes (if the vehicle moves along an already visited path), jump deltas are inserted.
	// Note that the first item carries no valid information other than that the graph is non-empty.
	delta_item_t graph[GRAPH_LENGTH];

	// The consolidated head position represents the last position which shall no longer be considered by the consolidation pass.
	// Moving this around arbitrarily should not result in undefined behaviour.
	ipos_t consolidated_head_pos = { .x = 0, .y = 0, .z = 0 };
	size_t consolidated_head_index = 0;

	// If the graph is altered in a way that previously exported indices become invalid, the graph version is incremented.
	int graph_version = 0;

	// If this index or anything before changes, the graph version must be incremented.
	size_t pinned_index = 0;


	// Nodes keep track of lines in the flight path that pass close to each other.
	// The nodes are stored at the end of the graph buffer, the node with the index 0 denotes the last node.
	// When deltas and nodes collide, the graph is full.
	// Node 0 (index1, coef1) represents the home position and must not be removed.
	node_t *nodes = (node_t *)(graph + GRAPH_LENGTH) - 1;
	size_t node_count = 1;

	// True as long as any nodes in the buffer have dirty set to 1
	bool have_dirty_nodes = true;



	struct compress_perf_t {
		hrt_abstime runtime;
		size_t deltas_before;
		size_t nodes_before;
		size_t deltas_after;
		size_t nodes_after;
	};

	compress_perf_t perf_measurements[MAX_PERF_MEASUREMENTS];
};

struct Tracker::path_finding_context_t {
	ipos_t current_pos; // The position corresponding to (current_index, current_coef)
	size_t current_index;
	unsigned int current_coef;
	size_t checkpoint_index; // If (current_index, current_coef) becomes equal to this checkpoint, a re-evaluation of the best direction is forced, which may involve switching lines on an intersection.
	unsigned int checkpoint_coef;

	int graph_version;
};

#endif // NAVIGATOR_TRACKER_H