# PX4 Mission & Geofence Utility

This tool visualizes PX4 navigator unit-test data by pasting C++ code directly. It also includes a creator tab for adding new mission, fence, rally, and path-check data on the map and generating C++ snippets for the new items.

It follows the helper functions defined in `test_RTL_helpers.h`.

## Setup & Running

### Prerequisites
You need Python installed. Then install the required libraries:

Validated with:
- Streamlit 1.40.1
- Python 3.8.10

```bash
pip install streamlit folium streamlit-folium pydeck
```


### Running the App
Navigate to the directory containing the `mission_tools.py` script and run:

```bash
streamlit run mission_tools.py
```

This will open the tool in your default web browser (usually at `http://localhost:8501`).

---

## C++ Syntax Guide

To ensure the parser correctly identifies and plots your data, follow these syntax patterns in your unit tests.

### 1. Mission Items

The parser supports current mission helper patterns and preserves mission order inside `std::vector<mission_item_s>` containers. It plots positional mission items and ignores control items such as `makeVtolTransitionItem(...)` and `makeDoJump(...)` without breaking the route.

```cpp
std::vector<mission_item_s> mission{
    makeTakeoffItem(47.0000, 8.0000, 500.f),
    makePositionItem(47.0005, 8.0000, 520.f),
    makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
    makePositionItem(47.0010, 8.0010, 540.f),
    makeDoJump(0, 2, 1),
    makeLandItem(47.0020, 8.0020, 500.f),
};
```

Supported forms:

```cpp
makePositionItem(lat, lon, alt)
makePositionItem(lat, lon, alt, NAV_CMD_WAYPOINT)
makeTakeoffItem(lat, lon, alt)
makeLandItem(lat, lon, alt)
makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt)
makeTakeoffItemFromOffset(base_lat, base_lon, north_m, east_m, alt)
makeLandItemFromOffset(base_lat, base_lon, north_m, east_m, alt)
```

`makeTakeoffItem` and `makeLandItem` are the canonical generator output for the first and last mission items. Intermediate points use `makePositionItem(...)`.

The parser also resolves simple numeric constants and expressions such as `kBaseLat`, `kBaseLat + 0.001`, `kAlt + 20.f`, and `kAlt - 10.f`.

It also supports appending more mission items after initialization:

```cpp
auto mission = std::vector<mission_item_s> {
    makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
};
mission.push_back(makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt));
mission.push_back(makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt - 10.f));
```

### 2. Polygon Geofences

The parser identifies polygon fences by finding calls to `writePolygonFence(...)`.

```cpp
// 1. Define Vertices Array
const std::array<Mission::LatLonAlt, 4> fence_vertices {{
    {47.0000, 8.0000, 0.f},
    {47.0001, 8.0000, 0.f},
    {47.0001, 8.0001, 0.f},
    {47.0000, 8.0001, 0.f}
}};

// 2. Call Write Function
writePolygonFence(fence_vertices, NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION);
```

* Definition: Vertices in `std::array<Mission::LatLonAlt, N>`.
* Registration: Call `writePolygonFence(variable_name, type_enum)`.
* Types:
  * `NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION` -> Green.
  * `NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION` -> Red.

### 3. Circle Geofences

```cpp
const Mission::LatLonAlt center {47.0000, 8.0000, 0.f};
writeCircleFence(center, 500.f, NAV_CMD_FENCE_CIRCLE_INCLUSION);
```

* The variable name passed to `writeCircleFence` must match a previously defined `Mission::LatLonAlt` variable.
* Use `NAV_CMD_FENCE_CIRCLE_EXCLUSION` for red plotting.

### 4. Rally Points / Safe Points

The parser supports several rally/safe point formats.

#### 4a. `makeSafePointAbsolute()` calls

```cpp
makeSafePointAbsolute(47.0000, 7.9990, 500.f)
```

#### 4b. `makeSafePointFromOffset()` calls

```cpp
makeSafePointFromOffset(base_lat, base_lon, north_m, east_m, alt)
```

* Offset values converted to GPS coordinates.

### 5. Vehicle Locations

The parser supports multiple vehicle location formats.

#### 5a. `MakeLocation()` calls (original format)

```cpp
MakeLocation("Label String", MissionDataset::Default, 46.10, 2.31, 450.3, 30.0f, 0.0f)
```

* Signature: `MakeLocation("Label", Dataset, lat, lon, alt, vx, vy)`.

#### 5b. `VehicleLocation` struct arrays

```cpp
struct VehicleLocation {
    const char *label;
    double lat, lon;
    float alt;
    float vx, vy;
};

static const VehicleLocation kLocations[] = {
    {"Label", 46.105, 2.302, 463.0f, 15.f, 15.f},
    // ...
};
```

* The struct definition is optional (the parser only needs the array).
* Velocity values can reference `constexpr float` variables (e.g., `kVel`).

### 6. Positions (MissionRoutePlanner)

These are visualized as standalone map markers.

#### 6a. `MissionRoutePlanner::Position{lat, lon, alt}`

```cpp
MissionRoutePlanner::Position vehicle_pos{47.0000, 8.0015, 500.f};
```

#### 6b. `makePositionAbsolute(lat, lon, alt)`

```cpp
makePositionAbsolute(47.0000, 8.0000, 500.f)
```

#### 6c. `makePositionFromOffset(base_lat, base_lon, north_m, east_m, alt)`

```cpp
makePositionFromOffset(base_lat, base_lon, 100.0, 200.0, 500.f)
```

### 7. Path Checks

```cpp
Geofence::PathCheck path;
path.lat_from = 47.01;
path.lon_from = 8.00;
path.lat_to   = 47.02;
path.lon_to   = 8.01;
```

* Must assign `.lat_from`, `.lon_from`, `.lat_to`, and `.lon_to` for the same variable.
* Supports array indexing (e.g., `paths[0].lat_from = ...`).
* Supports referencing variables (e.g., `path.lat_to = center.lat`).

### 8. Projections

```cpp
MakeProjection(Dataset, start_idx, end_idx, lat, lon, alt, xtrack, on_seg)
```

* Captures the **4th, 5th, and 6th** arguments as Lat, Lon, and Alt.

### 9. Standalone Mission::LatLonAlt Points

```cpp
const Mission::LatLonAlt vehicle_pos{47.0000, 8.0000, 500.0f};
static constexpr Mission::LatLonAlt proj_pos = {47.0005, 8.0005, 500.0f};
```

---

## Code Generator Output

When using the **Creator** tab to click on the map and generate C++ code, the tool outputs:

- **Mission items**: `makeTakeoffItem(lat, lon, alt)` for the first waypoint, `makeLandItem(lat, lon, alt)` for the last, and `makePositionItem(lat, lon, alt)` for all others, wrapped in a `std::vector<mission_item_s>`.
- **Rally/Safe points**: `makeSafePointAbsolute(lat, lon, alt)` in a `std::vector<mission_item_s>`.
- **Vehicle positions**: `makePositionAbsolute(lat, lon, alt)` for `MissionRoutePlanner::Position`.
- **Fences and Paths**: Same format as before.

---

## Offset-to-GPS Conversion

The tool uses a WGS84 spherical approximation to convert local NED offsets to GPS coordinates, matching the C++ `add_vector_to_global_position()` function:

```
lat = lat_ref + degrees(north_m / 6371000.0)
lon = lon_ref + degrees(east_m / (6371000.0 * cos(radians(lat_ref))))
```

This is accurate to within a few meters for typical test distances (< 10 km).
