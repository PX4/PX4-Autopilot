# PX4 Mission & Geofence Utility

This tool allows developers to visualize PX4 unit test mission data (Missions, Geofences, Rally Points, Bank Turn Scenarios, etc.) by pasting C++ code directly. It also includes a creator tab to generate C++ waypoint arrays by clicking on a map.

It supports the RtlRoutePlanner helper functions defined in `RtlRoutePlannerTestHelpers.h`.

## Setup & Running

### Prerequisites
You need Python installed. Then install the required libraries:

Version:
- Streamlit, version 1.40.1
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

To ensure the parser correctly identifies and plots your data, follow these syntax patterns in your C++ unit test files.

### 1. Mission Items

The parser supports multiple mission item formats. It filters items and **only plots** `NAV_CMD_TAKEOFF`, `NAV_CMD_WAYPOINT`, and `NAV_CMD_LAND`.

#### 1a. `makePositionItem()` calls

```cpp
makePositionItem(47.0000, 8.0000, 500.f, NAV_CMD_WAYPOINT)
makePositionItem(47.0000, 8.0040, 500.f, NAV_CMD_TAKEOFF)
```

* Signature: `makePositionItem(lat, lon, alt, NAV_CMD_*)`.

#### 1b. Absolute takeoff/land helpers

```cpp
makeTakeoffItem(47.0000, 8.0000, 500.f)
makeLandItem(47.0000, 8.0020, 500.f)
```

* Same as `makePositionItem` with `NAV_CMD_TAKEOFF` / `NAV_CMD_LAND` and `autocontinue=false`.

#### 1c. Offset-based helpers

```cpp
makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_WAYPOINT)
makeTakeoffItemFromOffset(base_lat, base_lon, north_m, east_m, alt)
makeLandItemFromOffset(base_lat, base_lon, north_m, east_m, alt)
```

* Offset values are converted to GPS coordinates using WGS84 spherical approximation.
* All numeric arguments must be literal values (not variable references) for offset helpers.

### 2. Polygon Geofences

The parser identifies polygon fences by finding calls to the `writePolygonFence` helper function.

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

### 5. Bank Turn Scenarios

```cpp
static constexpr BankTurnGeofenceScenario kBankTurnScenarios[] = {
    {
        "NoWind",
        {47.3977420, 8.5455940, 500.0f},
        {47.3959744, 8.5460557, 500.0f},
        100.0f,
        0.0f,
        0.0f,
        20.0f,
        true,
    },
};
```

* Must be declared as `BankTurnGeofenceScenario <var>[] = { ... };`.
* Each entry must follow the exact field order shown above.

### 6. Vehicle Locations

The parser supports multiple vehicle location formats.

#### 6a. `MakeLocation()` calls (original format)

```cpp
MakeLocation("Label String", MissionDataset::Default, 46.10, 2.31, 450.3, 30.0f, 0.0f)
```

* Signature: `MakeLocation("Label", Dataset, lat, lon, alt, vx, vy)`.

#### 6b. `VehicleLocation` struct arrays (new format)

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

### 7. Positions (RtlRoutePlanner)

These are visualized as standalone map markers.

#### 7a. `RtlRoutePlanner::Position{lat, lon, alt}`

```cpp
RtlRoutePlanner::Position vehicle_pos{47.0000, 8.0015, 500.f};
```

#### 7b. `makePositionAbsolute(lat, lon, alt)`

```cpp
makePositionAbsolute(47.0000, 8.0000, 500.f)
```

#### 7c. `makePositionFromOffset(base_lat, base_lon, north_m, east_m, alt)`

```cpp
makePositionFromOffset(base_lat, base_lon, 100.0, 200.0, 500.f)
```

### 8. Path Checks

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

### 9. Projections

```cpp
MakeProjection(Dataset, start_idx, end_idx, lat, lon, alt, xtrack, on_seg)
```

* Captures the **4th, 5th, and 6th** arguments as Lat, Lon, and Alt.

### 10. Standalone Mission::LatLonAlt Points

```cpp
const Mission::LatLonAlt vehicle_pos{47.0000, 8.0000, 500.0f};
static constexpr Mission::LatLonAlt proj_pos = {47.0005, 8.0005, 500.0f};
```

---

## Code Generator Output

When using the **Creator** tab to click on the map and generate C++ code, the tool outputs:

- **Mission items**: `makeTakeoffItem(lat, lon, alt)` for the first waypoint, `makeLandItem(lat, lon, alt)` for the last, and `makePositionItem(lat, lon, alt)` for all others, wrapped in a `std::vector<mission_item_s>`.
- **Rally/Safe points**: `makeSafePointAbsolute(lat, lon, alt)` in a `std::vector<mission_item_s>`.
- **Vehicle positions**: `makePositionAbsolute(lat, lon, alt)` for `RtlRoutePlanner::Position`.
- **Fences, Bank Turns, Paths**: Same format as before (unchanged).

---

## Offset-to-GPS Conversion

The tool uses a WGS84 spherical approximation to convert local NED offsets to GPS coordinates, matching the C++ `add_vector_to_global_position()` function:

```
lat = lat_ref + degrees(north_m / 6371000.0)
lon = lon_ref + degrees(east_m / (6371000.0 * cos(radians(lat_ref))))
```

This is accurate to within a few meters for typical test distances (< 10 km).
