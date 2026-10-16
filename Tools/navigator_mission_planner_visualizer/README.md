# Mission route planner visualizer

Draw missions, safe points and inclusion/exclusion geofences, run PX4's route planner, and export C++ unit tests.

From the repository root:

```bash
pip install streamlit folium streamlit-folium pydeck
make px4_sitl_test EXTERNAL_MODULES_LOCATION=$PWD/Tools/navigator_mission_planner_visualizer/planner_cli mission_route_planner_cli
cd Tools/navigator_mission_planner_visualizer
streamlit run mission_planner_tools.py
```

Paste a test under **C++ source**, select **Parse**, then **Load parsed items into the editor**. Choose **Geofences** in the sidebar to draw inclusion or exclusion polygons and circles. Select **Plan Return** or **Plan Mission rejoin** to check the scene.

See [Add and visualize unit tests](AddAndVisualizeUnitTests.md) for drawing controls, supported C++ fixtures, test export and geofence limitations.
