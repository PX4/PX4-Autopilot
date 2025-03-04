# Gazebo Plugins for PX4

This directory contains the Gazebo plugins used by PX4 for simulation.

## Directory Structure

Each plugin is contained in its own subdirectory with all of its source code and build files.

Current plugins:
- `optical_flow`: Optical flow sensor simulation

## Adding New Plugins

To add a new plugin:

1. Copy the `template_plugin` directory and rename it to your plugin name
2. Update the project name in the CMakeLists.txt file
3. Rename and update the TemplateSystem files to match your plugin functionality
4. Add your plugin to the top-level CMakeLists.txt by adding:
   ```cmake
   add_subdirectory(your_plugin_directory)
   ```
5. Add your plugin's target to the `gz_plugins` target dependencies in the top-level CMakeLists.txt:
   ```cmake
   add_custom_target(gz_plugins ALL DEPENDS OpticalFlowSystem YourPluginSystem)
   ```
6. Update the server.config file to load your plugin:
   ```xml
   <plugin entity_name="*" entity_type="world" filename="libYourPluginSystem.so" name="custom::YourPluginSystem"/>
   ```

## Building

The plugins are built automatically when building PX4 simulation targets.

## Using in Gazebo

To use a plugin in your Gazebo simulation, it needs to be loaded in the server configuration file.
See `gz_bridge/server.config` for an example of how plugins are loaded.