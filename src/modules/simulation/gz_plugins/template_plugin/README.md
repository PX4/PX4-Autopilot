# Template Gazebo Plugin

This is a template for creating new Gazebo plugins for PX4. Follow these steps to create your own plugin:

1. Copy this directory and rename it to your plugin name
2. Update the project name in CMakeLists.txt
3. Rename and implement the TemplateSystem.hpp/cpp files
4. Add your plugin to the top-level CMakeLists.txt in the gz_plugins directory:
   ```cmake
   add_subdirectory(your_plugin_directory)
   ```
5. Add your plugin's target to the `px4_gz_plugins` target dependencies in the top-level CMakeLists.txt:
   ```cmake
   add_custom_target(px4_gz_plugins ALL DEPENDS OpticalFlowSystem YourPluginSystem)
   ```
6. Update the server.config file to load your plugin:
   ```xml
   <plugin entity_name="*" entity_type="world" filename="libYourPluginSystem.so" name="custom::YourPluginSystem"/>
   ```

## Plugin Structure

This template follows the standard Gazebo plugin structure:

- `TemplateSystem.hpp/cpp`: The main plugin system class that is loaded by Gazebo
- CMakeLists.txt: Build configuration for this plugin

## Testing Your Plugin

After building, you can test your plugin by adding it to the server.config file and running a simulation.
