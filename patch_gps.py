
import sys
import os

file_path = '/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_mavlink_interface.cpp'

with open(file_path, 'r') as f:
    lines = f.readlines()

new_lines = []
skip = False
patched = False

for line in lines:
    if "physics::ModelPtr nested_model;" in line and not patched:
        new_lines.append("  // Patched by Antigravity to support multi-GPS\n")
        new_lines.append("  for (auto nested_model : model_->NestedModels()) {\n")
        new_lines.append("    CreateSensorSubscription(&GazeboMavlinkInterface::LidarCallback, this, joints, nested_model, kDefaultLidarModelNaming);\n")
        new_lines.append("    CreateSensorSubscription(&GazeboMavlinkInterface::SonarCallback, this, joints, nested_model, kDefaultSonarModelNaming);\n")
        new_lines.append("    CreateSensorSubscription(&GazeboMavlinkInterface::GpsCallback, this, joints, nested_model, kDefaultGPSModelNaming);\n")
        new_lines.append("    CreateSensorSubscription(&GazeboMavlinkInterface::AirspeedCallback, this, joints, nested_model, kDefaultAirspeedModelJointNaming);\n")
        new_lines.append("  }\n")
        patched = True
        skip = True

    if skip:
        # We need to skip the original implementation lines until the existing calls are passed.
        # The original code structure involves an if block and then the calls.
        # I'll rely on string matching to stop skipping.
        if "CreateSensorSubscription(&GazeboMavlinkInterface::AirspeedCallback" in line:
             skip = False
        continue

    new_lines.append(line)

with open(file_path, 'w') as f:
    f.writelines(new_lines)

print("File patched successfully.")
