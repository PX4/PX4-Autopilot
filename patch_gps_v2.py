
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
        new_lines.append("  // Patched by Antigravity (Robust)\n")
        new_lines.append("  for (unsigned int i = 0; i < model_->NestedModels().size(); ++i) {\n")
        new_lines.append("    physics::ModelPtr nested_model = model_->NestedModels()[i];\n")
        new_lines.append("    if (!nested_model) continue;\n")
        new_lines.append("    gzmsg << \"[MavlinkInterface] Checking nested model: \" << nested_model->GetName() << \"\\n\";\n")
        new_lines.append("    CreateSensorSubscription(&GazeboMavlinkInterface::LidarCallback, this, joints, nested_model, kDefaultLidarModelNaming);\n")
        new_lines.append("    CreateSensorSubscription(&GazeboMavlinkInterface::SonarCallback, this, joints, nested_model, kDefaultSonarModelNaming);\n")
        new_lines.append("    CreateSensorSubscription(&GazeboMavlinkInterface::GpsCallback, this, joints, nested_model, kDefaultGPSModelNaming);\n")
        new_lines.append("    CreateSensorSubscription(&GazeboMavlinkInterface::AirspeedCallback, this, joints, nested_model, kDefaultAirspeedModelJointNaming);\n")
        new_lines.append("  }\n")
        patched = True
        skip = True

    if skip:
        if "CreateSensorSubscription(&GazeboMavlinkInterface::AirspeedCallback" in line:
             skip = False
        continue

    new_lines.append(line)

with open(file_path, 'w') as f:
    f.writelines(new_lines)

print("File patched (robust) successfully.")
