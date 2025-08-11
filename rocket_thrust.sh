#!/bin/bash

echo "🚀 Applying rocket thrust for 1 second..."
echo "Thrust direction: Negative Z-axis (rocket body axis)"

# Apply thrust
gz topic -t "/model/rocket_plane_0/joint/propeller_joint/cmd_thrust" -m gz.msgs.Double -p "data: 21.0"

echo "⚡ Thrust applied! Rocket should be accelerating upward..."

# Wait 1 second
sleep 1

# Stop thrust
gz topic -t "/model/rocket_plane_0/joint/propeller_joint/cmd_thrust" -m gz.msgs.Double -p "data: 0.0"

echo "🛑 Thrust stopped after 1 second. Rocket should now coast."