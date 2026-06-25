---
applyTo: "src/modules/simulation/**,Tools/simulation/**"
---

# Simulation Review Guidelines

In addition to the core code review guidelines:

- Physics fidelity: noise models should match reality (GPS noise is not Gaussian)
- Keep gz_bridge generic; vehicle-specific logic belongs in plugins
- Prefer gz-transport over ROS2 dependencies when possible
- Use wrench commands for physics correctness vs kinematic constraints
- Library generic/specific boundary: only base classes in common libs
