# USV Dynamics Plugin

This plugin implements comprehensive hydrodynamics simulation for Unmanned Surface Vehicles (USVs) in Gazebo. It provides realistic water-surface vehicle dynamics including hydrodynamic forces, buoyancy, drag, and added mass effects for boat/ship simulation.

## Features

- **Added Mass Effects**: Models virtual mass added by displaced water during acceleration
- **Hydrodynamic Drag**: Linear and quadratic drag forces in all 6 degrees of freedom
- **Coriolis Effects**: Cross-coupling between linear and angular motions for realistic turning behavior
- **Buoyancy Simulation**: Grid-based buoyancy calculation using discrete hull points
- **Twin-Hull Support**: Designed for catamaran-style vessels with configurable dimensions

## Physics Model

The plugin implements a 6-DOF hydrodynamic model with:

### Added Mass Matrix (6x6)
- Surge (X), Sway (Y), and Yaw (N) added mass coefficients
- Models the virtual mass effect of water displaced during acceleration

### Drag Forces
- **Linear Drag**: Proportional to velocity in each DOF
- **Quadratic Drag**: Proportional to velocity squared for surge, sway, and yaw
- **Velocity-dependent**: Drag magnitude varies with speed

### Buoyancy Forces
- Grid-based calculation across hull length and width
- Circular cross-section approximation for hull segments
- Dynamic buoyancy based on water immersion depth

## Configuration Parameters

### Hull Geometry
- `boatLength`: Overall vessel length [m] (default: 1.35)
- `boatWidth`: Beam width between hulls [m] (default: 1.0)
- `hullRadius`: Demi-hull radius [m] (default: 0.213)
- `length_n`: Number of hull segments for buoyancy calculation (default: 10)

### Environmental Parameters
- `waterLevel`: Water surface height [m] (default: 0.5)
- `waterDensity`: Water density [kg/m³] (default: 997.7735)

### Hydrodynamic Coefficients

#### Added Mass
- `xDotU`: Added mass coefficient in surge (default: 5)
- `yDotV`: Added mass coefficient in sway (default: 5)
- `nDotR`: Added mass coefficient in yaw (default: 1)

#### Linear Drag
- `xU`: Linear drag coefficient in surge (default: 20)
- `yV`: Linear drag coefficient in sway (default: 20)
- `zW`: Linear drag coefficient in heave (default: 20)
- `kP`: Linear drag coefficient in roll (default: 20)
- `mQ`: Linear drag coefficient in pitch (default: 20)
- `nR`: Linear drag coefficient in yaw (default: 20)

#### Quadratic Drag
- `xUU`: Quadratic drag coefficient in surge (default: 0)
- `yVV`: Quadratic drag coefficient in sway (default: 0)
- `nRR`: Quadratic drag coefficient in yaw (default: 0)

## Usage

### Basic Usage

Add the plugin to your boat model SDF file:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="boat">

    <!-- Your model links and joints -->

    <link name="base_link">
      <!-- Define the main hull link -->
    </link>

    <plugin
      filename="libUsivDynamicsPlugin.so"
      name="custom::UsvDynamics">
      <bodyName>base_link</bodyName>

      <!-- Hull geometry -->
      <boatLength>1.35</boatLength>
      <boatWidth>1.0</boatWidth>
      <hullRadius>0.213</hullRadius>
      <length_n>10</length_n>

      <!-- Environmental parameters -->
      <waterLevel>0.5</waterLevel>
      <waterDensity>997.7735</waterDensity>

      <!-- Added mass coefficients -->
      <xDotU>5</xDotU>
      <yDotV>5</yDotV>
      <nDotR>1</nDotR>

      <!-- Linear drag coefficients -->
      <xU>20</xU>
      <yV>20</yV>
      <zW>20</zW>
      <kP>20</kP>
      <mQ>20</mQ>
      <nR>20</nR>

      <!-- Quadratic drag coefficients -->
      <xUU>0</xUU>
      <yVV>0</yVV>
      <nRR>0</nRR>
    </plugin>

  </model>
</sdf>
```

### Running with PX4

To use the plugin with a boat model in PX4:

```bash
make px4_sitl gz_boat
```

## Dependencies

- Eigen3: Matrix operations
- Gazebo Garden/Harmonic: Simulation framework
- PX4 simulation environment

## Based On

This plugin is a direct port of the USV dynamics plugin from Gazebo Classic, originally developed by Brian Bingham, adapted for modern Gazebo (Garden/Harmonic) and PX4 integration.
