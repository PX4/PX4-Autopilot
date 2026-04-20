---
applyTo: "src/modules/ekf2/**,src/lib/wind_estimator/**,src/lib/world_magnetic_model/**"
---

# Estimation Review Guidelines

In addition to the core code review guidelines:

- Check for singularities in aerospace math (euler angles near gimbal lock, sideslip at low airspeed)
- Flag aliasing from downsampling sensor data without proper filtering
- Verify Kalman filter correctness (Joseph form, innovation variance, covariance symmetry)
- Consider CPU cost on embedded targets (avoid unnecessary sqrt, limit fusion rate)
- Verify frame/coordinate system correctness (FRD vs NED, body vs earth frame)
