# Rangefinder in PX4
The rangefinder distance (Range) can be used for 2 distinct purposes: EKF Z Position/Velocity observation, and Terrain Height observation.

#### Definitions
- Altitude = EKF Z Position
- Range = Distance to ground (kinematically consistent, gated)
- Terrain Height = Altitude - Range
- Kinematic Consisteny (KC) = Measurements are consistent with aircrafts estimated motion.

## Use cases
- Improve Altitude Hold stabilization while hovering. Mitigate disturbances from wind gusts, GPS multipath, and vibration.
- Fixed wing landing. Terrain Height is important for glide slope.
- Multi-Copter takeoff height accuracy. GPS accuracy is poor while on the ground and barometer suffers from ground effect.
- Multi-Copter landing. Same as above.
- Optical Flow. Velocity estimate relies on Range to convert from Pixel_Vx/Vy to Body_Vx/Vy.
- Anything else?

---

# Discussion

## Problems
Currently the rangefinder usage in PX4 has many bugs and is generally broken. The KC check uses an LPF with a slow time constant and requires vertical motion. The stuck and fog checks can cause false positives due to the simple logic. The terrain estimate doesn't reflect reality. The baro ground effect cause the dist_bottom to grow during takeoff and therefore corrupt the estimate.

## How it should work
The rangefinder distance to ground should be fused into the Altitude Estimate while on the ground and during takeoff. If the measurements are valid and consistent, they represent the most accurate distance to ground. If a takeoff to 2.5m is issued, the local position setpoint, position, and dist_bottom should all be 2.5m.

There is (almost?) no use case for Terrain Following. I think most users would expect the drone to hold absolute position in space if an absolute reference is available (GPS, Vision, Baro relative to Home). The default behavior should be fuse Range measurements as the primary Altitude reference as long as the samples are KC. This ensures that the drone takes off to the correct altitude. Since baro suffers from noise (wind gusts) the fusion of high precision rangefinder measurements while KC should allow minimal deviation of the Local Z Position. After Range fusion is disabled once due to KC check failure, the Local Positon Z can deviate from the Range. This is expected as Terrain can change. During hover while the Range is KC, the measurements are fused and provide a highly accurate observation to ensure hover altitude is maintained precisely.

Indoors flight is also worth considering. Currently users must change EKF2_HGT_REF to Range in order to fly indoors. I think we could be smarter about this and detect when the baro is unreliable and switch to Range as primary altitude reference. We should see KC between Range and IMU_z samples and mark the baro as faulty. This can work during takeoff if Range is the primary reference from takeoff, we would see KC between Range and IMU and see a high baro test ratio.

---

# EKF Implementation

### Raw Sample Pre-filtering
Samples from the rangefinder are popped from a buffer and time-aligned with the IMU sample. The samples are rejected if the quality, tilt, or range are out of bounds. These are hard constraints.
- quality <= 0
- tilt > tilt_max
- sample_rng > rng_max

If samples are rejected or missing for long enough, a timeout occurs which disables fusion of Range into Altitude/Terrain Estimates.

### Kinematic Consistency Check
Samples are now considered potentially valid, but must pass through another filter which evaluates the current samples kinematic consistency. Range Fusion into Altitude/Terrain is disabled as soon as the measurements fail this check.

### Fusion for Terrain Estimate
- Must be KC (only requirement)
	- If not KC, disable fusion

**KC valid**
- If fusion is disabled, reset the terrain estimate to range and zero aid src innovation.
- Fuse measurement into Terrain state

### Fusion for Altitude Estimate
- Must be KC and RangeAidConditionsPassed (RangeAidConditionsPassed = (time gated, in_range, xy_vel < thresh))
	- If not KC or RangeAidConditionsPassed, disable fusion

**KC valid**
- If fusion is disabled do nothing
	- unless on ground, then reset altitude to zero
	- unless innovation rejected, then zero aid src innovation.
- Fuse measurement into Altitude state
