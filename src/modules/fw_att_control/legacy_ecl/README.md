# legacy_ecl (historical reference, not built)

This directory is **not referenced by any CMakeLists.txt and is not compiled**. It reconstructs
the pre-quaternion fixed-wing attitude controllers (`ECL_RollController`, `ECL_PitchController`,
`ECL_YawController`) as they existed around PR #975, with an additional optimization applied on
top: a truly-fused `sincosf` (`fast_sincosf.h`) that shares one argument reduction between sin and
cos, since the target flight-controller libm does not.

That architecture was since replaced by the quaternion-based controller in
`FixedwingAttitudeControl.cpp`, which does not have an equivalent trig hotspot, so this
optimization has no live target in the current codebase. These files are included as reference for
the technique and its measured results (see the PR description) rather than as a change intended
to be built or merged as-is.
