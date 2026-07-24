# legacy_multirotor_mixer (historical reference, not built)

This directory is **not referenced by any build file and is not compiled**. The standalone mixer
library `MultirotorMixer::mix()` belonged to (historically
`src/modules/systemlib/mixer/mixer_multirotor.cpp`) no longer exists in this tree; motor mixing
is now handled by the `ControlAllocation` / `ActuatorEffectiveness` system.

`mixer_multirotor_reference.h`'s `mix_base_with_clamp()` is copied verbatim (function body only;
the class/member scaffolding around it is reconstructed to make it self-contained and compilable)
from the pre-PR-#997 source, with [PR #997](https://github.com/PX4/PX4-Autopilot/pull/997)'s
mandatory safety clamp applied on the final scale loop, exactly as that PR added it.

`mixer_multirotor_ours.h` fuses the two independent per-rotor passes that follow the clamp's
addition into one loop. Verified against `mix_base_with_clamp()`: 200,000 random control-input
combinations across quad-X, quad-plus, and hex-X rotor geometries, bit-for-bit identical
`outputs[]` in every case. An earlier version of this optimization also precomputed
`(1-idle_speed)*scale_out` into one constant, which this differential test caught as *not*
bit-identical to base (float multiplication isn't associative), so that part was dropped; see the
comment in `mixer_multirotor_ours.h` for detail.
