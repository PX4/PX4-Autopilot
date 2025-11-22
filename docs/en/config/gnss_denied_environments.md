# GNSS-Denied Environments

PX4 can be used without a GNSS or in GNSS-denied environments, such as when signals are blocked or jammed.
This section describes the various configuration options, and when they are needed.

## Overview

<!--

Here is want to cover that flight modes provide features for stabilization of altitude, position, velocity and so on.
Some of them require global position while others, require only altitude, or a local position.
Modes that don't require global position can be used without a GNSS or in a GNSS denied environment, but you may still need to configure some default, which typically assume the presence of a GNSS.

However if you don't have a GNSS or you need to be able to fly in GNSS denied environment you can still use the automatic modes in some circumstances, given another source of position.


-->

## Modes that don't require GNSS

- **Manual mode** (no stabilization assistance)
- **Stabilized mode** (attitude stabilization only)
- **Altitude mode** (if you have a reliable barometer/rangefinder)

---

<!-- Below is working information for data mining.
I want to structure so that people know when they need to make these changes, and direct them towards the stuff they need.
-->

## Primary Solution: Configure for GPS-Denied Operations

**1. Enable arming without GPS:**

[COM_ARM_WO_GPS](../advanced_config/parameter_reference#COM_ARM_WO_GPS)

```
COM_ARM_WO_GPS = 1
```

This allows arming when GPS is unavailable, but PX4 will still check other health conditions.

**2. Configure EKF2 GPS requirements:**

```
EKF2_GPS_CHECK = 0        # Disables GPS quality checks
EKF2_REQ_GPS_H = 0        # No horizontal GPS accuracy required
```

**3. Set appropriate flight mode:**
For indoor flight without GPS, you must use modes that don’t require position estimation:

- **Manual mode** (no stabilization assistance)
- **Stabilized mode** (attitude stabilization only)
- **Altitude mode** (if you have a reliable barometer/rangefinder)

You **cannot** use Position, Hold, Mission, or RTL modes without a position source.

## Better Alternative: Add Indoor Position Estimation

Rather than flying blind, consider adding:

- **Optical flow sensor** + downward rangefinder (PX4Flow, etc.)
- **Motion capture system** (OptiTrack, Vicon)
- **Visual-Inertial Odometry (VIO)** (T265, etc.)

With these, you can enable:

```
EKF2_AID_MASK = 2    # Use optical flow
```

## Circuit Breaker Parameters

If you really need to bypass specific checks for testing:

```
CBRK_GPSFAIL = 240024       # Bypass GPS failure check
```

Circuit breakers disable critical safety features. Only use for controlled testing, and **never** set all of them at once without understanding the implications.

## Important Safety Notes

- Pre-arm checks exist to prevent crashes and flyaways
- Indoor flight without position estimation is high-risk even for experienced pilots
- Always test in Stabilized mode first before attempting anything else
- Consider using a simulator (SITL) to practice GPS-denied flight first
- Make sure you have manual override capability at all times

**Recommended approach:** Start with `COM_ARM_WO_GPS = 1` and `EKF2_GPS_CHECK = 0`, fly in Stabilized mode, and only disable additional checks if you have specific reasons and understand the risks.​​​​​​​​​​​​​​​​

### To add

1. There is also EKF2_GPS_MODE "dead-reckoning mode" in main which allows the GNSS to avoid faulting if there is a velocity source - so more for flying in GPS denied environments, but also allows missions to work.
   Docs are getting final review https://github.com/PX4/PX4-Autopilot/pull/25796.
2. YOu can fly the auto modes, at least for MC, if you have a local position and have mapped it to the global position https://docs.px4.io/main/en/ros/external_position_estimation#enabling-auto-modes-with-a-local-position

I'm sure you know all of that. THis is mostly notes for me.

Are behaviours/setups differing for FW vs MC?
