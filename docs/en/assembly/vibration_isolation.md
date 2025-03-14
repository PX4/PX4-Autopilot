# Vibration Isolation

This topic shows how to determine whether vibration levels are too high, and lists some simple steps to improve vibration characteristics.

## Overview

Flight Control boards with in-built accelerometers or gyros are sensitive to vibrations.
High vibration levels can cause a range of problems, including reduced flight efficiency/performance, shorter flight times and increased vehicle wear-and-tear. In extreme cases vibration may lead to sensor clipping/failures, possibly resulting in estimation failures and fly-aways.

Well-designed airframes damp/reduce the amplitude of specific structural resonances at the autopilot mounting location.
Further isolation may be needed in order to reduce vibration to the level that sensitive components can handle (e.g. some flight controllers must be attached to the airframe using some form of anti-vibration foam/mount - while others are internally isolated).

## Vibration Analysis

[Log Analysis using Flight Review > Vibration](../log/flight_review.md#vibration) explains how to use logs to confirm whether vibration is a probable cause of flight problems.

## Basic Vibration Fixes

A few of simple steps that may reduce vibrations are:

- Make sure everything is firmly attached on the vehicle (landing gear, GPS mast, etc.).
- Use balanced propellers.
- Make sure to use high-quality components for the propellers, motors, ESC and airframe.
  Each of these components can make a big difference.
- Use a vibration-isolation method to mount the autopilot.
  Many flight controllers come with _mounting foam_ that you can use for this purpose, while others have inbuilt vibration-isolation mechanisms.
- As a _last_ measure, adjust the [software filters](../config_mc/filter_tuning.md).
  It is better to reduce the source of vibrations, rather than filtering them out in software.

## References

Some references that you may find useful are:

- [An Introduction to Shock & Vibration Response Spectra, Tom Irvine](http://www.vibrationdata.com/tutorials2/srs_intr.pdf) (free paper)
- [Structural Dynamics and Vibration in Practice - An Engineering Handbook, Douglas Thorby](https://books.google.ch/books?id=PwzDuWDc8AgC&printsec=frontcover) (preview).
