NXWidgets
=========

In order to better support NuttX based platforms, a special graphical user
interface has been created called NXWidgets. NXWidgets is written in C++
and integrates seamlessly with the NuttX NX graphics subsystem in order
to provide graphic objects, or "widgets," in the NX Graphics Subsystem

Some of the features of NXWidgets include:

o Conservative C++

  NXWidgets is written entirely in C++ but using only selected “embedded
  friendly” C++ constructs that are fully supported under NuttX. No
  additional C++ support libraries are required.

o NX Integration

  NXWidgets integrate seamlessly with the NX graphics system. Think of the
  X server under Linux … the NX graphics system is like a tiny X server
  that provides windowing under NuttX. By adding NXWidgets, you can support
  graphics objects like buttons and text boxes in the NX windows and toolbars.

o Small Footprint

  NXWidgets is tailored for use MCUs in embedded applications. It is ideally
  suited for mid- and upper-range of most MCU families. A complete NXWidgets
  is possible in as little as 40Kb of FLASH and maybe 4Kb of SRAM.

o Output Devices

  NXWidgets will work on the high-end frame buffer devices as well as on LCDs
  connected via serial or parallel ports to a small MCU.

o Input Devices

  NXWidgets will accept position and selection inputs from a mouse or a
  touchscreen. It will also support character input from a keyboard such as a
  USB keyboard. NXWidgets supports on very special widget called CKeypad that
  will provide keyboard input via an on-screen keypad that can be operated
  via mouse or touchscreen inputs.

o Many Graphic Objects

  Some of the graphic objects supported by NXWidgets include labels, buttons,
  text boxes, button arrays, check boxes, cycle buttons, images, sliders,
  scrollable list boxes, progress bars, and more.

Note:  Many of the fundamental classed in NxWidgets derive from the Antony
Dzeryn's "Woopsi" project: http://woopsi.org/ which also has a BSD style
license.  See the COPYING file for details.
