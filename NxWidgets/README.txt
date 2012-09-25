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

Directory Structure
===================

Kconfig

  This is a Kconfig file that should be provided at apps/NxWidgets/Kconfig.
  When copied to that location, it will be used by the NuttX configuration
  systems to configure settings for NxWidgets and NxWM

libnxwidgets

  The source code, header files, and build environment for NxWidgets is
  provided in this directory. 

UnitTests

  Provides a collection of unit-level tests for many of the individual
  widgets provided by libnxwidgets.

nxwm

  This directory holds a tiny desktop for small embedded devices with a
  touchscreen,. NxWM.  NxWM is true multiple window manager but only one
  window is displayed at a time.  This simplification helps performance on
  LCD based products (in the same way that a tiled window manager helps)
  and also makes the best use of small displays.  It is awkward from a
  human factors point-of-view trying to manage multiple windows on a
  small display.

  The window manager consists of a task bar with icons representing the
  running tasks.  If you touch the task's icon, it comes to the top.  Each
  window has a toolbar with (1) a title, (2) a minimize button, and (3) a
  stop application button using the standard icons for these things.

  There is always a start window that is available in the task bar.  When
  you touch the start window icon, it brings up the start window containing
  icons representing all of the available applications.  If you touch an
  icon in the start window, it will be started and added to the task bar.

  There is a base class that defines an add-on application and an
  interface that supports incorporation of new application.  The only
  application that is provided is NxConsole.  This is an  NSH session
  running in a window.  You should be able to select the NX icon in the start
  menu and create as many NSH sessions in windows as you want. (keybard input
  still comes through serial).
