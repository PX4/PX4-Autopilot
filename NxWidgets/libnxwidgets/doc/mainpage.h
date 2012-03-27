/*! \mainpage NXWidgets Documentation
 *
 * In order to better support NuttX based platforms, a special graphical user
 * interface has been created called NXWidgets. NXWidgets is written in C++
 * and integrates seamlessly with the NuttX NX graphics subystem in order to
 * provide graphic objects, or "widgets", in the <a title="NX Graphics
 * Subsystem" href="http://nuttx.sourceforge.net/NXGraphicsSubsystem.html">
 * NX Graphics Subsystem</a>.
 *
 * \section feature Features
 *
 * \subsection conservative_cxx Conservative C++
 *
 * Written entirely in C++ but using only selected "embedded
 * friendly" C++ constructs that are fully supported under NuttX. No
 * additional C++ support libraries are required.
 *
 * \subsection nx_integration NX Integration
 *
 * Integrates seamlessly with the NX graphics subsytem. Think of the X
 * server under Linux... the NX graphics subsystem is like a tiny X server
 * that provides windowing under NuttX. By adding NXWidgets, you can
 * support graphic objects like buttons and text boxes in the NX windows
 * and toolbars.
 *
 * \subsection small_footprint Small Footprint
 *
 * Tailored for use MCUs in embedded applications. It is ideally suited for
 * mid- and upper-range of most MCU families. A complete NXWidgets is
 * possible in as little as 40Kb of FLASH and maybe 4Kb of SRAM.
 *
 * \subsection output_devices Output Devices
 *
 * NXWidgets will work on the high-end fram buffer devices as well as on
 * LCDs connected via serial or parallel port to a small MCU.
 *
 * \subsection input_devices Input Devices
 *
 * NXWidgets will accept position and selection inputs from a mouse or a
 * touchscreen. It will also support character input from a keyboard such
 * as a USB keyboard. NXWidgets supports a very special widget called
 * CKeypad that will provide keyboard input via on-screen keypad that can
 * be operated via mouse or touchscreen inputs.
 *
 * \subsection many_graphic_objects Many Graphic Objects\
 *
 * Some of the graphic objects supported by NXWidgets include labels,
 * buttons, text boxes, button arrays, check boxes, cycle buttons, images,
 * sliders, scrollable list boxes, progress bars, and more.
 */
