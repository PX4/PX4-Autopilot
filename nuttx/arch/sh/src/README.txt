This directory provides a build area for all SH architectures.
The 'common' subdirectory contains source files shared by all SH
architectures; Source files unique to a specific SH chip
architecture are contained in a subdirectory named after the chip.
At configuration time, additional directories will be linked here:
'build' will be a link to the configs/*/src directory; 'chip' will
be a link to the SH chip sub-directory.
