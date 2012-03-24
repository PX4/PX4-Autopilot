compal_e88
==========

This directory contains the board support for compal e88 phones.

This port is based on patches contributed by Denis Carikli for both the
compal e99 and e88. At the time of initial check-in, the following phones
were tested:

* motorolla c155 (compal e99) with the compalram and highram configuration
* motorolla W220 (compal e88)
* The openmoko freerunner baseband(compal e88)

The patches were made by Alan Carvalho de Assis and Denis Carikli using
the Stefan Richter's patches that can be found here:

http://cgit.osmocom.org/cgit/nuttx-bb/log/?h=lputt%2Ftesting

http://bb.osmocom.org/trac/wiki/nuttx-bb/run detail the usage of nuttx with
sercomm(the transport used by osmocom-bb that runs on top of serial).

The way of loading nuttx(which is also documented in osmocom-bb wiki) depend
on the configuration(highram/compalram) and phone:

compalram is for the ramloader(for phone having a bootloader on flash)
highram is for phones having the romloader(if the phone has a bootrom) or for
loading in the ram trough a special loader(loaded first on ram by talking to
the ramloader) when having a ramloader(which can only load 64k).
