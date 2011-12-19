README
^^^^^^

This directory contains tiny graphics support for NuttX.  The contents of this directory
are only build if CONFIG_NX is defined in the NuttX configuration file.

Contents
^^^^^^^^
  Roadmap
  Related Header Files
  Directories
  Installing New Fonts
  Configuration Settings

Roadmap
^^^^^^^

This directory holds NuttX graphic packages.  Not all of these packages are implemented
at the present, but here is the longer term roadmap:

  NXWIDGETS - I had originally planned a high level, C++, object-oriented library for
              object-oriented access to graphics widgets.  However, because C++ compilers
              are not available for some of the targets supported by NuttX, I have
              decided to implement the entire solution in  C -- that makes the solution
              much uglier, but works fine on all platforms.
  NXTOOLKIT - A set of C graphics tools that provide higher-level window drawing
              operations.  The toolkit can be used for window-oriented graphics
              without NXWIDGETS and is built on top of NX.
  NXFONTS   - A set of C graphics tools for present (bitmap) font images.
  NX        - The tiny NuttX windowing system.  This includes both a small-footprint,
              single user implementaton (NXSU as described below) and a somewhat
              larger multi-user implentation (NXMU as described below).  Both
              conform to the same APIs as defined in include/nuttx/nx/nx.h and, hence,
              are more-or-less interchangable.  NX can be used without NXWIDGETS
              and without NXTOOLKIT for raw access to window memory.
  NXGLIB    - Low level graphics utilities and direct framebuffer rendering logic.
              NX is built on top of NXGLIB.

Related Header Files
^^^^^^^^^^^^^^^^^^^^

include/nuttx/nx/nxglib.h   -- Describes the NXGLIB C interfaces
include/nuttx/nx/nx.h       -- Describes the NX C interfaces
include/nuttx/nx/nxtk.h     -- Describe the NXTOOLKIT C interfaces
include/nuttx/nx/nxfont.h   -- Describe sthe NXFONT C interfaces
include/nuttx/nx/nxwidgets.h -- Will describe the NXWIDGETS classes (no longer planned)

Directories
^^^^^^^^^^^

graphics/nxglib
  The NuttX tiny graphics library.  The directory contains generic utilities
  support operations on primitive graphics objects and logic to rasterize directly
  into a framebuffer.  It has no concept of windows (other than the one, framebuffer
  window).

graphics/nxbe
  This is the "back-end" of a tiny windowing system.  It can be used with either of
  two front-ends to complete a windowing system (see nxmu and nxsu below).  It
  contains most of the important window management logic:  clipping, window controls,
  window drawing, etc.

graphics/nxsu
  This is the NX single user "front end".  When combined with the generic "back-end"
  (nxbe), it implements a single thread, single user windowing system.  The files
  in this directory present the window APIs described in include/nuttx/nx/nx.h.  The
  single user front-end is selected when CONFIG_NX_MULTIUSER is not defined in the
  NuttX configuration file.

graphics/nxmu
  This is the NX multi user "front end".  When combined with the generic "back-end"
  (nxbe), it implements a multi-threaded, multi-user windowing system.  The files
  in this directory present the window APIs described in include/nuttx/nx/nx.h.  The
  multi-user front end includes a graphics server that executes on its own thread;
  multiple graphics clients then communicate with the server via a POSIX message
  queue to serialize window operations from many threads. The multi-user front-end
  is selected when CONFIG_NX_MULTIUSER is defined in the NuttX configuration file.

graphics/nxfonts
  This is where the NXFONTS implementation resides.  This is a relatively low-
  level set of charset set/glyph management APIs.  See include/nuttx/nx/nxfonts.h

graphics/nxtk
  This is where the NXTOOLKIT implementation resides.  This toolkit is built on
  top of NX and works with either the single-user or multi-user NX version. See
  include/nuttx/nx/nxtk.h

graphics/nxwidgets
  At one time, I planned to put NXWIDGETS implementation here, but not anymore.

Installing New Fonts
^^^^^^^^^^^^^^^^^^^^

  There is a tool called bdf-converter in the directory tools/.  The bdf-converter
  program be used to convert fonts in Bitmap Distribution Format (BDF)
  into fonts that can be used in the NX graphics system.

  Below are general instructions for creating and installing a new font
  in the NX graphic system:

    1. Locate a font in BDF format,
    2. Use the bdf-converter program to convert the BDF font to the NuttX
       font format.  This will result in a C header file containing
       defintions.  That header file should be installed at, for example,
       graphics/nxfonts/nxfonts_myfont.h.

  Create a new NuttX configuration variable.  For example, suppose
  you define the following variable:  CONFIG_NXFONT_MYFONT.  Then
  you would need to:
  
    3. Define CONFIG_NXFONT_MYFONT=y in your NuttX configuration file.

  A font ID number has to be assigned for each new font.  The font ID
  is defined in the file include/nuttx/nx/nxfonts.h.  Those definitions
  have to be extended to support your new font.  Look at how the font ID
  enabled by CONFIG_NXFONT_SANS23X27 is defined and add an ID for your
  new font in a similar fashion:

    4. include/nuttx/nx/nxfonts.h. Add you new font as a possible system
       default font:
 
       #if defined(CONFIG_NXFONT_SANS23X27)
       # define NXFONT_DEFAULT FONTID_SANS23X27
       #elif defined(CONFIG_NXFONT_MYFONT)
       # define NXFONT_DEFAULT FONTID_MYFONT
       #endif

       Then define the actual font ID.  Make sure that the font ID value
       is unique:
 
       enum nx_fontid_e
       {
         FONTID_DEFAULT     = 0      /* The default font */
       #ifdef CONFIG_NXFONT_SANS23X27
         , FONTID_SANS23X27 = 1      /* The 23x27 sans serif font */
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
         , FONTID_MYFONT    = 2      /* My shiny, new font */
       #endif
       ...
 
  New Add the font to the NX build system.  There are several files that
  you have to modify to to this.  Look how the build system uses the
  font CONFIG_NXFONT_SANS23X27 for examaples:

    5. nuttx/graphics/Makefile.  This file needs logic to auto-generate
       a C source file from the header file that you generated with the
       the bdf-converter program.  Notice NXFONTS_FONTID=2; this must be
       set to the same font ID value that you defined in the
       include/nuttx/nx/nxfonts.h file.

       genfontsources:
         ifeq ($(CONFIG_NXFONT_SANS23X27),y)
	      @$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=1 EXTRADEFINES=$(EXTRADEFINES)
        endif
         ifeq ($(CONFIG_NXFONT_MYFONT),y)
	      @$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=2 EXTRADEFINES=$(EXTRADEFINES)
        endif

    6. nuttx/graphics/nxfonts/Make.defs.  Set the make variable NXFSET_CSRCS.
       NXFSET_CSRCS determines the name of the font C file to build when
       NXFONTS_FONTID=2:

       ifeq ($(CONFIG_NXFONT_SANS23X27),y)
       NXFSET_CSRCS	+= nxfonts_bitmaps_sans23x27.c
       endif
       ifeq ($(CONFIG_NXFONT_MYFONT),y)
       NXFSET_CSRCS	+= nxfonts_bitmaps_myfont.c
       endif

    7. nuttx/graphics/nxfonts/Makefile.sources.  This is the Makefile used
       in step 5 that will actually generate the font C file.  So, given
       your NXFONTS_FONTID=2, it needs to determine a prefix to use for
       auto-generated variable and function names and (again) the name of
       the autogenerated file to create (this must be the same name that
       was used in nuttx/graphics/nxfonts/Make.defs):

       ifeq ($(NXFONTS_FONTID),1)
       NXFONTS_PREFIX	:= g_sans23x27_
       GEN_CSRC	= nxfonts_bitmaps_sans23x27.c
       endif
       ifeq ($(NXFONTS_FONTID),2)
       NXFONTS_PREFIX	:= g_myfont_
       GEN_CSRC	= nxfonts_bitmaps_myfont.c
       endif

    8. graphics/nxfonts/nxfonts_bitmaps.c.  This is the file that contains
       the generic font structures.  It is used as a "template" file by
       nuttx/graphics/nxfonts/Makefile.sources to create your customized
       font data set.

       #if NXFONTS_FONTID == 1
       #  include "nxfonts_sans23x27.h"
       #elif NXFONTS_FONTID == 2
       #  include "nxfonts_myfont.h"
       #else
       #  error "No font ID specified"
       #endif

       Where nxfonts_myfont.h is the NuttX font file that we generated in
       step 2 using the bdf-converter tool.

    9. graphics/nxfonts/nxfonts_getfont.c.  Finally, we need to extend the
       logic that does the run-time font lookups so that can find our new
       font.  The lookup function is NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid).
       The new font information needs to be added to data structures used by
       that function:
 
       #ifdef CONFIG_NXFONT_SANS23X27
       extern const struct nx_fontpackage_s g_sans23x27_package;
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
       extern const struct nx_fontpackage_s g_myfont_package;
       #endif

       static FAR const struct nx_fontpackage_s *g_fontpackages[] =
       {
       #ifdef CONFIG_NXFONT_SANS23X27
       &g_sans23x27_package,
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
       &g_myfont_package,
       #endif
       NULL
       };

Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^

CONFIG_NX
  Enables overall support for graphics library and NX
CONFIG_NX_MULTIUSER
  Configures NX in multi-user mode
CONFIG_NX_NPLANES
  Some YUV color formats requires support for multiple planes, one for each
  color component.  Unless you have such special hardware, this value should be
  undefined or set to 1.
CONFIG_NX_DISABLE_1BPP, CONFIG_NX_DISABLE_2BPP,
CONFIG_NX_DISABLE_4BPP, CONFIG_NX_DISABLE_8BPP,
CONFIG_NX_DISABLE_16BPP, CONFIG_NX_DISABLE_24BPP, and
CONFIG_NX_DISABLE_32BPP
  NX supports a variety of pixel depths.  You can save some memory by disabling
  support for unused color depths.
CONFIG_NX_PACKEDMSFIRST
  If a pixel depth of less than 8-bits is used, then NX needs to know if the
  pixels pack from the MS to LS or from LS to MS
CONFIG_NX_MOUSE
  Build in support for mouse input.
CONFIG_NX_KBD
  Build in support of keypad/keyboard input.
CONFIG_NXTK_BORDERWIDTH
  Specifies with with of the border (in pixels) used with framed windows.
  The default is 4.
CONFIG_NXTK_BORDERCOLOR1 and CONFIG_NXTK_BORDERCOLOR2
  Specify the colors of the border used with framed windows.
  CONFIG_NXTK_BORDERCOLOR2 is the shadow side color and so is normally darker.
  The default is medium and dark grey, respectively
CONFIG_NXTK_AUTORAISE
  If set, a window will be raised to the top if the mouse position is over a
  visible portion of the window.  Default: A mouse button must be clicked over
  a visible portion of the window.
CONFIG_NXFONTS_CHARBITS
  The number of bits in the character set.  Current options are only 7 and 8.
  The default is 7.

CONFIG_NXFONT_SANS17X22
  This option enables support for a tiny, 17x22 san serif font
  (font ID FONTID_SANS17X22 == 14).
CONFIG_NXFONT_SANS20X26
  This option enables support for a tiny, 20x26 san serif font
  (font ID FONTID_SANS20X26 == 15).
CONFIG_NXFONT_SANS23X27
  This option enables support for a tiny, 23x27 san serif font
  (font ID FONTID_SANS23X27 == 1).
CONFIG_NXFONT_SANS22X29
  This option enables support for a small, 22x29 san serif font
  (font ID FONTID_SANS22X29 == 2).
CONFIG_NXFONT_SANS28X37
  This option enables support for a medium, 28x37 san serif font
  (font ID FONTID_SANS28X37 == 3).
CONFIG_NXFONT_SANS39X48
  This option enables support for a large, 39x48 san serif font
  (font ID FONTID_SANS39X48 == 4).
CONFIG_NXFONT_SANS17X23B
  This option enables support for a tiny, 17x23 san serif bold font
  (font ID FONTID_SANS17X23B == 16).
CONFIG_NXFONT_SANS20X27B
  This option enables support for a tiny, 20x27 san serif bold font
  (font ID FONTID_SANS20X27B == 17).
CONFIG_NXFONT_SANS22X29B
  This option enables support for a small, 22x29 san serif bold font
  (font ID FONTID_SANS22X29B == 5).
CONFIG_NXFONT_SANS28X37B
  This option enables support for a medium, 28x37 san serif bold font
  (font ID FONTID_SANS28X37B == 6).
CONFIG_NXFONT_SANS40X49B
  This option enables support for a large, 40x49 san serif bold font
  (font ID FONTID_SANS40X49B == 7).
CONFIG_NXFONT_SERIF22X29
  This option enables support for a small, 22x29 font (with serifs)
  (font ID FONTID_SERIF22X29 == 8).
CONFIG_NXFONT_SERIF29X37
  This option enables support for a medium, 29x37 font (with serifs)
  (font ID FONTID_SERIF29X37 == 9).
CONFIG_NXFONT_SERIF38X48
  This option enables support for a large, 38x48 font (with serifs)
  (font ID FONTID_SERIF38X48 == 10).
CONFIG_NXFONT_SERIF22X28B
  This option enables support for a small, 27x38 bold font (with serifs)
  (font ID FONTID_SERIF22X28B == 11).
CONFIG_NXFONT_SERIF27X38B
  This option enables support for a medium, 27x38 bold font (with serifs)
  (font ID FONTID_SERIF27X38B == 12).
CONFIG_NXFONT_SERIF38X49B
  This option enables support for a large, 38x49 bold font (with serifs)
  (font ID FONTID_SERIF38X49B == 13).

NX Multi-user only options:

CONFIG_NX_BLOCKING
  Open the client message queues in blocking mode.  In this case,
  nx_eventhandler() will not return until a message is received and processed.
CONFIG_NX_MXSERVERMSGS and CONFIG_NX_MXCLIENTMSGS
  Specifies the maximum number of messages that can fit in the message queues.
  No additional resources are allocated, but this can be set to prevent
  flooding of the client or server with too many messages (CONFIG_PREALLOC_MQ_MSGS
  controls how many messages are pre-allocated).


