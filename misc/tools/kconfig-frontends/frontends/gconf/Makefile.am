bin_PROGRAMS = gconf

gconf_SOURCES = gconf.c gconf.glade
gconf_CPPFLAGS = $(AM_CPPFLAGS)                 \
                 $(intl_CPPFLAGS)               \
                 -I$(top_srcdir)/libs/parser    \
                 -I$(top_builddir)/libs/images  \
                 -DGUI_PATH='"$(pkgdatadir)/gconf.glade"'
gconf_CFLAGS = $(AM_CFLAGS) \
               $(kf_CFLAGS) \
               $(gtk_CFLAGS)
gconf_LDADD = $(top_builddir)/libs/parser/libkconfig-parser.la  \
              $(top_builddir)/libs/images/libkconfig-images.a   \
              $(intl_LIBS) $(gtk_LIBS) $(gconf_EXTRA_LIBS)
gconfdir = $(pkgdatadir)
gconf_DATA = gconf.glade
EXTRA_DIST = gconf.c.patch
