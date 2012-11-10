bin_PROGRAMS = qconf

qconf_SOURCES = qconf.cc qconf.h
BUILT_SOURCES = qconf.moc
qconf_CPPFLAGS = $(AM_CPPFLAGS)                 \
                 $(intl_CPPFLAGS)               \
                 -I$(top_srcdir)/libs/parser    \
                 -I$(top_builddir)/libs/images
qconf_CXXFLAGS = $(AM_CXXFLAGS) \
                 $(kf_CFLAGS)   \
                 $(qt4_CFLAGS)
qconf_LDADD = $(top_builddir)/libs/parser/libkconfig-parser.la  \
              $(top_builddir)/libs/images/libkconfig-images.a   \
              $(intl_LIBS) $(qt4_LIBS) $(qconf_EXTRA_LIBS)
CLEANFILES = qconf.moc
EXTRA_DIST = qconf.cc.patch

AM_V_MOC = $(AM_V_MOC_$(V))
AM_V_MOC_ = $(AM_V_MOC_$(AM_DEFAULT_VERBOSITY))
AM_V_MOC_0 = @echo "  MOC   " $@;

.h.moc:
	$(AM_V_MOC)$(MOC) -i $< -o $@
