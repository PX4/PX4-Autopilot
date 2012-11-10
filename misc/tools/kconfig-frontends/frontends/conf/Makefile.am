bin_PROGRAMS = conf

conf_SOURCES = conf.c
conf_CPPFLAGS = $(AM_CPPFLAGS)      \
                $(intl_CPPFLAGS)    \
                -I$(top_srcdir)/libs/parser
conf_CFLAGS = $(AM_CFLAGS) $(kf_CFLAGS)
conf_LDADD = $(top_builddir)/libs/parser/libkconfig-parser.la  \
             $(intl_LIBS)                                      \
             $(conf_EXTRA_LIBS)
