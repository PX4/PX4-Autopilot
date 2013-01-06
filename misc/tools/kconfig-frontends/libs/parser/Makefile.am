SUFFIXES = .gperf

lib_LTLIBRARIES = libkconfig-parser.la
libkconfig_parser_la_SOURCES = yconf.y
dist_EXTRA_libkconfig_parser_la_SOURCES = \
    hconf.gperf lconf.l                   \
    confdata.c menu.c symbol.c util.c     \
    expr.c expr.h lkc.h lkc_proto.h
libkconfig_parser_la_CPPFLAGS = -DROOTMENU="\"$(root_menu)\""   \
                                -DCONFIG_=\"$(config_prefix)\"  \
                                $(intl_CPPFLAGS)
libkconfig_parser_la_CFLAGS = $(AM_CFLAGS) $(kf_CFLAGS)
libkconfig_parser_la_LDFLAGS = -release $(KCONFIGPARSER_LIB_VERSION) -no-undefined
libkconfig_parser_la_LIBADD = $(intl_LIBS)

kconfig_includedir = $(includedir)/kconfig
kconfig_include_HEADERS = list.h lkc.h expr.h lkc_proto.h

BUILT_SOURCES = hconf.c lconf.c
CLEANFILES = hconf.c lconf.c yconf.c
EXTRA_DIST = yconf.y.patch

AM_V_GPERF = $(AM_V_GPERF_$(V))
AM_V_GPERF_ = $(AM_V_GPERF_$(AM_DEFAULT_VERBOSITY))
AM_V_GPERF_0 = @echo "  GPERF " $@;

.gperf.c:
	$(AM_V_GPERF)$(GPERF) -t --output-file $@ -a -C -E -g -k 1,3,$$ -p -t $<

# The following rule produces a warning:
#   libs/parser/Makefile.am:41: user target `.l.c' defined here...
#  	/usr/share/automake-1.11/am/lex.am: ... overrides Automake target
#  	`.l.c' defined here
#
# This is expected, and can't be avoided (for now).
# That's because, when working with lex+yacc sources, the default is to
# build each files searately, and then link them together into the final
# output. But the Linux kernel's parser simply #include-s the lexer,
# so we can't put lconf.l into the _SOURCES (it's in EXTRA_SOURCES),
# and thus automake does not catch the need to call lex.
# Secondly, when flex is told to change the symbols' prefix (kconfig
# uses zconf in lieue of the original yy), then the output file is
# also renamed, but automake does not now that, and make would fail
# because it would think no file was generated.
.l.c:
	$(AM_V_LEX)$(LEXCOMPILE) -o$@ $<
