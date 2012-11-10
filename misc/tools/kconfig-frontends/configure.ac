#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

#---------------------------------------------------------------------------
# Global initialisation

#----------------------------------------
# Prepare autoconf
AC_PREREQ([2.67])
AC_INIT(
    [kconfig-frontends],
    [m4_esyscmd_s([./scripts/version.sh])],
    [yann.morin.1998@free.fr])
AC_CONFIG_SRCDIR([frontends/conf/conf.c])
# Use a config.h to avoid brazilions -DHAVE_FOO on compile lines
AC_CONFIG_HEADERS([scripts/.autostuff/config.h])
AC_CONFIG_AUX_DIR([scripts/.autostuff/scripts])
AC_CONFIG_MACRO_DIR([scripts/.autostuff/m4])

#----------------------------------------
# Prepare automake

# We want to allow the user to override our default program-prefix,
# so we must set-it now, before automake has a chance to interpret
# it, but after the options are parsed, so as not to overwrite the
# value (if any) set by the user
AS_IF(
    [test "$program_prefix" = NONE],
    [program_prefix=kconfig-])

AM_INIT_AUTOMAKE

AS_IF(
    [test "$(${srcdir}/scripts/version.sh --internal)" = "hg"],
    [AM_SILENT_RULES],
    [AM_SILENT_RULES([yes])])

AS_IF(
    [test $AM_DEFAULT_VERBOSITY -eq 0],
    [SILENT_MAKEFLAGS="--no-print-directory -s"],
    [SILENT_MAKEFLAGS=""])
AC_SUBST([SILENT_MAKEFLAGS])

#----------------------------------------
# Prepare libtool
LT_PREREQ([2.2.6])
LT_INIT([disable-static])

#---------------------------------------------------------------------------
# Set misc options

# By default, do not build with -Wall, unless the user asks for it
AC_ARG_ENABLE(
    [wall],
    [AS_HELP_STRING(
        [--enable-wall],
        [build with -Wall (default=no)])],
    [AS_CASE(
        ["$enableval"],
        [yes], [wall_CFLAGS=-Wall],
        [*],   [wall_CFLAGS=""])])
AC_SUBST([wall_CFLAGS],[${wall_CFLAGS}])

# By default, do not build with -Werror, unless the user asks for it
AC_ARG_ENABLE(
    [werror],
    [AS_HELP_STRING(
        [--enable-werror],
        [build with -Werror (default=no)])],
    [AS_CASE(
        ["$enableval"],
        [yes], [werror_CFLAGS=-Werror],
        [*],   [werror_CFLAGS=""])])
AC_SUBST([werror_CFLAGS],[${werror_CFLAGS}])

# Although there is a default (="linux") in the code, we do provide
# a default here, to get a consistent autostuff behavior
AC_ARG_ENABLE(
    [root-menu-prompt],
    [AS_HELP_STRING(
        [--enable-root-menu-prompt=PROMPT],
        [set the root-menu prompt (default=Configuration)])],
    [AS_CASE(
        ["$enableval"],
        [yes], [root_menu=Configuration],
        [no],  [root_menu=],
               [# Escape the $ signs, otherwise they would get munged by make
                # Also, append a space at the end, to separate the package
                # name from the literal 'Configuration'
                root_menu="$( echo "$enableval" |sed -r -e 's/\$/\\$$/g;' )"])])
AC_SUBST([root_menu], [${root_menu=Configuration}])

AC_ARG_ENABLE(
    [config-prefix],
    [AS_HELP_STRING(
        [--enable-config-prefix=PREFIX],
        [the prefix to the config option (default=CONFIG_)])],
    [AS_CASE(
        ["$enableval"],
        [*" "*],[AC_MSG_ERROR([config prefix can not contain spaces: '$enableval'])],
        [yes],  [config_prefix=CONFIG_],
        [no],   [config_prefix=],
                [config_prefix=$enableval])])
AC_SUBST([config_prefix], [${config_prefix-CONFIG_}])

AC_ARG_ENABLE(
    [utils],
    [AS_HELP_STRING(
        [--disable-utils],
        [install utilities to manage .config files (default=no)])])
AC_SUBST([enable_utils], [${enable_utils:-yes}])

AC_ARG_ENABLE(
    [L10n],
    [AS_HELP_STRING(
        [--disable-L10n],
        [enable localisation (L10n) (default=auto)])])
AC_SUBST([enable_L10n], [${enable_L10n:-yes}])

#----------------------------------------
# Options to selectively enable/disable frontends
# All are selected by default
AC_ARG_ENABLE(
    [conf],
    [AS_HELP_STRING(
        [--disable-conf],
        [conf, the stdin-based frontend (default=auto)])])
AC_SUBST([enable_conf], [${enable_conf:-auto}])
AC_ARG_ENABLE(
    [mconf],
    [AS_HELP_STRING(
        [--disable-mconf],
        [mconf, the traditional ncurses-based frontend (default=auto)])])
AC_SUBST([enable_mconf], [${enable_mconf:-auto}])
AC_ARG_ENABLE(
    [nconf],
    [AS_HELP_STRING(
        [--disable-nconf],
        [nconf, the modern ncurses-based frontend (default=auto)])])
AC_SUBST([enable_nconf], [${enable_nconf:-auto}])
AC_ARG_ENABLE(
    [gconf],
    [AS_HELP_STRING(
        [--disable-gconf],
        [gconf, the GTK-based frontend (default=auto)])])
AC_SUBST([enable_gconf], [${enable_gconf:-auto}])
AC_ARG_ENABLE(
    [qconf],
    [AS_HELP_STRING(
        [--disable-qconf],
        [qconf, the QT-based frontend (default=auto)])])
AC_SUBST([enable_qconf], [${enable_qconf:-auto}])

AC_ARG_ENABLE(
    [frontends],
    [AS_HELP_STRING(
        [--enable-frontends=list],
        [enables only the set of frontends in comma-separated 'list'
         (default: auto selection), takes precedence over all
         --enable-*conf, above])],
    [for f in conf mconf nconf gconf qconf; do
        AS_CASE(
            ["$enableval"],
            [yes],      [eval enable_$f=yes],
            ["$f"],     [eval enable_$f=yes],
            ["$f",*],   [eval enable_$f=yes],
            [*,"$f"],   [eval enable_$f=yes],
            [*,"$f",*], [eval enable_$f=yes],
                        [eval enable_$f=no])
     done])
AC_SUBST([enable_frontends])

#----------------------------------------
# What extra CFLAGS we will be using
AC_SUBST([kf_CFLAGS], ["$wall_CFLAGS $werror_CFLAGS"])

#----------------------------------------
# Dependencies that will be needed, depending on the frontends
AS_CASE(
    ["$enable_mconf":"$enable_nconf"],
    [*yes*],  [need_curses=yes],
    [*auto*], [need_curses=auto],
              [need_curses=no])
[need_panel_menu="$enable_nconf"]
AS_CASE(
    ["$enable_gconf":"$enable_qconf"],
    [*yes*],  [need_pkgconfig=yes],
    [*auto*], [need_pkgconfig=yes],
              [need_pkgconfig=no ])
[need_gtk="$enable_gconf"]
[need_qt="$enable_qconf"]

#---------------------------------------------------------------------------
# Now check we have the required stuff

#----------------------------------------
# Some program checks
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CXX
AC_C_INLINE
AC_PROG_MAKE_SET
AC_CHECK_PROGS(
    [GPERF],
    [gperf])
AS_IF(
    [test -z "$GPERF"],
    [AC_MSG_ERROR([can not find gperf])])
AS_IF(
    [test "$need_pkgconfig" = "yes"],
    [PKG_PROG_PKG_CONFIG()])
AC_PROG_LEX
AC_SUBST([AM_LFLAGS], ["-L -P zconf"])
AC_PROG_YACC
AC_SUBST([AM_YFLAGS], ["-t -l -p zconf"])

#----------------------------------------
# Check for standard headers
AC_HEADER_STDC
AC_HEADER_STDBOOL
AC_CHECK_HEADERS([ fcntl.h limits.h locale.h ])
AC_CHECK_HEADERS([ stdlib.h string.h sys/time.h unistd.h ])
AC_TYPE_SIZE_T

#----------------------------------------
# Checks for library functions.
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_FUNC_ALLOCA
AC_CHECK_FUNCS([ bzero memmove memset ])
AC_CHECK_FUNCS([ strcasecmp strchr strcspn strdup strncasecmp strpbrk strrchr strspn strtol ])
AC_CHECK_FUNCS([ gettimeofday mkdir regcomp setlocale uname ])

#----------------------------------------
# Check for gettext, for the kconfig frontends
[has_gettext="$enable_L10n"]
AS_IF(
    [test "$has_gettext" = "yes"],
    [AC_CHECK_HEADERS(
        [libintl.h],
        [ac_ct_gettext_hdr=$ac_header; break],
        [has_gettext=no])])
AS_IF(
    [test "$has_gettext" = "yes"],
    [AC_CHECK_DECL(
        [gettext],,
        [has_gettext=no],
        [#include <$ac_ct_gettext_hdr>])])
AS_IF(
    [test "$has_gettext" = "yes"],
    [LIBS_old="$LIBS"
     LIBS=
     AC_SEARCH_LIBS(
        [gettext],
        [intl],,
        [has_gettext=no])
    intl_LIBS="$LIBS"
    LIBS="$LIBS_old"])
AS_IF(
    [test "$has_gettext" = "no"],
    [intl_CPPFLAGS=-DKBUILD_NO_NLS])
AC_SUBST([intl_CPPFLAGS])
AC_SUBST([intl_LIBS])

#----------------------------------------
# Check for ncurses, for the mconf & nconf frontends
AS_IF(
    [test "$need_curses" = "yes" -o "$need_curses" = "auto"],
    [AC_SUBST([CURSES_LOC])
     AC_SUBST([ncurses_LIBS])
     LIBS_old="$LIBS"
     LIBS=
     AC_CHECK_HEADERS(
        [ncursesw/curses.h ncurses/ncurses.h ncurses/curses.h ncurses.h curses.h],
        [CURSES_LOC=$ac_header; break])
     AS_IF(
        [test -z "$CURSES_LOC"],
        [AS_IF(
            [test "$need_curses" = "yes"],
            [AC_MSG_ERROR([could not find curses headers (frontends: mconf/nconf)])],
            [has_curses=no])])
     AC_SEARCH_LIBS(
        [initscr],
        [ncursesw ncurses curses],
        [ac_ct_curses_lib_found=yes; break])
     AS_IF(
        [test -z "$ac_ct_curses_lib_found"],
        [AS_IF(
            [test "$need_curses" = "yes"],
            [AC_MSG_ERROR([could not find curses library (frontends: mconf/nconf)])],
            [has_curses=no])])
     ncurses_LIBS="$LIBS"
     LIBS=$LIBS_old])

AS_IF(
    [test "$has_curses" = "no" ],
    [enable_mconf=no; enable_nconf=no])

#----------------------------------------
# Check for libpanel and libmenu, for the nconf frontend
AS_IF(
    [test "$need_panel_menu" = "yes" -o "$need_panel_menu" = "auto"],
    [AC_SUBST([ncurses_extra_LIBS])
     AC_SUBST([ncurses_extra_CPPFLAGS])
     AS_CASE(
        [$CURSES_LOC],
        [ncursesw/*],[ncurses_extra_CPPFLAGS="-I/usr/include/ncursesw"],
        [ncurses/*],[ncurses_extra_CPPFLAGS="-I/usr/include/ncurses"])
     LIBS_old="$LIBS"
     LIBS=
     AC_SEARCH_LIBS(
        [new_panel],
        [panelw panel],
        [ac_ct_panel_lib_found=yes; break],,
        [$ncurses_LIBS])
     AS_IF(
        [test -z "$ac_ct_panel_lib_found"],
        [AS_IF(
            [test "$need_panel_menu" = "yes"],
            [AC_MSG_ERROR([could not find libpanel library (frontend: nconf)])],
            [has_panel_menu=no])])
     AC_SEARCH_LIBS(
        [menu_init],
        [menuw menu],
        [ac_ct_menu_lib_found=yes; break],,
        [$ncurses_LIBS])
     AS_IF(
        [test -z "$ac_ct_panel_lib_found"],
        [AS_IF(
            [test "$need_panel_menu" = "yes"],
            [AC_MSG_ERROR([could not find libmenu library (frontend: nconf)])],
            [has_panel_menu=no])])
     ncurses_extra_LIBS="$LIBS"
     LIBS=$LIBS_old])

AS_IF(
    [test "$has_panel_menu" = "no" ],
    [enable_nconf=no])

#----------------------------------------
# Check headers and libs for gconf
AS_IF(
    [test "$need_gtk" = "yes" -o "$need_gtk" = "auto"],
    [PKG_CHECK_MODULES(
        [gtk],
        [gtk+-2.0 gmodule-2.0 libglade-2.0],
        [has_gtk=yes],
        [AS_IF(
            [test "$need_gtk" = "yes"],
            [AC_MSG_ERROR([could not find GTK+ headers and/or libraries (frontend: gconf)])],
            [has_gtk=no])])])

AS_IF(
    [test "$has_gtk" = "no" ],
    [enable_gconf=no])

#----------------------------------------
# Check headers and libs for qconf
AS_IF(
    [test "$need_qt" = "yes" -o "$need_qt" = "auto"],
    [PKG_CHECK_MODULES(
        [qt4],
        [QtCore QtGui Qt3Support],
        [has_qt=yes; need_moc="$need_qt"],
        [AS_IF(
            [test "$need_qt" = "yes"],
            [AC_MSG_ERROR([could not find QT4 headers and/or libraries (frontend: qconf)])],
            [has_qt=no; need_moc=no])])])

AC_ARG_VAR([MOC], [Qt meta object compiler (moc) command])
AS_IF(
    [test "$need_moc" = "yes" -o "$need_moc" = "auto"],
    [QT4_BINDIR=`$PKG_CONFIG Qt --variable bindir`
     AC_PATH_PROGS(
        [MOC],
        [moc-qt4 moc],,
        [$QT4_BINDIR:$PATH])
     AS_IF(
        [test -n "$MOC"],
        [has_moc=yes],
        [AS_IF(
            [test "$need_moc" = "yes"],
            [AC_MSG_ERROR([could not find moc (frontend: qconf)])],
            [has_moc=no])])])

AS_IF(
    [test "$has_qt" = "no" -o "$has_moc" = "no"],
    [enable_qconf=no])

#----------------------------------------
# Per-frontends extra libraries
AC_ARG_VAR([conf_EXTRA_LIBS],  [Extra libraries to build the conf frontend] )
AC_ARG_VAR([gconf_EXTRA_LIBS], [Extra libraries to build the gconf frontend])
AC_ARG_VAR([mconf_EXTRA_LIBS], [Extra libraries to build the mconf frontend])
AC_ARG_VAR([nconf_EXTRA_LIBS], [Extra libraries to build the nconf frontend])
AC_ARG_VAR([qconf_EXTRA_LIBS], [Extra libraries to build the qconf frontend])

#---------------------------------------------------------------------------
# Now, we know what frontends to enable
AS_IF([test "$enable_conf"  = "auto"], [enable_conf=yes ])
AS_IF([test "$enable_gconf" = "auto"], [enable_gconf=yes])
AS_IF([test "$enable_mconf" = "auto"], [enable_mconf=yes])
AS_IF([test "$enable_nconf" = "auto"], [enable_nconf=yes])
AS_IF([test "$enable_qconf" = "auto"], [enable_qconf=yes])

#----------------------------------------
# Check if the lxdialog library should be built
AS_IF(
    [test "$enable_mconf" = "yes"],
    [need_lxdialog=yes],
    [need_lxdialog=no])

#----------------------------------------
# Check if the images library should be built
AS_IF(
    [test "$enable_gconf" = "yes" -o "$enable_qconf" = "yes"],
    [need_images=yes],
    [need_images=no])

#----------------------------------------
# Setup automake conditional build
AM_CONDITIONAL(
    [COND_conf],
    [test "$enable_conf" = "yes"])
AM_CONDITIONAL(
    [COND_mconf],
    [test "$enable_mconf" = "yes"])
AM_CONDITIONAL(
    [COND_nconf],
    [test "$enable_nconf" = "yes"])
AM_CONDITIONAL(
    [COND_gconf],
    [test "$enable_gconf" = "yes"])
AM_CONDITIONAL(
    [COND_qconf],
    [test "$enable_qconf" = "yes"])
AM_CONDITIONAL(
    [COND_lxdialog],
    [test "$need_lxdialog" = "yes"])
AM_CONDITIONAL(
    [COND_images],
    [test "$need_images" = "yes"])
AM_CONDITIONAL(
    [COND_utils],
    [test "$enable_utils" = "yes"])
AM_CONDITIONAL(
    [COND_utils_gettext],
    [test "$has_gettext" = "yes"])

#----------------------------------------
# Get the version to apply to the parser shared library
AC_SUBST(
    [KCONFIGPARSER_LIB_VERSION],
    [m4_esyscmd_s([./scripts/version.sh --plain])])

#----------------------------------------
# Finalise
AC_CONFIG_FILES([
    Makefile
    docs/Makefile
    libs/Makefile
    libs/images/Makefile
    libs/lxdialog/Makefile
    libs/parser/Makefile
    frontends/Makefile
    frontends/conf/Makefile
    frontends/mconf/Makefile
    frontends/nconf/Makefile
    frontends/gconf/Makefile
    frontends/qconf/Makefile
    utils/Makefile
    scripts/Makefile
])
AC_OUTPUT

#----------------------------------------
# Pretty-print the configuration settings
[fe_list=]
AS_IF([test "$enable_conf"  = "yes"], [fe_list="$fe_list conf" ])
AS_IF([test "$enable_gconf" = "yes"], [fe_list="$fe_list gconf"])
AS_IF([test "$enable_mconf" = "yes"], [fe_list="$fe_list mconf"])
AS_IF([test "$enable_nconf" = "yes"], [fe_list="$fe_list nconf"])
AS_IF([test "$enable_qconf" = "yes"], [fe_list="$fe_list qconf"])

[lib_list=]
AS_IF(
    [test "$enable_shared" = "yes"],
    [lib_list="$lib_list shared (version: $KCONFIGPARSER_LIB_VERSION)"])
AS_IF(
    [test "$enable_static" = "yes"],
    [lib_list="$lib_list${lib_list:+,} static"])

AC_MSG_NOTICE()
AC_MSG_NOTICE([Configured with:])
AC_MSG_NOTICE([- parser library     :$lib_list])
AC_MSG_NOTICE([  - root-menu prompt : $root_menu])
AC_MSG_NOTICE([  - config prefix    : $config_prefix])
AC_MSG_NOTICE([- frontends          :$fe_list])
AC_MSG_NOTICE([  - transform name   : $program_transform_name])
AC_MSG_NOTICE([  - localised        : $has_gettext])
AC_MSG_NOTICE([- install utilities  : $enable_utils])
AC_MSG_NOTICE([- CFLAGS CXXFLAGS    : $wall_CFLAGS $werror_CFLAGS])
