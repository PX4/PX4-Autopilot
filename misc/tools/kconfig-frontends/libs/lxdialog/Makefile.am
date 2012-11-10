noinst_LIBRARIES = libkconfig-lxdialog.a

libkconfig_lxdialog_a_SOURCES = \
    checklist.c                 \
    dialog.h                    \
    inputbox.c                  \
    menubox.c                   \
    textbox.c                   \
    util.c                      \
    yesno.c

libkconfig_lxdialog_a_CPPFLAGS = $(AM_CPPFLAGS)                   \
                                 -DCURSES_LOC='"${CURSES_LOC}"'   \
                                 $(intl_CPPFLAGS)
liblxdialog_a_CFLAGS = $(AM_CFLAGS) $(kf_CFLAGS)
