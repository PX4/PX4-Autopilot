noinst_LIBRARIES = libkconfig-images.a

libkconfig_images_a_SOURCES = images.c_orig
nodist_libkconfig_images_a_SOURCES = images.c
BUILT_SOURCES = images.c images.h
CLEANFILES = images.c images.h

images.c: images.c_orig
	$(AM_V_GEN)$(SED) -r -e 's/^static //' $< >$@

images.h: images.c_orig
	$(AM_V_GEN)$(SED) -r -e '/^static (const char \*xpm_(.+)\[\]) = \{/!d; s//extern \1;/' \
	                  $< >$@
