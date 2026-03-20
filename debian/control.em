Source: @(PREFIX)px4-targets
Section: admin
Priority: optional
Maintainer: Auterion <distro@@auterion.com>
Build-Depends-Indep:
 debhelper (>= 11),
Standards-Version: 4.4.1

Package: @(PREFIX)px4-targets-production
Architecture: all
Depends: ${misc:Depends}
Description: Production binaries for NuttX targets

Package: @(PREFIX)px4-targets-development
Architecture: all
Depends: ${misc:Depends}
Description: Development binaries for NuttX targets
