netutils README.txt
^^^^^^^^^^^^^^^^^^^

Contents
--------

  - uIP Applications
  - Other Network Applications
  - Tips for Using Telnetd
  - Tips for Using DHCPC

uIP Applications
^^^^^^^^^^^^^^^^

This directory contains most of the network applications contained
under the uIP-1.0 apps directory.  As the uIP apps/README says,
these applications "are not all heavily tested."  These uIP apps
include:

  dhcpc     - Dynamic Host Configuration Protocol (DHCP) client.  See
              apps/include/netutils/dhcpc.h for interface information.
  resolv    - uIP DNS resolver.  See apps/include/netutils/resolv.h
              for interface information.
  smtp      - Simple Mail Transfer Protocol (SMTP) client.  See
              apps/include/netutils/smtp.h for interface information.
  webclient - HTTP web client.  See apps/include/netutils/webclient.h
              for interface information.
  webserver - HTTP web server.  See apps/include/netutils/httpd.h
              for interface information.

You may find additional information on these apps in the uIP forum
accessible through: http://www.sics.se/~adam/uip/index.php/Main_Page 

Other Network Applications
^^^^^^^^^^^^^^^^^^^^^^^^^^

Additional applications that were not part of uIP (but which are
highly influenced by uIP) include:

  dhcpd     - Dynamic Host Configuration Protocol (DHCP) server.  See
              apps/include/netutils/dhcpd.h for interface information.
  discover  - This daemon is useful for discovering devices in local
              networks, especially with DHCP configured devices.  It
              listens for UDP broadcasts which also can include a
              device class so that groups of devices can be discovered.
              It is also possible to address all classes with a kind of
              broadcast discover. (Contributed by Max Holtzberg).
  tftpc     - TFTP client.  See apps/include/netutils/tftp.h
              for interface information.
  telnetd   - TELNET server.  This is the Telnet logic adapted from
              uIP and generalized for use as the front end to any
              shell.  The telnet daemon creates sessions that are
              "wrapped" as character devices and mapped to stdin,
              stdout, and stderr.  Now the telnet session can be
              inherited by spawned tasks.
  ftpc      - FTP client.  See apps/include/ftpc.h for interface
              information.
  ftpd      - FTP server.   See apps/include/netutils/ftpd.h for interface
              information.
  thttpd    - This is a port of Jef Poskanzer's THTTPD HTPPD server.
              See http://acme.com/software/thttpd/ for general THTTPD
              information.  See apps/include/netutils/thttpd.h
              for interface information. Applications using this thttpd
              will need to provide an appconfig file in the configuration
              directory with instruction to build applications like:

               CONFIGURED_APPS += uiplib
               CONFIGURED_APPS += thttpd
  xmlrpc    - The Embeddable Lightweight XML-RPC Server discussed at
              http://www.drdobbs.com/web-development/an-embeddable-lightweight-xml-rpc-server/184405364

Tips for Using Telnetd
^^^^^^^^^^^^^^^^^^^^^^

Telnetd is set up to be the front end for a shell.  The primary use of
Telnetd in NuttX is to support the NuttShell (NSH) Telnet front end.  See
apps/include/netutils/telnetd.h for information about how to incorporate
Telnetd into your custom applications.

To enable and link the Telnetd daemon, you need to include the following in 
in your appconfig (apps/.config) file:

  CONFIGURED_APPS += uiplib
  CONFIGURED_APPS += netutils/telnetd

Also if the Telnet console is enabled, make sure that you have the following
set in the NuttX configuration file or else the performance will be very bad
(because there will be only one character per TCP transfer):
  
  CONFIG_STDIO_BUFFER_SIZE   Some value >= 64
  CONFIG_STDIO_LINEBUFFER=y  Since Telnetd is line oriented, line buffering
                             is optimal.

Tips for Using DHCPC
^^^^^^^^^^^^^^^^^^^^

If you use DHCPC/D, then some special configuration network options are
required.  These include:

  CONFIG_NET=y               Of course
  CONFIG_NSOCKET_DESCRIPTORS And, of course, you must allocate some
                             socket descriptors.
  CONFIG_NET_UDP=y           UDP support is required for DHCP
                             (as well as various other UDP-related
                             configuration settings).
  CONFIG_NET_BROADCAST=y     UDP broadcast support is needed.
  CONFIG_NET_BUFSIZE=650     The client must be prepared to receive
  (or larger)                DHCP messages of up to 576 bytes (excluding
                             Ethernet, IP, or UDP headers and FCS).

