netutils
^^^^^^^^

This directory contains most of the network applications contained
under the uIP-1.0 apps directory.  As the uIP apps/README says,
these applications "are not all heavily tested."  These uIP apps
include:

  dhcpc     - Dynamic Host Configuration Protocol (DHCP) client
  resolv    - uIP DNS resolver
  smtp      - Simple Mail Transfer Protocol (SMTP) client
  telnetd   - TELNET server
  webclient - HTTP web client
  webserver - HTTP web server

You may find additional information on these apps in the uIP forum
accessible through: http://www.sics.se/~adam/uip/index.php/Main_Page 

Additional applications that were not part of uIP (but which are
highly influenced by uIP) include:

  dhcpd     - Dynamic Host Configuration Protocol (DHCP) server
  tftpc     - TFTP client
  ftpc      - FTP client
  thttpd    - This is a port of Jef Poskanzer's THTTPD HTPPD server.
              See http://acme.com/software/thttpd/.

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
