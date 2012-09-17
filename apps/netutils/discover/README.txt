apps/netutils/discover README.txt
=================================

This daemon is useful for discovering devices in local networks, especially
with DHCP configured devices.  It listens for UDP broadcasts which also can
include a device class so that groups of devices can be discovered. It is
also possible to address all classes with a kind of broadcast discover.

See nuttx/tools/discover.py for a client example.
