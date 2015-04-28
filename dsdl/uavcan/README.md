Standard DSDL definitions
=========================

For details, please refer to the [UAVCAN specification](http://uavcan.org/).

## Standard DTID ranges

- Messages: [256, 1791)
- Services: [64, 447)

Rest are reserved for vendor-specific data types.

## Standard ID grouping

Note that all unallocated space can be claimed later.

### Messages

| ID                   | Types                                    | Note                                     |
| -------------------- | ---------------------------------------- | ---------------------------------------- |
| [256, 260)           | protocol.*                               | Highest priority                         |
| [260, 900)           | equipment.*                              | High priority                            |
| [1000, 1050)         | protocol.*                               |                                          |
| [1400, 1700)         | equipment.*                              | Low priority                             |
| 1780                 | mavlink.Message                          |                                          |
| [1785, 1791)         | protocol.debug.*                         | Lowest priority                          |

### Services

| ID                   | Types                                    | Note                                     |
| -------------------- | ---------------------------------------- | ---------------------------------------- |
| [100, 110)           | equipment.*                              |                                          |
| [200, 250)           | protocol.*                               |                                          |
