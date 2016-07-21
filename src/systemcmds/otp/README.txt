
nsh> otp
otp: usage:
otp show - show all block contents and lock bitsotp write <blocknum> <32 hex bytes, LSB first, no spaces> <crc32 for bytes> - will print values after readback
otp read <blocknum> - will print 32 bytes readback from block (and the current CRC)
otp lock <blocknum> <blocknum> - will permanently lock the specified block number

# show is intended to be human readable
nsh> otp show
 0: L 5058340000ac26000010000000ffffffffffffffffffffffffffffffffffffff 55393c7b
 1: L 3296c91156c6a2cd9d472bd768e74ad9c731d43b9180a994f9b927517e41425a e50fcf15
 2: L 5207fe5842dd9bcd27f2077759ca977292334e8c366969f3797efaaa6940ad00 26df18c8
 3: L c0190f185410e5fd13dce5b37b8f8030ab84e1fa1c026088eb1ee4b224fb662b ae616df5
 4: L a800a8110a956c55fe43698fa584e9a1d7f8b10495e3460c1de681c0c7b89875 3245288a
 5: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 6: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 7: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 8: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 9: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
10: U 5058340000002600001000000000000000000000000000000000000000000000 2fab057d
11: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
12: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
13: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
14: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
15: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6

# the readback for writing to block 10 failed (OTP bits can _only_ be changed to zero)
# even if unlocked there is NO WAY to change the back to a one.  So be careful even
# if testing
nsh> otp write 10 ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
otp: Write not accepted

# Because of this, we check a crc32 that must be included with each write command
# if wrong we will not do the write
nsh> otp write 10 ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea67
otp: CRC does not match bytes

# The _only_ response that indicates success is WRITTEN
# This demonstrates writing ffs
nsh> otp write 11 ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
WRITTEN

# This demonstrates writing some other string of bytes
nsh> otp write 11 5058340000ac26000010000000ffffffffffffffffffffffffffffffffffffff 55393c7b
WRITTEN

# See the happy bytes now on block 11
nsh> otp show
 0: L 5058340000ac26000010000000ffffffffffffffffffffffffffffffffffffff 55393c7b
 1: L 3296c91156c6a2cd9d472bd768e74ad9c731d43b9180a994f9b927517e41425a e50fcf15
 2: L 5207fe5842dd9bcd27f2077759ca977292334e8c366969f3797efaaa6940ad00 26df18c8
 3: L c0190f185410e5fd13dce5b37b8f8030ab84e1fa1c026088eb1ee4b224fb662b ae616df5
 4: L a800a8110a956c55fe43698fa584e9a1d7f8b10495e3460c1de681c0c7b89875 3245288a
 5: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 6: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 7: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 8: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 9: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
10: U 5058340000002600001000000000000000000000000000000000000000000000 2fab057d
11: U 5058340000ac26000010000000ffffffffffffffffffffffffffffffffffffff 55393c7b
12: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
13: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
14: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
15: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6

# To prevent locking incorrect blocks, the block number must be given twice
nsh> otp lock 10
otp: Invalid arguments
otp: usage:
otp show - show all block contents and lock bitsotp write <blocknum> <32 hex bytes, LSB first, no spaces> <crc32 for bytes> - will print values after readback
otp read <blocknum> - will print 32 bytes readback from block (and the current CRC)
otp lock <blocknum> <blocknum> - will permanently lock the specified block number

# The _only_ response that indicates success for the lock command is LOCKED
nsh> otp lock 10 10
LOCKED

# Notice that block 10 is now locked
nsh> otp show
 0: L 5058340000ac26000010000000ffffffffffffffffffffffffffffffffffffff 55393c7b
 1: L 3296c91156c6a2cd9d472bd768e74ad9c731d43b9180a994f9b927517e41425a e50fcf15
 2: L 5207fe5842dd9bcd27f2077759ca977292334e8c366969f3797efaaa6940ad00 26df18c8
 3: L c0190f185410e5fd13dce5b37b8f8030ab84e1fa1c026088eb1ee4b224fb662b ae616df5
 4: L a800a8110a956c55fe43698fa584e9a1d7f8b10495e3460c1de681c0c7b89875 3245288a
 5: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 6: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 7: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 8: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
 9: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
10: L 5058340000002600001000000000000000000000000000000000000000000000 2fab057d
11: U 5058340000ac26000010000000ffffffffffffffffffffffffffffffffffffff 55393c7b
12: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
13: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
14: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
15: U ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff e666fea6
nsh>
