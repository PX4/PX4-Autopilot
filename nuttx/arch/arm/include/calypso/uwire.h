#ifndef _CALYPSO_UWIRE_H
#define _CALYPSO_UWIRE_H
void uwire_init(void);
int uwire_xfer(int cs, int bitlen, const void *dout, void *din);
#endif

