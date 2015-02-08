#include <stdint.h>
#include <sys/types.h>
//#include "gmock/gmock.h"

#include "uORB/uORB.h"

/******************************************
 * uORB stubs (incomplete)
 *
 * TODO: use googlemock
******************************************/

orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data) {
	return (orb_advert_t)0;
}

int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data) {
	return 0;
}


