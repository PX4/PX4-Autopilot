#include <stdint.h>
#include <sys/types.h>
//#include "gmock/gmock.h"

#include "uORB/uORB.h"
#include <systemlib/param/param.h>

/******************************************
 * uORB stubs
******************************************/

/*
struct orb_metadata {
	const char *o_name;
	const size_t o_size;
};
typedef intptr_t	orb_advert_t;
extern orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data);
extern int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data);
*/

orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data) {
	return (orb_advert_t)0;
}

int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data) {
	return 0;
}

/******************************************
 * param stubs
******************************************/

//extern param_info_s * __param_start, __param_end;
struct param_info_s	param_info_base[5];
param_info_s *__param_start = &param_info_base[0];
param_info_s *__param_end = &param_info_base[4];
