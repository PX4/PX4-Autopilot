/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: generator.h 4 2007-08-27 13:11:03Z xtimor $
 *
 */

#ifndef __NMEA_GENERATOR_H__
#define __NMEA_GENERATOR_H__

#include "info.h"

#ifdef  __cplusplus
extern "C" {
#endif

/*
 * high level
 */

struct _nmeaGENERATOR;

enum nmeaGENTYPE
{
    NMEA_GEN_NOISE = 0,
    NMEA_GEN_STATIC,
    NMEA_GEN_ROTATE,

    NMEA_GEN_SAT_STATIC,
    NMEA_GEN_SAT_ROTATE,
    NMEA_GEN_POS_RANDMOVE,

    NMEA_GEN_LAST
};

struct _nmeaGENERATOR * nmea_create_generator(int type, nmeaINFO *info);
void    nmea_destroy_generator(struct _nmeaGENERATOR *gen);

int     nmea_generate_from(
        char *buff, int buff_sz,    /* buffer */
        nmeaINFO *info,             /* source info */
        struct _nmeaGENERATOR *gen, /* generator */
        int generate_mask           /* mask of sentence`s (e.g. GPGGA | GPGSA) */
        );

/*
 * low level
 */

typedef int (*nmeaNMEA_GEN_INIT)(struct _nmeaGENERATOR *gen, nmeaINFO *info);
typedef int (*nmeaNMEA_GEN_LOOP)(struct _nmeaGENERATOR *gen, nmeaINFO *info);
typedef int (*nmeaNMEA_GEN_RESET)(struct _nmeaGENERATOR *gen, nmeaINFO *info);
typedef int (*nmeaNMEA_GEN_DESTROY)(struct _nmeaGENERATOR *gen);

typedef struct _nmeaGENERATOR
{
    void                *gen_data;
    nmeaNMEA_GEN_INIT    init_call;
    nmeaNMEA_GEN_LOOP    loop_call;
    nmeaNMEA_GEN_RESET   reset_call;
    nmeaNMEA_GEN_DESTROY destroy_call;
    struct _nmeaGENERATOR *next;

} nmeaGENERATOR;

int     nmea_gen_init(nmeaGENERATOR *gen, nmeaINFO *info);
int     nmea_gen_loop(nmeaGENERATOR *gen, nmeaINFO *info);
int     nmea_gen_reset(nmeaGENERATOR *gen, nmeaINFO *info);
void    nmea_gen_destroy(nmeaGENERATOR *gen);
void    nmea_gen_add(nmeaGENERATOR *to, nmeaGENERATOR *gen);

#ifdef  __cplusplus
}
#endif

#endif /* __NMEA_GENERATOR_H__ */
