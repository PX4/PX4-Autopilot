/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: generator.c 17 2008-03-11 11:56:11Z xtimor $
 *
 */

#include "nmea/gmath.h"
#include "nmea/generate.h"
#include "nmea/generator.h"
#include "nmea/context.h"

#include <string.h>
#include <stdlib.h>

#if defined(NMEA_WIN) && defined(_MSC_VER)
# pragma warning(disable: 4100) /* unreferenced formal parameter */
#endif

float nmea_random(float min, float max)
{
    static float rand_max = MAX_RAND;//RAND_MAX; //nuttx defines MAX_RAND instead of RAND_MAX
    float rand_val = rand();
    float bounds = max - min;
    return min + (rand_val * bounds) / rand_max;
}

/*
 * low level
 */

int nmea_gen_init(nmeaGENERATOR *gen, nmeaINFO *info)
{
    int RetVal = 1; int smask = info->smask;
    nmeaGENERATOR *igen = gen;

    nmea_zero_INFO(info);
    info->smask = smask;

    info->lat = NMEA_DEF_LAT;
    info->lon = NMEA_DEF_LON;

    while(RetVal && igen)
    {
        if(igen->init_call)
            RetVal = (*igen->init_call)(igen, info);
        igen = igen->next;
    }

    return RetVal;
}

int nmea_gen_loop(nmeaGENERATOR *gen, nmeaINFO *info)
{
    int RetVal = 1;

    if(gen->loop_call)
        RetVal = (*gen->loop_call)(gen, info);

    if(RetVal && gen->next)
        RetVal = nmea_gen_loop(gen->next, info);

    return RetVal;
}

int nmea_gen_reset(nmeaGENERATOR *gen, nmeaINFO *info)
{
    int RetVal = 1;

    if(gen->reset_call)
        RetVal = (*gen->reset_call)(gen, info);

    return RetVal;
}

void nmea_gen_destroy(nmeaGENERATOR *gen)
{
    if(gen->next)
    {
        nmea_gen_destroy(gen->next);
        gen->next = 0;
    }

    if(gen->destroy_call)
        (*gen->destroy_call)(gen);

    free(gen);
}

void nmea_gen_add(nmeaGENERATOR *to, nmeaGENERATOR *gen)
{
    if(to->next)
        nmea_gen_add(to->next, gen);
    else
        to->next = gen;
}

int nmea_generate_from(
    char *buff, int buff_sz,
    nmeaINFO *info,
    nmeaGENERATOR *gen,
    int generate_mask
    )
{
    int retval;

    if(0 != (retval = nmea_gen_loop(gen, info)))
        retval = nmea_generate(buff, buff_sz, info, generate_mask);

    return retval;
}

/*
 * NOISE generator
 */

int nmea_igen_noise_init(nmeaGENERATOR *gen, nmeaINFO *info)
{
    return 1;
}

int nmea_igen_noise_loop(nmeaGENERATOR *gen, nmeaINFO *info)
{
    int it;
    int in_use;

    info->sig = (int)nmea_random(1, 3);
    info->PDOP = nmea_random(0, 9);
    info->HDOP = nmea_random(0, 9);
    info->VDOP = nmea_random(0, 9);
    info->fix = (int)nmea_random(2, 3);
    info->lat = nmea_random(0, 100);
    info->lon = nmea_random(0, 100);
    info->speed = nmea_random(0, 100);
    info->direction = nmea_random(0, 360);
    info->declination = nmea_random(0, 360);
    info->elv = (int)nmea_random(-100, 100);

    info->satinfo.inuse = 0;
    info->satinfo.inview = 0;

    for(it = 0; it < 12; ++it)
    {
        info->satinfo.sat[it].id = it;
        info->satinfo.sat[it].in_use = in_use = (int)nmea_random(0, 3);
        info->satinfo.sat[it].elv = (int)nmea_random(0, 90);
        info->satinfo.sat[it].azimuth = (int)nmea_random(0, 359);
        info->satinfo.sat[it].sig = (int)(in_use?nmea_random(40, 99):nmea_random(0, 40));

        if(in_use)
            info->satinfo.inuse++;
        if(info->satinfo.sat[it].sig > 0)
            info->satinfo.inview++;
    }

    return 1;
}

int nmea_igen_noise_reset(nmeaGENERATOR *gen, nmeaINFO *info)
{
    return 1;
}

/*
 * STATIC generator
 */

int nmea_igen_static_loop(nmeaGENERATOR *gen, nmeaINFO *info)
{
    nmea_time_now(&info->utc);
    return 1;
};

int nmea_igen_static_reset(nmeaGENERATOR *gen, nmeaINFO *info)
{
    info->satinfo.inuse = 4;
    info->satinfo.inview = 4;

    info->satinfo.sat[0].id = 1;
    info->satinfo.sat[0].in_use = 1;
    info->satinfo.sat[0].elv = 50;
    info->satinfo.sat[0].azimuth = 0;
    info->satinfo.sat[0].sig = 99;

    info->satinfo.sat[1].id = 2;
    info->satinfo.sat[1].in_use = 1;
    info->satinfo.sat[1].elv = 50;
    info->satinfo.sat[1].azimuth = 90;
    info->satinfo.sat[1].sig = 99;

    info->satinfo.sat[2].id = 3;
    info->satinfo.sat[2].in_use = 1;
    info->satinfo.sat[2].elv = 50;
    info->satinfo.sat[2].azimuth = 180;
    info->satinfo.sat[2].sig = 99;

    info->satinfo.sat[3].id = 4;
    info->satinfo.sat[3].in_use = 1;
    info->satinfo.sat[3].elv = 50;
    info->satinfo.sat[3].azimuth = 270;
    info->satinfo.sat[3].sig = 99;

    return 1;
}

int nmea_igen_static_init(nmeaGENERATOR *gen, nmeaINFO *info)
{
    info->sig = 3;
    info->fix = 3;

    nmea_igen_static_reset(gen, info);

    return 1;
}

/*
 * SAT_ROTATE generator
 */

int nmea_igen_rotate_loop(nmeaGENERATOR *gen, nmeaINFO *info)
{
    int it;
    int count = info->satinfo.inview;
    float deg = 360 / (count?count:1);
    float srt = (count?(info->satinfo.sat[0].azimuth):0) + 5;

    nmea_time_now(&info->utc);

    for(it = 0; it < count; ++it)
    {
        info->satinfo.sat[it].azimuth =
            (int)((srt >= 360)?srt - 360:srt);
        srt += deg;
    }

    return 1;
};

int nmea_igen_rotate_reset(nmeaGENERATOR *gen, nmeaINFO *info)
{
    int it;
    float deg = 360 / 8;
    float srt = 0;

    info->satinfo.inuse = 8;
    info->satinfo.inview = 8;

    for(it = 0; it < info->satinfo.inview; ++it)
    {
        info->satinfo.sat[it].id = it + 1;
        info->satinfo.sat[it].in_use = 1;
        info->satinfo.sat[it].elv = 5;
        info->satinfo.sat[it].azimuth = (int)srt;
        info->satinfo.sat[it].sig = 80;
        srt += deg;
    }

    return 1;
}

int nmea_igen_rotate_init(nmeaGENERATOR *gen, nmeaINFO *info)
{
    info->sig = 3;
    info->fix = 3;

    nmea_igen_rotate_reset(gen, info);

    return 1;
}

/*
 * POS_RANDMOVE generator
 */

int nmea_igen_pos_rmove_init(nmeaGENERATOR *gen, nmeaINFO *info)
{    
    info->sig = 3;
    info->fix = 3;
    info->direction = info->declination = 0;
    info->speed = 20;
    return 1;
}

int nmea_igen_pos_rmove_loop(nmeaGENERATOR *gen, nmeaINFO *info)
{
    nmeaPOS crd;

    info->direction += nmea_random(-10, 10);
    info->speed += nmea_random(-2, 3);

    if(info->direction < 0)
        info->direction = 359 + info->direction;
    if(info->direction > 359)
        info->direction -= 359;

    if(info->speed > 40)
        info->speed = 40;
    if(info->speed < 1)
        info->speed = 1;

    nmea_info2pos(info, &crd);
    nmea_move_horz(&crd, &crd, info->direction, info->speed / 3600);
    nmea_pos2info(&crd, info);

    info->declination = info->direction;

    return 1;
};

int nmea_igen_pos_rmove_destroy(nmeaGENERATOR *gen)
{
    return 1;
};

/*
 * generator create
 */

nmeaGENERATOR * __nmea_create_generator(int type, nmeaINFO *info)
{
    nmeaGENERATOR *gen = 0;

    switch(type)
    {
    case NMEA_GEN_NOISE:
        if(0 == (gen = malloc(sizeof(nmeaGENERATOR))))
            nmea_error("Insufficient memory!");
        else
        {
            memset(gen, 0, sizeof(nmeaGENERATOR));
            gen->init_call = &nmea_igen_noise_init;
            gen->loop_call = &nmea_igen_noise_loop;
            gen->reset_call = &nmea_igen_noise_reset;
        }
        break;
    case NMEA_GEN_STATIC:
    case NMEA_GEN_SAT_STATIC:
        if(0 == (gen = malloc(sizeof(nmeaGENERATOR))))
            nmea_error("Insufficient memory!");
        else
        {
            memset(gen, 0, sizeof(nmeaGENERATOR));
            gen->init_call = &nmea_igen_static_init;
            gen->loop_call = &nmea_igen_static_loop;
            gen->reset_call = &nmea_igen_static_reset;
        }
        break;
    case NMEA_GEN_SAT_ROTATE:
        if(0 == (gen = malloc(sizeof(nmeaGENERATOR))))
            nmea_error("Insufficient memory!");
        else
        {
            memset(gen, 0, sizeof(nmeaGENERATOR));
            gen->init_call = &nmea_igen_rotate_init;
            gen->loop_call = &nmea_igen_rotate_loop;
            gen->reset_call = &nmea_igen_rotate_reset;
        }
        break;
    case NMEA_GEN_POS_RANDMOVE:
        if(0 == (gen = malloc(sizeof(nmeaGENERATOR))))
            nmea_error("Insufficient memory!");
        else
        {
            memset(gen, 0, sizeof(nmeaGENERATOR));
            gen->init_call = &nmea_igen_pos_rmove_init;
            gen->loop_call = &nmea_igen_pos_rmove_loop;
            gen->destroy_call = &nmea_igen_pos_rmove_destroy;
        }
        break;
    case NMEA_GEN_ROTATE:
        gen = __nmea_create_generator(NMEA_GEN_SAT_ROTATE, info);
        nmea_gen_add(gen, __nmea_create_generator(NMEA_GEN_POS_RANDMOVE, info));
        break;
    };

    return gen;
}

nmeaGENERATOR * nmea_create_generator(int type, nmeaINFO *info)
{
    nmeaGENERATOR *gen = __nmea_create_generator(type, info);

    if(gen)
        nmea_gen_init(gen, info);

    return gen;
}

void nmea_destroy_generator(nmeaGENERATOR *gen)
{
    nmea_gen_destroy(gen);
}

#if defined(NMEA_WIN) && defined(_MSC_VER)
# pragma warning(default: 4100)
#endif
