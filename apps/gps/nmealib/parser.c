/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: parser.c 17 2008-03-11 11:56:11Z xtimor $
 *
 */

/**
 * \file parser.h
 */
#include "nmea/tok.h"
#include "nmea/parse.h"
#include "nmea/parser.h"
#include "nmea/context.h"

#include <string.h>
#include <stdlib.h>

typedef struct _nmeaParserNODE
{
    int packType;
    void *pack;
    struct _nmeaParserNODE *next_node;

} nmeaParserNODE;

/*
 * high level
 */

/**
 * \brief Initialization of parser object
 * @return true (1) - success or false (0) - fail
 */
int nmea_parser_init(nmeaPARSER *parser)
{
    int resv = 0;
    int buff_size = nmea_property()->parse_buff_size;

    //NMEA_ASSERT(parser);

    if(buff_size < NMEA_MIN_PARSEBUFF)
        buff_size = NMEA_MIN_PARSEBUFF;

    memset(parser, 0, sizeof(nmeaPARSER));

    if(0 == (parser->buffer = malloc(buff_size)))
        nmea_error("Insufficient memory!");
    else
    {
        parser->buff_size = buff_size;
        resv = 1;
    }    

    return resv;
}

/**
 * \brief Destroy parser object
 */
void nmea_parser_destroy(nmeaPARSER *parser)
{
    //NMEA_ASSERT(parser && parser->buffer);
    free(parser->buffer);
    nmea_parser_queue_clear(parser);
    memset(parser, 0, sizeof(nmeaPARSER));
}

/**
 * \brief Analysis of buffer and put results to information structure
 * @return Number of packets wos parsed
 */
int nmea_parse(    
    nmeaPARSER *parser,
    const char *buff, int buff_sz,
    nmeaINFO *info
    )
{
    int ptype, nread = 0;
    void *pack = 0;

    //NMEA_ASSERT(parser && parser->buffer);

    nmea_parser_push(parser, buff, buff_sz);

    while(GPNON != (ptype = nmea_parser_pop(parser, &pack)))
    {
        nread++;

        switch(ptype)
        {
        case GPGGA:
            nmea_GPGGA2info((nmeaGPGGA *)pack, info);
            break;
        case GPGSA:
            nmea_GPGSA2info((nmeaGPGSA *)pack, info);
            break;
        case GPGSV:
            nmea_GPGSV2info((nmeaGPGSV *)pack, info);
            break;
        case GPRMC:
            nmea_GPRMC2info((nmeaGPRMC *)pack, info);
            break;
        case GPVTG:
            nmea_GPVTG2info((nmeaGPVTG *)pack, info);
            break;
        };

        free(pack);
    }

    return nread;
}

/*
 * low level
 */

int nmea_parser_real_push(nmeaPARSER *parser, const char *buff, int buff_sz)
{
    int nparsed = 0, crc, sen_sz, ptype;
    nmeaParserNODE *node = 0;

    //NMEA_ASSERT(parser && parser->buffer);

    /* clear unuse buffer (for debug) */
    /*
    memset(
        parser->buffer + parser->buff_use, 0,
        parser->buff_size - parser->buff_use
        );
        */

    /* add */
    if(parser->buff_use + buff_sz >= parser->buff_size)
        nmea_parser_buff_clear(parser);

    memcpy(parser->buffer + parser->buff_use, buff, buff_sz);
    parser->buff_use += buff_sz;

    /* parse */
    for(;;node = 0)
    {
        sen_sz = nmea_find_tail(
            (const char *)parser->buffer + nparsed,
            (int)parser->buff_use - nparsed, &crc);

        if(!sen_sz)
        {
            if(nparsed)
                memcpy(
                parser->buffer,
                parser->buffer + nparsed,
                parser->buff_use -= nparsed);
            break;
        }
        else if(crc >= 0)
        {
            ptype = nmea_pack_type(
                (const char *)parser->buffer + nparsed + 1,
                parser->buff_use - nparsed - 1);

            if(0 == (node = malloc(sizeof(nmeaParserNODE))))
                goto mem_fail;

            node->pack = 0;

            switch(ptype)
            {
            case GPGGA:
                if(0 == (node->pack = malloc(sizeof(nmeaGPGGA))))
                    goto mem_fail;
                node->packType = GPGGA;
                if(!nmea_parse_GPGGA(
                    (const char *)parser->buffer + nparsed,
                    sen_sz, (nmeaGPGGA *)node->pack))
                {
                    free(node);
                    node = 0;
                }
                break;
            case GPGSA:
                if(0 == (node->pack = malloc(sizeof(nmeaGPGSA))))
                    goto mem_fail;
                node->packType = GPGSA;
                if(!nmea_parse_GPGSA(
                    (const char *)parser->buffer + nparsed,
                    sen_sz, (nmeaGPGSA *)node->pack))
                {
                    free(node);
                    node = 0;
                }
                break;
            case GPGSV:
                if(0 == (node->pack = malloc(sizeof(nmeaGPGSV))))
                    goto mem_fail;
                node->packType = GPGSV;
                if(!nmea_parse_GPGSV(
                    (const char *)parser->buffer + nparsed,
                    sen_sz, (nmeaGPGSV *)node->pack))
                {
                    free(node);
                    node = 0;
                }
                break;
            case GPRMC:
                if(0 == (node->pack = malloc(sizeof(nmeaGPRMC))))
                    goto mem_fail;
                node->packType = GPRMC;
                if(!nmea_parse_GPRMC(
                    (const char *)parser->buffer + nparsed,
                    sen_sz, (nmeaGPRMC *)node->pack))
                {
                    free(node);
                    node = 0;
                }
                break;
            case GPVTG:
                if(0 == (node->pack = malloc(sizeof(nmeaGPVTG))))
                    goto mem_fail;
                node->packType = GPVTG;
                if(!nmea_parse_GPVTG(
                    (const char *)parser->buffer + nparsed,
                    sen_sz, (nmeaGPVTG *)node->pack))
                {
                    free(node);
                    node = 0;
                }
                break;
            default:
                free(node);
                node = 0;
                break;
            };

            if(node)
            {
                if(parser->end_node)
                    ((nmeaParserNODE *)parser->end_node)->next_node = node;
                parser->end_node = node;
                if(!parser->top_node)
                    parser->top_node = node;
                node->next_node = 0;
            }
        }

        nparsed += sen_sz;
    }

    return nparsed;

mem_fail:
    if(node)
        free(node);

    nmea_error("Insufficient memory!");

    return -1;
}

/**
 * \brief Analysis of buffer and keep results into parser
 * @return Number of bytes wos parsed from buffer
 */
int nmea_parser_push(nmeaPARSER *parser, const char *buff, int buff_sz)
{
    int nparse, nparsed = 0;

    do
    {
        if(buff_sz > parser->buff_size)
            nparse = parser->buff_size;
        else
            nparse = buff_sz;

        nparsed += nmea_parser_real_push(
            parser, buff, nparse);

        buff_sz -= nparse;

    } while(buff_sz);

    return nparsed;
}

/**
 * \brief Get type of top packet keeped into parser
 * @return Type of packet
 * @see nmeaPACKTYPE
 */
int nmea_parser_top(nmeaPARSER *parser)
{
    int retval = GPNON;
    nmeaParserNODE *node = (nmeaParserNODE *)parser->top_node;

    //NMEA_ASSERT(parser && parser->buffer);

    if(node)
        retval = node->packType;

    return retval;
}

/**
 * \brief Withdraw top packet from parser
 * @return Received packet type
 * @see nmeaPACKTYPE
 */
int nmea_parser_pop(nmeaPARSER *parser, void **pack_ptr)
{
    int retval = GPNON;
    nmeaParserNODE *node = (nmeaParserNODE *)parser->top_node;

    //NMEA_ASSERT(parser && parser->buffer);

    if(node)
    {
        *pack_ptr = node->pack;
        retval = node->packType;
        parser->top_node = node->next_node;
        if(!parser->top_node)
            parser->end_node = 0;
        free(node);
    }

    return retval;
}

/**
 * \brief Get top packet from parser without withdraw
 * @return Received packet type
 * @see nmeaPACKTYPE
 */
int nmea_parser_peek(nmeaPARSER *parser, void **pack_ptr)
{
    int retval = GPNON;
    nmeaParserNODE *node = (nmeaParserNODE *)parser->top_node;

    //NMEA_ASSERT(parser && parser->buffer);

    if(node)
    {
        *pack_ptr = node->pack;
        retval = node->packType;
    }

    return retval;
}

/**
 * \brief Delete top packet from parser
 * @return Deleted packet type
 * @see nmeaPACKTYPE
 */
int nmea_parser_drop(nmeaPARSER *parser)
{
    int retval = GPNON;
    nmeaParserNODE *node = (nmeaParserNODE *)parser->top_node;

    //NMEA_ASSERT(parser && parser->buffer);

    if(node)
    {
        if(node->pack)
            free(node->pack);
        retval = node->packType;
        parser->top_node = node->next_node;
        if(!parser->top_node)
            parser->end_node = 0;
        free(node);
    }

    return retval;
}

/**
 * \brief Clear cache of parser
 * @return true (1) - success
 */
int nmea_parser_buff_clear(nmeaPARSER *parser)
{
    //NMEA_ASSERT(parser && parser->buffer);
    parser->buff_use = 0;
    return 1;
}

/**
 * \brief Clear packets queue into parser
 * @return true (1) - success
 */
int nmea_parser_queue_clear(nmeaPARSER *parser)
{
    //NMEA_ASSERT(parser);
    while(parser->top_node)
        nmea_parser_drop(parser);
    return 1;
}
