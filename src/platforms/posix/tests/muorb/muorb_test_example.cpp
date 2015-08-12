/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hello_example.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "muorb_test_example.h"
#include <px4_log.h>
#include <unistd.h>
#include <stdio.h>
#include <px4_defines.h>

px4::AppState MuorbTestExample::appState;

int MuorbTestExample::main()
{
	int rc;
	appState.setRunning(true);
	rc =  PingPongTest();
	appState.setRunning(false);
	return rc;
}

int  MuorbTestExample::DefaultTest()
{
        int i=0;
        orb_advert_t pub_id = orb_advertise( ORB_ID( esc_status ), & m_esc_status );
        if( pub_id == 0 )
        {
            PX4_ERR( "error publishing esc_status" );
            return -1;
        }
        orb_advert_t pub_id_vc = orb_advertise( ORB_ID( vehicle_command ), & m_vc );
        if( pub_id_vc == 0 )
        {
            PX4_ERR( "error publishing vehicle_command" );
            return -1;
        }
        if( orb_publish( ORB_ID( vehicle_command ), pub_id_vc, &m_vc ) == PX4_ERROR )
        {
           PX4_ERR( "[%d]Error publishing the vechile command message", i );
           return -1;
        }
        int sub_vc = orb_subscribe( ORB_ID( vehicle_command ) );
        if ( sub_vc == PX4_ERROR )
        {
           PX4_ERR( "Error subscribing to vehicle_command topic" );
           return -1;
        }

        while (!appState.exitRequested() && i<100) {

                PX4_DEBUG("[%d]  Doing work...", i );
                if( orb_publish( ORB_ID( esc_status ), pub_id, &m_esc_status ) == PX4_ERROR )
                {
                   PX4_ERR( "[%d]Error publishing the esc status message for iter", i );
                   break;
                }
                bool updated = false;
                if( orb_check( sub_vc, &updated ) == 0 )
                {
                    if( updated )
                    {
                       PX4_DEBUG( "[%d]Vechicle Status is updated... reading new value", i );
                       if( orb_copy( ORB_ID( vehicle_command ), sub_vc, &m_vc ) != 0 )
                       {
                           PX4_ERR( "[%d]Error calling orb copy for vechivle status... ", i );
                           break;
                       }
                       if( orb_publish( ORB_ID( vehicle_command ), pub_id_vc, &m_vc ) == PX4_ERROR )
                       {
                          PX4_ERR( "[%d]Error publishing the vechile command message", i );
                          break;
                       }
                    }
                    else
                    {
                       PX4_DEBUG( "[%d] VC topic is not updated", i );
                    }
                }
                else
                {
                    PX4_ERR( "[%d]Error checking the updated status for vechile command... ", i );
                    break;
                } 
                
         ++i;
        }
        return 0;
}

int MuorbTestExample::PingPongTest()
{
        int i=0;
        orb_advert_t pub_id_vc = orb_advertise( ORB_ID( vehicle_command ), & m_vc );
        if( pub_id_vc == 0 )
        {
            PX4_ERR( "error publishing vehicle_command" );
            return -1;
        }
        if( orb_publish( ORB_ID( vehicle_command ), pub_id_vc, &m_vc ) == PX4_ERROR )
        {
           PX4_ERR( "[%d]Error publishing the vechile command message", i );
           return -1;
        }
        int sub_esc_status = orb_subscribe( ORB_ID( esc_status ) );
        if ( sub_esc_status == PX4_ERROR )
        {
           PX4_ERR( "Error subscribing to esc_status topic" );
           return -1;
        }

        while (!appState.exitRequested() ) {

                PX4_INFO("[%d]  Doing work...", i );
                bool updated = false;
                if( orb_check( sub_esc_status, &updated ) == 0 )
                {
                    if( updated )
                    {
                       PX4_INFO( "[%d]ESC status is updated... reading new value", i );
                       if( orb_copy( ORB_ID( esc_status ), sub_esc_status, &m_esc_status ) != 0 )
                       {
                           PX4_ERR( "[%d]Error calling orb copy for esc status... ", i );
                           break;
                       }
                       if( orb_publish( ORB_ID( vehicle_command ), pub_id_vc, &m_vc ) == PX4_ERROR )
                       {
                          PX4_ERR( "[%d]Error publishing the vechile command message", i );
                          break;
                       }
                    }
                    else
                    {
                       PX4_INFO( "[%d] esc status topic is not updated", i );
                    }
                }
                else
                {
                    PX4_ERR( "[%d]Error checking the updated status for esc status... ", i );
                    break;
                }
                // sleep for 1 sec.
                usleep( 1000000 ); 

                ++i;
        }
        return 0;
}
