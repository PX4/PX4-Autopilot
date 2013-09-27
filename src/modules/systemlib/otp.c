/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Authors: 
 *      Lorenz Meier <lm@inf.ethz.ch>
 *      David "Buzz" Bussenschutt <davidbuzz@gmail.com> 
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
 * @file otp.c
 * otp estimation
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author David "Buzz" Bussenschutt <davidbuzz@gmail.com>
 *
 */

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>  // memset
#include "conversions.h"
#include "otp.h"
#include "err.h"   // warnx 
#include <assert.h>


int val_read(void* dest, volatile const void* src, int bytes)
{
	
	int i;
	for (i = 0; i < bytes / 4; i++) {
		*(((volatile uint32_t *)dest) + i) = *(((volatile uint32_t *)src) + i);
	}
	return i*4;
}


int write_otp(uint8_t id_type, uint32_t vid, uint32_t pid, char* signature)
{
	
	warnx("write_otp: PX4 / %02X / %02X / %02X  / ... etc  \n",id_type, vid, pid); 

        // descriptor
        F_write_byte( ADDR_OTP_START, 'P'); //  write the 'P' from PX4. to first byte in OTP 
        F_write_byte( ADDR_OTP_START+1, 'X'); //  write the 'P' from PX4. to first byte in OTP 
        F_write_byte( ADDR_OTP_START+2, '4'); 
        F_write_byte( ADDR_OTP_START+3, '\0'); 
        //id_type
        F_write_byte( ADDR_OTP_START+4, id_type); 
        // vid and pid are 4 bytes each
        F_write_word( ADDR_OTP_START+5, vid); 
        F_write_word( ADDR_OTP_START+9, pid); 
        // leave some 19 bytes of space, and go to the next block...
        // then the auth sig starts 
        for ( int i = 0 ; i < 128 ; i++ ) {
            F_write_byte( ADDR_OTP_START+32+i, signature[i]); 
        }

        
}

int lock_otp(void)
{
	//determine the required locking size - can only write full lock bytes */
//	int size = sizeof(struct otp) / 32;
//	
//	struct otp_lock otp_lock_mem;
//
//	memset(&otp_lock_mem, OTP_LOCK_UNLOCKED, sizeof(otp_lock_mem));
//	for (int i = 0; i < sizeof(otp_lock_mem) / sizeof(otp_lock_mem.lock_bytes[0]); i++)
//		otp_lock_mem.lock_bytes[i] = OTP_LOCK_LOCKED;
	//XXX add the actual call here to write the OTP_LOCK bytes only at final stage 
	// val_copy(lock_ptr, &otp_lock_mem, sizeof(otp_lock_mem));
			
    int locksize = 5;
	// or just realise it's exctly 5x 32byte blocks we need to lock.  1 block for ID,type,vid,pid, and 4 blocks for certificate, which is 128 bytes. 
        for ( int i = 0 ; i < locksize ; i++ ) {
            F_write_byte( ADDR_OTP_LOCK_START+i, OTP_LOCK_LOCKED); 
        }

}



// COMPLETE, BUSY, or other flash error?  
uint8_t F_GetStatus(void) { 
    uint8_t fs = F_COMPLETE; 
    if((FLASH->status & F_BSY) == F_BSY) { fs = F_BUSY; } else { 
        if((FLASH->status & F_WRPERR) != (uint32_t)0x00) { fs = F_ERROR_WRP; } else { 
            if((FLASH->status & (uint32_t)0xEF) != (uint32_t)0x00) { fs = F_ERROR_PROGRAM; } else { 
                if((FLASH->status & F_OPERR) != (uint32_t)0x00) { fs = F_ERROR_OPERATION; } else { 
                    fs = F_COMPLETE; } } } }  
                    return fs; 
} 


// enable FLASH Registers 
void F_unlock(void)
{
  if((FLASH->control & F_CR_LOCK) != 0)
  {
    FLASH->key = F_KEY1;
    FLASH->key = F_KEY2;
  }  
}

//  lock the FLASH Registers 
void F_lock(void)
{
  FLASH->control |= F_CR_LOCK;
}

// flash write word. 
uint8_t F_write_word(uint32_t Address, uint32_t Data)
{
    unsigned char octet[4] = {0,0,0,0};  
    for (int i=0; i<4; i++)
    {
       octet[i] = ( Data >> (i*8) ) & 0xFF;
       F_write_byte(Address+i,octet[i]);
    }
     
 }

// flash write byte
uint8_t F_write_byte(uint32_t Address, uint8_t Data)
{
  volatile uint8_t status = F_COMPLETE;
  
  //warnx("F_write_byte: %08X %02d", Address , Data ) ;

  //Check the parameters 
  assert(IS_F_ADDRESS(Address));

       //Wait for FLASH operation to complete by polling on BUSY flag.
      status = F_GetStatus();
      while(status == F_BUSY){ status = F_GetStatus();}
  
  if(status == F_COMPLETE)
  {
    //if the previous operation is completed, proceed to program the new data 
    FLASH->control &= CR_PSIZE_MASK;
    FLASH->control |= F_PSIZE_BYTE;
    FLASH->control |= F_CR_PG;
  
    *(volatile uint8_t*)Address = Data;

       //Wait for FLASH operation to complete by polling on BUSY flag.
      status = F_GetStatus();
      while(status == F_BUSY){ status = F_GetStatus();}

    //if the program operation is completed, disable the PG Bit 
    FLASH->control &= (~F_CR_PG);
  } 

  //Return the Program Status 
  return status;
}



 