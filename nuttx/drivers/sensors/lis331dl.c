/****************************************************************************
 * drivers/sensors/lis331dl.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/** \file
 *  \author Uros Platise
 *  \brief ST LIS331DL I2C Device Driver
 **/ 
 
#include <nuttx/config.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/lis331dl.h>

/************************************************************************************
 * LIS331DL Internal Registers
 ************************************************************************************/

#define ST_LIS331DL_WHOAMI          0x0F    /* who am I register */
#define ST_LIS331DL_WHOAMI_VALUE    0x3B    /* Valid result is 0x3B */

#define ST_LIS331DL_CTRL_REG1       0x20
#define ST_LIS331DL_CR1_DR          0x80    /* Data-rate selection 0: 100 Hz, 1: 400 Hz */
#define ST_LIS331DL_CR1_PD          0x40    /* Active Mode (1) / Power-down (0) */
#define ST_LIS331DL_CR1_FS          0x20    /* Full Scale (1) +-9g or Normal Scale (0) +-2g */
#define ST_LIS331DL_CR1_ST          0x18    /* Self test enable */
#define ST_LIS331DL_CR1_ZEN         0x04    /* Z-Axis Enable */
#define ST_LIS331DL_CR1_YEN         0x02    /* Y-Axis Enable */
#define ST_LIS331DL_CR1_XEN         0x01    /* X-Axis Enable */

#define ST_LIS331DL_CTRL_REG2       0x21
#define ST_LIS331DL_CTRL_REG3       0x22

#define ST_LIS331DL_HP_FILTER_RESET 0x23

#define ST_LIS331DL_STATUS_REG      0x27    /* Status Register */
#define ST_LIS331DL_SR_ZYXOR        0x80    /* OR'ed X,Y and Z data over-run  */
#define ST_LIS331DL_SR_ZOR          0x40    /* individual data over-run ... */
#define ST_LIS331DL_SR_YOR          0x20
#define ST_LIS331DL_SR_XOR          0x10
#define ST_LIS331DL_SR_ZYXDA        0x08    /* OR'ed X,Y and Z data available */
#define ST_LIS331DL_SR_ZDA          0x04    /* individual data available ... */
#define ST_LIS331DL_SR_YDA          0x02
#define ST_LIS331DL_SR_XDA          0x01

#define ST_LIS331DL_OUT_X           0x29
#define ST_LIS331DL_OUT_Y           0x2B
#define ST_LIS331DL_OUT_Z           0x2D

 
/************************************************************************************
 * Private Data Types
 ************************************************************************************/

struct lis331dl_dev_s {
    struct i2c_dev_s *          i2c;
    
    uint8_t                     address;
    struct lis331dl_vector_s a;
    uint8_t                     cr1;
    uint8_t                     cr2;    
    uint8_t                     cr3;
};

 
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/** LIS331DL Access with range check
 * 
 * \param dev LIS331 DL Private Structure
 * \param subaddr LIS331 Sub Address
 * \param buf Pointer to buffer, either for read or write access
 * \param length when >0 it denotes read access, when <0 it denotes write access of -length
 * \return OK on success or errno is set.
 **/
int lis331dl_access(struct lis331dl_dev_s * dev, uint8_t subaddr, uint8_t *buf, int length)
{
    uint16_t flags = 0;
    int      retval;
    
    if (length > 0) {
        flags = I2C_M_READ; 
    }
    else {
        flags = I2C_M_NORESTART;
        length = -length;
    }

    /* Check valid address ranges and set auto address increment flag */
    
    if (subaddr == 0x0F) {
        if (length > 1) length = 1;
    }
    else if (subaddr >= 0x20 && subaddr < 0x24) {
        if (length > (0x24 - subaddr) ) length = 0x24 - subaddr;
    }
    else if (subaddr >= 0x27 && subaddr < 0x2E) {
        if (length > (0x2E - subaddr) ) length = 0x2E - subaddr;
    }
    else if (subaddr >= 0x30 && subaddr < 0x40) {
        if (length > (0x40 - subaddr) ) length = 0x40 - subaddr;
    }
    else {
        errno = EFAULT;
        return ERROR;
    }

    if (length > 1) subaddr |= 0x80;
    
    /* Create message and send */
    
    struct i2c_msg_s msgv[2] = {
      {
        .addr   = dev->address,
        .flags  = 0,
        .buffer = &subaddr,
        .length = 1
      },
      {
        .addr   = dev->address,
        .flags  = flags,
        .buffer = buf,
        .length = length
      }
    };
    
    if ( (retval = I2C_TRANSFER(dev->i2c, msgv, 2)) == OK )
        return length;
        
    return retval;
}


int lis331dl_readregs(struct lis331dl_dev_s * dev)
{
    if (lis331dl_access(dev, ST_LIS331DL_CTRL_REG1, &dev->cr1, 3) != 3) return ERROR;
    return OK;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct lis331dl_dev_s * lis331dl_init(struct i2c_dev_s * i2c, uint16_t address)
{
    struct lis331dl_dev_s * dev;
    uint8_t retval;
    
    ASSERT(i2c);
    ASSERT(address);
    
    if ( (dev = kmalloc( sizeof(struct lis331dl_dev_s) )) == NULL ) {
        errno = ENOMEM;
        return NULL;
    }
        
    memset(dev, 0, sizeof(struct lis331dl_dev_s));
    dev->i2c     = i2c;
    dev->address = address;
    
    /* Probe device */
    
    if (lis331dl_access(dev, ST_LIS331DL_WHOAMI, &retval, 1) > 0) {
    
        /* Check chip identification, in the future several more compatible parts
         * may be added here.
         */
         
        if (retval == ST_LIS331DL_WHOAMI_VALUE) {
        
            /* Copy LIS331DL registers to our private structure and power-up device */
            
            if (lis331dl_readregs(dev)==OK && lis331dl_powerup(dev)==OK) {
      
                /* Normal exit point */
                errno = 0;
                return dev;
            }
            retval = errno;
        }
        
        /* Otherwise, we mark an invalid device found at given address */
        retval = ENODEV;
    }
    else {
        /* No response at given address is marked as */
        retval = EFAULT;
    }

    /* Error exit */
    kfree(dev);
    errno = retval;
    return NULL;
}


int lis331dl_deinit(struct lis331dl_dev_s * dev)
{
    ASSERT(dev);
    
    lis331dl_powerdown(dev);
    kfree(dev);
    
    return OK;
}


int lis331dl_powerup(struct lis331dl_dev_s * dev)
{
    dev->cr1 = ST_LIS331DL_CR1_PD |
        ST_LIS331DL_CR1_ZEN | ST_LIS331DL_CR1_YEN | ST_LIS331DL_CR1_XEN;
    dev->cr2 = 0;
    dev->cr3 = 0;
        
    if (lis331dl_access(dev, ST_LIS331DL_CTRL_REG1, &dev->cr1, -3) == 3) return OK;
    return ERROR;
}


int lis331dl_powerdown(struct lis331dl_dev_s * dev)
{
    dev->cr1 = ST_LIS331DL_CR1_ZEN | ST_LIS331DL_CR1_YEN | ST_LIS331DL_CR1_XEN;
    dev->cr2 = 0;
    dev->cr3 = 0;
        
    if (lis331dl_access(dev, ST_LIS331DL_CTRL_REG1, &dev->cr1, -3) == 3) return OK;
    return ERROR;
}


int lis331dl_setconversion(struct lis331dl_dev_s * dev, bool full, bool fast)
{
    dev->cr1 = ST_LIS331DL_CR1_PD | 
        (full ? ST_LIS331DL_CR1_FS : 0) | (fast ? ST_LIS331DL_CR1_DR : 0) |
        ST_LIS331DL_CR1_ZEN | ST_LIS331DL_CR1_YEN | ST_LIS331DL_CR1_XEN;
        
    if (lis331dl_access(dev, ST_LIS331DL_CTRL_REG1, &dev->cr1, -1) == 1) return OK;
    return ERROR;
}


int lis331dl_getprecision(struct lis331dl_dev_s * dev)
{
    if (dev->cr1 & ST_LIS331DL_CR1_FS)
        return 9200/127;   /* typ. 9.2g full scale */
    return 2300/127;       /* typ. 2.3g full scale */
}


int lis331dl_getsamplerate(struct lis331dl_dev_s * dev)
{   
    if (dev->cr1 & ST_LIS331DL_CR1_DR)
        return 400;
    return 100;
}


const struct lis331dl_vector_s * lis331dl_getreadings(struct lis331dl_dev_s * dev)
{
    uint8_t retval[7];

    ASSERT(dev);
    
    if (lis331dl_access(dev, ST_LIS331DL_STATUS_REG, retval, 7) == 7) {
    
        /* If result is not yet ready, return NULL */
        
        if ( !(retval[0] & ST_LIS331DL_SR_ZYXDA) ) {
            errno = EAGAIN;
            return NULL;
        }
            
        dev->a.x = retval[2];
        dev->a.y = retval[4];
        dev->a.z = retval[6];
        return &dev->a;
    }
    
    return NULL;
}
