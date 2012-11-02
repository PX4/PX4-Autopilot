/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright *    notice, this list of conditions and the following disclaimer in
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdlib.h>
#include <nuttx/power/pm.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <assert.h>
#include <setjmp.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*
 * PPM decoder tuning parameters
 */
# define PPM_MAX_PULSE_WIDTH	500		/* maximum width of a pulse */
# define PPM_MIN_CHANNEL_VALUE	800		/* shortest valid channel signal */
# define PPM_MAX_CHANNEL_VALUE	2200		/* longest valid channel signal */
# define PPM_MIN_START		2500		/* shortest valid start gap */

/****************************************************************************
* Public Data
****************************************************************************/

/* decoded PPM buffer */
#define PPM_MAX_CHANNELS	12
uint16_t ppm_buffer[PPM_MAX_CHANNELS];
unsigned ppm_decoded_channels;
uint64_t ppm_last_valid_decode = 0;

/****************************************************************************
 * Extern functions
 ****************************************************************************/
extern void up_serial(void);
extern void down_serial(void);
extern void cpuload_initialize_once(void);


/****************************************************************************
 * Public functions
 ****************************************************************************/

void up_systemreset(void);

/* irq */
void up_disable_irq(void);
void up_enable_irq(void);

/* spi */
void up_spiinitialize(void);

/* ppm */
void ppm_input_init(unsigned count_max);
void ppm_input_decode(unsigned reset, unsigned count);
int nsh_archinitialize(void);

/* power management */
void pm_initialize(void);
enum pm_state_e pm_checkstate(void);
int pm_changestate(enum pm_state_e newstate);

/****************************************************************************
 * Private functions
 ****************************************************************************/
static void uninit_arch(void);

/************************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Reset the system.
 *
 ************************************************************************************/
void
up_systemreset(void)
{
    uninit_arch();
}

/************************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ************************************************************************************/

FAR struct i2c_dev_s *up_i2cinitialize(int port)
{
  struct i2c_dev_s * inst = NULL;
  if (!(inst = kmalloc( sizeof(struct i2c_dev_s))))
  {
    return NULL;
  }

  struct i2c_ops_s * ops = NULL;
  if (!(ops = kmalloc( sizeof(struct i2c_ops_s))))
  {
    return NULL;
  }

  /* Initialize instance */
  inst->ops       = &ops;
  return inst;
}

/************************************************************************************
 * Name: up_i2cuninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ************************************************************************************/

int up_i2cuninitialize(FAR struct i2c_dev_s * dev)
{
  ASSERT(dev);
  kfree(dev);
  return OK;
}

/************************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ************************************************************************************/
int up_i2creset(FAR struct i2c_dev_s * dev)
{
  ASSERT(dev);
  return OK;
}


void
up_disable_irq(void)
{
}

void
up_enable_irq(void)
{
}

void
up_spiinitialize(void)
{
}

void
ppm_input_init(unsigned count_max) 
{
}

void
ppm_input_decode(unsigned reset, unsigned count)
{
}

int
nsh_archinitialize(void)
{
    vdbg("initializing sim arch\n");
    up_serial();
    atexit(uninit_arch);
    return 0;
}

void
pm_initialize(void)
{
    cpuload_initialize_once();
}

enum pm_state_e
pm_checkstate(void)
{
    return 0;
}

int
pm_changestate(enum pm_state_e newstate)
{
    return 0;
}

void
uninit_arch(void)
{
    vdbg("uninit");
    down_serial();
    ASSERT(0);
}
