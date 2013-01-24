/************************************************************************************
 * arch/arm/src/stm32/stm32_flash.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 ************************************************************************************/

/* Provides standard flash access functions, to be used by the  flash mtd driver.
 * The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <errno.h>

#include "stm32_flash.h"
#include "stm32_rcc.h"
#include "stm32_waste.h"

#include "up_arch.h"

/* Only for the STM32F10xx family for now */

#ifdef CONFIG_STM32_STM32F10XX

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

/************************************************************************************
 * Private Functions
 ************************************************************************************/

void stm32_flash_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_KEYR);
    }
}

void stm32_flash_lock(void)
{
  modifyreg16(STM32_FLASH_CR, 0, FLASH_CR_LOCK);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

uint16_t up_progmem_npages(void)
{
  return STM32_FLASH_NPAGES;
}

bool up_progmem_isuniform(void)
{
  return true;
}

uint16_t up_progmem_pagesize(uint16_t page)
{
  return STM32_FLASH_PAGESIZE;
}

int up_progmem_getpage(uint32_t addr)
{
  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if (addr >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  return addr / STM32_FLASH_PAGESIZE;
}

int up_progmem_erasepage(uint16_t page)
{
  uint32_t addr;
  uint16_t count;

  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Get flash ready and begin erasing single page */

  if (!(getreg32(STM32_RCC_CR) & RCC_CR_HSION))
    {
      return -EPERM;
    }

  stm32_flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PER);
  putreg32(page * STM32_FLASH_PAGESIZE, STM32_FLASH_AR);
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);

  while(getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PER, 0);

  /* Verify */

  for (addr = page * STM32_FLASH_PAGESIZE + STM32_FLASH_BASE, count = STM32_FLASH_PAGESIZE;
       count; count-=4, addr += 4)
    {
      if (getreg32(addr) != 0xffffffff)
        {
          return -EIO;
        }
    }

  return STM32_FLASH_PAGESIZE;
}

int up_progmem_ispageerased(uint16_t page)
{
  uint32_t addr;
  uint16_t count;
  uint16_t bwritten = 0;

  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = page * STM32_FLASH_PAGESIZE + STM32_FLASH_BASE, count = STM32_FLASH_PAGESIZE;
       count; count--, addr++)
    {
      if (getreg8(addr) != 0xff)
        {
          bwritten++;
        }
    }

  return bwritten;
}

int up_progmem_write(uint32_t addr, const void *buf, size_t count)
{
  uint16_t *hword = (uint16_t *)buf;
  size_t written = count;

  /* STM32 requires half-word access */

  if (count & 1)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if ((addr+count) >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  /* Get flash ready and begin flashing */

  if (!(getreg32(STM32_RCC_CR) & RCC_CR_HSION))
    {
      return -EPERM;
    }

  stm32_flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

  for (addr += STM32_FLASH_BASE; count; count-=2, hword++, addr+=2)
    {
      /* Write half-word and wait to complete */

      putreg16(*hword, addr);

      while(getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

      /* Verify */

      if (getreg32(STM32_FLASH_SR) & FLASH_SR_WRPRT_ERR)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          return -EROFS;
        }

      if (getreg16(addr) != *hword)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          return -EIO;
        }
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
  return written;
}

#endif /* CONFIG_STM32_STM32F10XX */
