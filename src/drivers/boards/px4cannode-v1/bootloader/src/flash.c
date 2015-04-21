#include <nuttx/config.h>
#include "app_config.h"

#include <stdint.h>
#include <stdlib.h>

#include <nuttx/progmem.h>

#include "chip.h"
#include "stm32.h"

#include "flash.h"


/****************************************************************************
 * Name: bl_flash_erase
 *
 * Description:
 *   This function erases the flash starting at the given address
 *
 * Input Parameters:
 *   address - The address of the flash to erase
 *
 * Returned value:
 *   On sucess FLASH_OK On Error one of the flash_error_t
 *
 ****************************************************************************/

flash_error_t bl_flash_erase(size_t address)
{
  /* 
   * FIXME (?): this may take a long time, and while flash is being erased it
   * might not be possible to execute interrupts, send NodeStatus messages etc.
   * We can pass a per page callback or yeild */

  flash_error_t status = FLASH_ERROR_AFU;

  ssize_t bllastpage = up_progmem_getpage(address - 1);

  if (bllastpage >= 0)
    {

      status = FLASH_ERROR_SUICIDE;
      ssize_t appfirstpage = up_progmem_getpage(address);

      if (appfirstpage > bllastpage)
        {

          size_t pagecnt = up_progmem_npages() - (bllastpage + 1);

          /* Erase the whole application flash region */
          status = FLASH_OK;

          while (status == FLASH_OK && pagecnt--)
            {

              ssize_t ps = up_progmem_erasepage(appfirstpage);
              if (ps <= 0)
                {
                  status = FLASH_ERASE_ERROR;
                }
            }
        }
    }
  return status;
}

/****************************************************************************
 * Name: bl_flash_write_word
 *
 * Description:
 *   This function erases the flash starting at the given address
 *
 * Input Parameters:
 *   flash_address - The address of the flash to write
 *   data          - A pointer to a buffer of 4 bytes to be written
 *                   to the flash.
 *
 * Returned value:
 *   On sucess FLASH_OK On Error one of the flash_error_t
 *
 ****************************************************************************/

flash_error_t bl_flash_write_word(uint32_t flash_address, const uint8_t data[4])
{

  flash_error_t status = FLASH_ERROR;
  if (flash_address >= APPLICATION_LOAD_ADDRESS &&
      (flash_address + sizeof(data)) <= (uint32_t) APPLICATION_LAST_32BIT_ADDRRESS)
    {
      if (sizeof(data) ==
          up_progmem_write((size_t) flash_address, (void *)data, sizeof(data)))
        {
          status = FLASH_OK;
        }
    }
  return status;
}
