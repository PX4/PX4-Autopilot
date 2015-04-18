#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>

#include "chip.h"
#include "stm32.h"

#include <errno.h>
#include "boot_app_shared.h"
#include "crc.h"

#define BOOTLOADER_COMMON_APP_SIGNATURE         0xB0A04150u
#define BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE  0xB0A0424Cu

/*  CAN_FiRx where (i=0..27|13, x=1, 2)
 *                      STM32_CAN1_FIR(i,x)
 * Using i = 2 does not requier there block
 * to be enabled nor FINIT in CAN_FMR to be set.
 * todo:Validate this claim on F2, F3
 */
#define crc_HiLOC       STM32_CAN1_FIR(2,1)
#define crc_LoLOC       STM32_CAN1_FIR(2,2)
#define signature_LOC   STM32_CAN1_FIR(3,1)
#define bus_speed_LOC   STM32_CAN1_FIR(3,2)
#define node_id_LOC     STM32_CAN1_FIR(4,1)
#define CRC_H 1
#define CRC_L 0

inline static void read(bootloader_app_shared_t * pcommon)
{
  pcommon->signature = getreg32(signature_LOC);
  pcommon->bus_speed = getreg32(bus_speed_LOC);
  pcommon->node_id = getreg32(node_id_LOC);
  pcommon->crc.ul[CRC_L] = getreg32(crc_LoLOC);
  pcommon->crc.ul[CRC_H] = getreg32(crc_HiLOC);

}

inline static void write(bootloader_app_shared_t * pcommon)
{
  putreg32(pcommon->signature, signature_LOC);
  putreg32(pcommon->bus_speed, bus_speed_LOC);
  putreg32(pcommon->node_id, node_id_LOC);
  putreg32(pcommon->crc.ul[CRC_L], crc_LoLOC);
  putreg32(pcommon->crc.ul[CRC_H], crc_HiLOC);

}

static uint64_t calulate_signature(bootloader_app_shared_t * pcommon)
{
  size_t size = sizeof(bootloader_app_shared_t) - sizeof(pcommon->crc);
  uint64_t crc = CRC64_INITIAL;
  uint8_t *protected = (uint8_t *) & pcommon->signature;

  while (size--)
    {
      crc = crc64_add(crc, *protected++);
    }
  crc ^= CRC64_OUTPUT_XOR;
  return crc;
}

void bootloader_app_shared_init(bootloader_app_shared_t * pcommon, eRole_t role)
{
  memset(pcommon, 0, sizeof(bootloader_app_shared_t));
  if (role != Invalid)
    {
      pcommon->signature =
        (role ==
         App ? BOOTLOADER_COMMON_APP_SIGNATURE :
         BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
    }

}

int bootloader_app_shared_read(bootloader_app_shared_t * pcommon,
                                     eRole_t role)
{
  int rv = -EBADR;
  bootloader_app_shared_t working;

  read(&working);

  if ((role == App ? working.signature == BOOTLOADER_COMMON_APP_SIGNATURE
       : working.signature == BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE)
      && (working.crc.ull == calulate_signature(&working)))
    {
      *pcommon = working;
      rv = OK;
    }
  return rv;
}

void bootloader_app_shared_write(bootloader_app_shared_t * common,
                                       eRole_t role)
{
  bootloader_app_shared_t working = *common;
  working.signature =
    (role ==
     App ? BOOTLOADER_COMMON_APP_SIGNATURE :
     BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
  working.crc.ull = calulate_signature(&working);
  write(&working);

}

void bootloader_app_shared_invalidate(void)
{
  bootloader_app_shared_t working;
  bootloader_app_shared_init(&working, Invalid);
  write(&working);
}
