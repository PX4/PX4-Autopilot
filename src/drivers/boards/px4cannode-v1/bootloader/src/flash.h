#pragma once

typedef enum
  {
    FLASH_OK = 0,
    FLASH_ERROR,
    FLASH_ERASE_ERROR,
    FLASH_ERASE_VERIFY_ERROR,
    FLASH_ERROR_SUICIDE,
    FLASH_ERROR_AFU,

  } flash_error_t;

flash_error_t bl_flash_erase(void);
flash_error_t bl_flash_write_word(uint32_t flash_address, const uint8_t * word);
uint64_t flash_crc(uint32_t flash_address, size_t length, uint64_t initial_crc);
