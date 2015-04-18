
/*
 *
 */
#pragma once

/*
Application firmware descriptor.

This is located by the linker script somewhere aftert the vector table.
(within the first several kilobytes of the application);
This struct must be aligned on an 8-byte boundary.

The bootloader scans through the application FLASH image until it
finds the signature.

The image_crc is calculated as follows:
 1) All fields of this struct must be initalized with the correct information about
 the firmware image bin file (Here after refered to as image)
 2) image_crc set to 0;
 3) The CRC 64 is caculated over the image from offset 0 up to and including the
   last byte of the image file.
 4) The caculated CRC 64 is stored in image_crc
 5) The new image file is then written to a file a ".img" extention.
*/
typedef struct
  {
    uint64_t signature;
    uint64_t image_crc;
    uint32_t image_size;
    uint32_t vcs_commit;
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t reserved[6];
  }
__attribute__ ((packed)) app_descriptor_t;

// APDesc00
#define APP_DESCRIPTOR_SIGNATURE_ID 'A','P','D','e','s','c'
#define APP_DESCRIPTOR_SIGNATURE_REV '0','0'
#define APP_DESCRIPTOR_SIGNATURE APP_DESCRIPTOR_SIGNATURE_ID, APP_DESCRIPTOR_SIGNATURE_REV

/*
Bootloader/app common structure.

The data in this struct is passed in SRAM or the the CAN filter registers
from bootloader to application and application to bootloader.

Do not assume any mapping or location for the passing of this data
that is done in the read and write routines and is abstracted on purpose.

The application must write BOOTLOADER_COMMON_APP_SIGNATURE to the signature
field when passing data to the bootloader; when the bootloader passes data
to the app, it must write BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE to the
signature field.

The CRC is calculated over the structure from signature to the last byte.
The resuluting values are then copied to the CAN filter registers by
bootloader_app_shared_read and bootloader_app_shared_write.

*/
typedef struct
  {
    union
      {
        uint64_t ull;
        uint32_t ul[2];
      } crc;
    uint32_t signature;
    uint32_t bus_speed;
    uint32_t node_id;
  } __attribute__ ((packed)) bootloader_app_shared_t;

typedef enum eRole
  {
    Invalid,
    App,
    BootLoader
  } eRole_t;


void bootloader_app_shared_init(bootloader_app_shared_t * common,
                                     eRole_t role);
int bootloader_app_shared_read(bootloader_app_shared_t * common,
                                     eRole_t role);
void bootloader_app_shared_write(bootloader_app_shared_t * common,
                                       eRole_t role);
void bootloader_app_shared_invalidate(void);
