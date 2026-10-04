/* CM4 memory map, the single source for the CM7 (src/board_config.h,
 * src/rpmsg.c) and the CM4 image (link.ld.in, startup.c, rpmsg_remote.c).
 * Plain integer expressions only: link.ld.in is preprocessed with this file.
 *
 * Only the code TCM is known to be reachable from the CM7 (LMEM backdoor
 * window at CM4_CODE_TCM_PA). Code, .data, the resource table, vrings,
 * rpmsg buffers, the application name and the status block all live there;
 * the system TCM holds CM4-private bss and stack only.
 */
#pragma once

#define CM4_TCM_SIZE       (128 * 1024)
#define CM4_CODE_TCM_DA    0x1FFE0000	/* CM4 view */
#define CM4_SYS_TCM_DA     0x20000000
#define CM4_CODE_TCM_PA    0x20200000	/* CM7 view through the backdoor window */
#define CM4_SYS_TCM_PA     0x20220000	/* second window half; never read by the CM7 */

/* Shared window: upper half of the code TCM. The top 32 bytes are the
 * application name (16, NUL padded) then the status block (16) and are
 * excluded from the space the host may place vrings and buffers in.
 */
#define CM4_SHM_OFFSET     0x10000
#define CM4_SHM_SIZE       (64 * 1024)
#define CM4_STATUS_SIZE    16
#define CM4_APP_NAME_LEN   16
#define CM4_SHM_RESERVED   (CM4_STATUS_SIZE + CM4_APP_NAME_LEN)

#define CM4_SHM_DA         (CM4_CODE_TCM_DA + CM4_SHM_OFFSET)
#define CM4_SHM_PA         (CM4_CODE_TCM_PA + CM4_SHM_OFFSET)
#define CM4_SHM_USABLE     (CM4_SHM_SIZE - CM4_SHM_RESERVED)
#define CM4_STATUS_DA      (CM4_SHM_DA + CM4_SHM_SIZE - CM4_STATUS_SIZE)
#define CM4_STATUS_PA      (CM4_SHM_PA + CM4_SHM_SIZE - CM4_STATUS_SIZE)
#define CM4_APP_NAME_DA    (CM4_STATUS_DA - CM4_APP_NAME_LEN)
#define CM4_APP_NAME_PA    (CM4_STATUS_PA - CM4_APP_NAME_LEN)
