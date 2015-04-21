#include <nuttx/config.h>
#include "board_config.h"

#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "chip.h"
#include "stm32.h"
#include "nvic.h"

#include "flash.h"
#include "timer.h"
#include "driver.h"
#include "protocol.h"
#include "crc.h"
#include "boot_app_shared.h"

#include "board_config.h"


uint64_t get_short_unique_id(void);
void send_log_message(uint8_t node_id, uint8_t level, uint8_t stage,
                                 uint8_t status);

int get_dynamic_node_id(bl_timer_id tboot, uint32_t *allocated_node_id);
int autobaud_and_get_dynamic_node_id(bl_timer_id tboot, can_speed_t *speed,
                                    uint32_t *node_id);
void poll_getnodeinfo(uint8_t node_id, uint32_t uptime, uint8_t status);

int wait_for_beginfirmwareupdate(bl_timer_id tboot,
                                             uint8_t * fw_path,
                                             uint8_t * fw_path_length,
                                             uint8_t * fw_source_node_id);

void file_getinfo(size_t * fw_image_size, uint64_t * fw_image_crc,
                             const uint8_t * fw_path, uint8_t fw_path_length,
                             uint8_t fw_source_node_id);

flash_error_t file_read_and_program(uint8_t fw_source_node_id,
                                               uint8_t fw_path_length,
                                               const uint8_t * fw_path,
                                               size_t fw_image_size,
                                               uint32_t * fw_word0);

static uint8_t is_app_valid(uint32_t first_word);
void find_descriptor(void);
void application_run(size_t fw_image_size);

volatile struct {
  can_speed_t bus_speed;
  volatile uint8_t node_id;
  volatile uint8_t status_code;
  volatile bool app_valid;
  volatile uint32_t uptime;
  volatile app_descriptor_t *fw_image_descriptor;
  volatile uint32_t *fw_image;
  bool  wait_for_getnodeinfo;
  bool  app_bl_request;
  bool sent_node_info_response;

} bootloader;


static void uptime_process(bl_timer_id id, void *context)
{
  bootloader.uptime++;
}

/*
 * Once we have a noide id
 * Handle GetNodeInfo replies regardless of application state
 */

static void node_info_process(bl_timer_id id, void *context)
{

  uavcan_getnodeinfo_response_t response;
  uavcan_nodestatus_t node_status;
  uavcan_frame_id_t frame_id;
  size_t frame_len;
  uint32_t rx_message_id;
  uint8_t frame_payload[8];

  node_status.uptime_sec = bootloader.uptime;
  node_status.status_code = bootloader.status_code;
  node_status.vendor_specific_status_code = 0u;
  uavcan_pack_nodestatus(response.nodestatus, &node_status);

  board_get_hardware_version(&response.hardware_version);
  response.name_length = board_get_product_name(response.name, sizeof(response.name));
  memset(&response.software_version,0, sizeof(response.software_version));

  if (bootloader.app_valid) {
      response.software_version.major =
          bootloader.fw_image_descriptor->major_version;
      response.software_version.minor =
          bootloader.fw_image_descriptor->minor_version;
      response.software_version.optional_field_mask = 3u;
          bootloader.fw_image_descriptor->minor_version;
      response.software_version.vcs_commit =
          bootloader.fw_image_descriptor->vcs_commit;
      response.software_version.image_crc =
          bootloader.fw_image_descriptor->image_crc;
  }

  rx_message_id = 0u;
  if (can_rx(&rx_message_id, &frame_len, frame_payload, fifoGetNodeInfo) &&
      uavcan_parse_message_id(&frame_id, rx_message_id, UAVCAN_GETNODEINFO_DTID) &&
          frame_payload[0] == bootloader.node_id &&
          frame_id.last_frame) {
      uavcan_tx_getnodeinfo_response(bootloader.node_id, &response,
                                     frame_id.source_node_id,
                                     frame_id.transfer_id);

      bootloader.sent_node_info_response = true;
  }


}

/*
 * Once we have a noide id
 * Handle GetNodeInfo replies regardless of application state
 */
static void node_status_process(bl_timer_id id, void *context)
{
  uavcan_tx_nodestatus(bootloader.node_id, bootloader.uptime, bootloader.status_code);
}

__EXPORT int main(int argc, char *argv[])
{

  uint64_t fw_image_crc;
  size_t fw_image_size;
  uint32_t fw_word0;
  uint8_t fw_path[200];
  uint8_t fw_path_length;
  uint8_t fw_source_node_id;
  uint8_t error_log_stage;
  flash_error_t status;
  bootloader_app_shared_t common;

#ifdef OPT_ENABLE_WD
  /* ... */
#endif

  memset((void *)&bootloader, 0, sizeof(bootloader));

  bootloader.status_code = UAVCAN_NODESTATUS_STATUS_INITIALIZING;

  error_log_stage = UAVCAN_LOGMESSAGE_STAGE_INIT;

  bootloader.fw_image = (volatile uint32_t *)(APPLICATION_LOAD_ADDRESS);

#if defined(OPT_WAIT_FOR_GETNODEINFO)
  bootloader.wait_for_getnodeinfo = 1;
#endif

#if defined(OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO)
  bootloader.wait_for_getnodeinfo &= stm32_gpioread(GPIO_GETNODEINFO_JUMPER) ^ OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO_INVERT;
#endif


  bootloader.app_valid = is_app_valid(bootloader.fw_image[0]);

  board_indicate(reset);


  bootloader.app_bl_request = (OK == bootloader_app_shared_read(&common, App)) &&
    common.bus_speed && common.node_id;
  bootloader_app_shared_invalidate();

  bl_timer_cb_t p = null_cb;
  p.cb = uptime_process;
  timer_allocate(modeRepeating|modeStarted, 1000, &p);

  p.cb = node_info_process;
  bl_timer_id tinfo = timer_allocate(modeRepeating, OPT_NODE_INFO_RATE_MS, &p);

  p.cb = node_status_process;
  bl_timer_id tstatus = timer_allocate(modeRepeating, OPT_NODE_STATUS_RATE_MS, &p);



  bl_timer_id tboot = timer_allocate(modeRepeating, OPT_TBOOT_MS , 0);

  if (!bootloader.wait_for_getnodeinfo && !bootloader.app_bl_request && bootloader.app_valid) {
      timer_start(tboot);
  }

  if (bootloader.app_bl_request) {

      bootloader.bus_speed = common.bus_speed;
      bootloader.node_id = (uint8_t) common.node_id;
      can_init(can_freq2speed(common.bus_speed), CAN_Mode_Normal);

  } else {

      can_speed_t speed;

      if (CAN_OK != autobaud_and_get_dynamic_node_id(tboot, &speed, &common.node_id)) {
          goto boot;
      }

      /* reset uptime */
      bootloader.uptime = 0;
      common.bus_speed = can_speed2freq(speed);
      bootloader.node_id = (uint8_t) common.node_id;

  }
  // Start Processes that requier Node Id
  timer_start(tinfo);
  timer_start(tstatus);

  while (!bootloader.sent_node_info_response) {
      if (timer_expired(tboot))
        {
          goto boot;
        }
  }

  if (bootloader.app_valid &&
     (bootloader.wait_for_getnodeinfo ||
      bootloader.app_bl_request)) {

      timer_start(tboot);
  }

  do
    {
      if (CAN_BOOT_TIMEOUT == wait_for_beginfirmwareupdate(tboot,fw_path,
                                                          &fw_path_length,
                                                          &fw_source_node_id))
        {
          goto boot;
        }
    }
  while (!fw_source_node_id);

  timer_stop(tboot);
  board_indicate(fw_update_start);

  file_getinfo(&fw_image_size, &fw_image_crc, fw_path, fw_path_length,
               fw_source_node_id);

  //todo:Check this
  if (fw_image_size < sizeof(app_descriptor_t) || !fw_image_crc)
    {
      error_log_stage = UAVCAN_LOGMESSAGE_STAGE_GET_INFO;
      goto failure;
    }

  /* UAVCANBootloader_v0.3 #28.6: 1023.LogMessage.uavcan("Erase") */
  send_log_message(bootloader.node_id,
                              UAVCAN_LOGMESSAGE_LEVEL_INFO,
                              UAVCAN_LOGMESSAGE_STAGE_ERASE,
                              UAVCAN_LOGMESSAGE_RESULT_START);


  /* Need to signal that the app is no longer valid  if Node Info Request are done */

  bootloader.app_valid = false;

  status = bl_flash_erase(APPLICATION_LOAD_ADDRESS);
  if (status != FLASH_OK)
    {
      /* UAVCANBootloader_v0.3 #28.8: [Erase
       * Failed]:INDICATE_FW_UPDATE_ERASE_FAIL */
      board_indicate(fw_update_erase_fail);

      error_log_stage = UAVCAN_LOGMESSAGE_STAGE_ERASE;
      goto failure;
    }

  status = file_read_and_program(fw_source_node_id, fw_path_length, fw_path,
                                 fw_image_size, &fw_word0);
  if (status != FLASH_OK)
    {
      error_log_stage = UAVCAN_LOGMESSAGE_STAGE_PROGRAM;
      goto failure;
    }

  /* UAVCANBootloader_v0.3 #41: CalulateCRC(file_info) */
  if (!is_app_valid(fw_word0))
    {
      bootloader.app_valid = 0u;

      /* UAVCANBootloader_v0.3 #43: [crc Fail]:INDICATE_FW_UPDATE_INVALID_CRC */
      board_indicate(fw_update_invalid_crc);

      error_log_stage = UAVCAN_LOGMESSAGE_STAGE_VALIDATE;
      goto failure;
    }

  /* UAVCANBootloader_v0.3 #46: [crc == fw_crc]:write word0 */
  status = bl_flash_write_word((uint32_t) bootloader.fw_image, (uint8_t *) & fw_word0);
  if (status != FLASH_OK)
    {
      /* Not specifically listed in UAVCANBootloader_v0.3:
       * 1023.LogMessage.uavcan */
      error_log_stage = UAVCAN_LOGMESSAGE_STAGE_FINALIZE;
      goto failure;
    }

  /* UAVCANBootloader_v0.3 #47: ValidateBootLoaderAppCommon() */
  bootloader_app_shared_write(&common, BootLoader);

  /* Send a completion log message */
  send_log_message(bootloader.node_id,
                              UAVCAN_LOGMESSAGE_LEVEL_INFO,
                              UAVCAN_LOGMESSAGE_STAGE_FINALIZE,
                              UAVCAN_LOGMESSAGE_RESULT_OK);

  /* TODO UAVCANBootloader_v0.3 #48: KickTheDog() */

boot:
  /* UAVCANBootloader_v0.3 #50: jump_to_app */
  application_run(fw_image_size);

  /* We will fall thru if the Iamage is bad */

failure:
  /* UAVCANBootloader_v0.3 #28.2:
   * [!validateFileInfo(file_info)]:1023.LogMessage.uavcan (errorcode) */
  /* UAVCANBootloader_v0.3 #28.9: [Erase Fail]:1023.LogMessage.uavcan */
  /* UAVCANBootloader_v0.3 #31: [Program Fail]::1023.LogMessage.uavcan */
  /* UAVCANBootloader_v0.3 #38: [(retries == 0 &&
   * timeout)]:1023.LogMessage.uavcan */
  /* UAVCANBootloader_v0.3 #42: [crc Faill]:1023.LogMessage.uavcan */
  send_log_message(bootloader.node_id,
                              UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                              error_log_stage, UAVCAN_LOGMESSAGE_RESULT_FAIL);

  /* UAVCANBootloader_v0.3 #28.3:
   * [!validateFileInfo(file_info)]:550.NodeStatus.uavcan(uptime=t,
   * STATUS_CRITICAL) */
  /* UAVCANBootloader_v0.3 #28.10: [Erase
   * Fail]:550.NodeStatus.uavcan(uptime=t,STATUS_CRITICAL) */
  /* UAVCANBootloader_v0.3 #32: [Program Fail]:550.NodeStatus.uavcan(uptime=t,
   * STATUS_CRITICAL) */
  /* UAVCANBootloader_v0.3 #39: [(retries == 0 &&
   * timeout)]:550.NodeStatus.uavcan(uptime=t, STATUS_CRITICAL) */
  /* UAVCANBootloader_v0.3 #44: [crc Fail]:550.NodeStatus.uavcan(uptime=t,
   * STATUS_CRITICAL) */
  bootloader.status_code = UAVCAN_NODESTATUS_STATUS_CRITICAL;

  /* UAVCANBootloader_v0.3 #28.1:
   * [Retries==0]:InvalidateBootLoaderAppCommon(),RestartWithDelay(20,000ms) */
  /* UAVCANBootloader_v0.3 #40: [Retries==0]:InvalidateBootLoaderAppCommon(),
   * RestartWithDelay(20,000ms) */
  /* UAVCANBootloader_v0.3 #45: [crc
   * Fail]:InvalidateBootLoaderAppCommon(),RestartWithDelay(20,000ms) */

  bl_timer_id tmr = timer_allocate(modeTimeout|modeStarted , OPT_RESTART_TIMEOUT_MS, 0);
  while(!timer_expired(tmr))
    {
      ;
    }
  timer_free(tmr);
  up_systemreset();
}

uint64_t get_short_unique_id(void)
{
  uavcan_hardwareversion_t hw_version;
  uint64_t result;
  size_t i;

  board_get_hardware_version(&hw_version);

  result = CRC64_INITIAL;
  for (i = 0; i < 16u; i++)
    {
      result = crc64_add(result, hw_version.unique_id[i]);
    }

  return (result ^ CRC64_OUTPUT_XOR) & 0x01FFFFFFFFFFFFFFL;
}

int autobaud_and_get_dynamic_node_id(bl_timer_id tboot, can_speed_t *speed, uint32_t *node_id)
{
  board_indicate(autobaud_start);
  int rv = can_autobaud(speed, tboot);
  if (rv != CAN_BOOT_TIMEOUT) {
      can_init(*speed, CAN_Mode_Normal);

      board_indicate(autobaud_end);
      board_indicate(allocation_start);
      rv = get_dynamic_node_id(tboot, node_id);
      if (rv != CAN_BOOT_TIMEOUT) {
        board_indicate(allocation_end);
      }
  }
  return rv;
}

int get_dynamic_node_id(bl_timer_id tboot, uint32_t *allocated_node_id)
{
    uavcan_hardwareversion_t hw_version;
    uavcan_frame_id_t rx_frame_id;

    uint32_t rx_message_id;
    uint8_t  rx_payload[8];
    size_t   rx_len;
    uint16_t rx_crc;

    struct {
      uint8_t node_id;
      uint8_t transfer_id;
      uint8_t allocated_node_id;
      uint8_t frame_index;
    } server;

    *allocated_node_id     = 0;

    uint8_t transfer_id = 0;

    size_t i;
    size_t offset;
    uint16_t expected_crc;

    bool unique_id_matched = false;


    board_get_hardware_version(&hw_version);

    memset(&server,0 , sizeof(server));
    rx_crc = 0u;

    /*
    Rule A: on initialization, the client subscribes to
    uavcan.protocol.dynamic_node_id.Allocation and starts a Request Timer with
    interval of Trequestinterval = 1 second
    */

    bl_timer_id trequest = timer_allocate(modeTimeout|modeStarted, 1000, 0);

    do {
        /*
        Rule B. On expiration of Request Timer:
        1) Request Timer restarts.
        2) The client broadcasts a first-stage Allocation request message,
           where the fields are assigned following values:
                node_id                 - preferred node ID, or zero if the
                                          client doesn't have any preference
                first_part_of_unique_id - true
                unique_id               - first 7 bytes of unique ID
        */
        if (timer_expired(trequest)) {
            uavcan_tx_allocation_message(*allocated_node_id, 16u, hw_version.unique_id,
                                         0u, transfer_id++);
            timer_start(trequest);
        }

        if (can_rx(&rx_message_id, &rx_len, rx_payload, fifoAll) &&
            uavcan_parse_message_id(&rx_frame_id, rx_message_id, UAVCAN_DYNAMICNODEIDALLOCATION_DTID)) {

          /*
          Rule C. On any Allocation message, even if other rules also match:
          1) Request Timer restarts.
          */

          timer_start(trequest);

          /*
          Skip this message if it's anonymous (from another client), or if it's
          from a different server to the one we're listening to at the moment
          */
          if (!rx_frame_id.source_node_id || (server.node_id &&
                      rx_frame_id.source_node_id != server.node_id)) {
              continue;
          } else if (!server.node_id ||
                      rx_frame_id.transfer_id != server.transfer_id ||
                      rx_frame_id.frame_index == 0) {
              /* First (only?) frame of the transfer */
              if (rx_frame_id.last_frame) {
                  offset = 1u;
              } else {
                  rx_crc = (uint16_t)(rx_payload[0] | (rx_payload[1] << 8u));
                  server.allocated_node_id = rx_payload[2];
                  offset = 3u;
              }
              server.node_id = rx_frame_id.source_node_id;
              server.transfer_id = rx_frame_id.transfer_id;
              unique_id_matched = 0u;
              server.frame_index = 1u;
          } else if (rx_frame_id.frame_index != server.frame_index) {
              /* Abort if the frame index is wrong */
              server.node_id = 0u;
              continue;
          } else {
              offset = 0u;
              server.frame_index++;
          }

          /*
          Rule D. On an Allocation message WHERE (source node ID is
          non-anonymous) AND (client's unique ID starts with the bytes available
          in the field unique_id) AND (unique_id is less than 16 bytes long):
          1) The client broadcasts a second-stage Allocation request message,
             where the fields are assigned following values:
                  node_id                 - same value as in the first-stage
                  first_part_of_unique_id - false
                  unique_id               - at most 7 bytes of local unique ID
                                            with an offset equal to number of
                                            bytes in the received unique ID

          Rule E. On an Allocation message WHERE (source node ID is
          non-anonymous) AND (unique_id fully matches client's unique ID) AND
          (node_id in the received message is not zero):
          1) Request Timer stops.
          2) The client initializes its node_id with the received value.
          3) The client terminates subscription to Allocation messages.
          4) Exit.
          */

          /* Count the number of unique ID bytes matched */
          for (i = offset; i < rx_len && unique_id_matched < 16u &&
                  hw_version.unique_id[unique_id_matched] == rx_payload[i];
              unique_id_matched++, i++);

          if (i < rx_len) {
              /* Abort if we didn't match the whole unique ID */
              server.node_id = 0u;
          } else if (rx_frame_id.last_frame && unique_id_matched < 16u) {
              /* Case D */
              uavcan_tx_allocation_message(*allocated_node_id, 16u, hw_version.unique_id,
                                           unique_id_matched, transfer_id++);
          } else if (rx_frame_id.last_frame) {
              /* Validate CRC */
              expected_crc = UAVCAN_ALLOCATION_CRC;
              expected_crc = crc16_add(expected_crc, server.allocated_node_id);
              expected_crc = crc16_signature(expected_crc, 16u,
                                             hw_version.unique_id);

              /* Case E */
              if (rx_crc == expected_crc) {
                  *allocated_node_id = server.allocated_node_id >> 1u;
                  break;
              }
          }
        }
    } while (!timer_expired(tboot));

    timer_free(trequest);
    return *allocated_node_id == 0 ? CAN_BOOT_TIMEOUT : CAN_OK;
}


int wait_for_beginfirmwareupdate(bl_timer_id tboot,
                                             uint8_t * fw_path,
                                             uint8_t * fw_path_length,
                                             uint8_t * fw_source_node_id)
{
  uavcan_beginfirmwareupdate_request_t request;
  uavcan_frame_id_t frame_id;
  size_t i;
  uint8_t frame_payload[8];
  can_error_t status;

  status = CAN_ERROR;
  fw_path[0] = 0;
  *fw_source_node_id = 0;

  while (status != CAN_OK)
    {
      if (timer_expired(tboot)) {
        return CAN_BOOT_TIMEOUT;
      }
      status = uavcan_rx_beginfirmwareupdate_request(bootloader.node_id,
                                                     &request, &frame_id);
    }

  if (status == CAN_OK)
    {
      /* UAVCANBootloader_v0.3 #22.2: Resp580.BeginFirmwareUpdate.uavcan */

      /* Send an ERROR_OK response */
      frame_payload[0] = frame_id.source_node_id;
      frame_payload[1] = 0u;

      frame_id.last_frame = 1u;
      frame_id.frame_index = 0u;
      frame_id.source_node_id = bootloader.node_id;
      frame_id.transfer_type = SERVICE_RESPONSE;

      can_tx(uavcan_make_message_id(&frame_id), 2u, frame_payload, MBAll);

      /* UAVCANBootloader_v0.3 #22.3: fwPath = image_file_remote_path */
      for (i = 0u; i < request.path_length; i++)
        {
          fw_path[i] = request.path[i];
        }
      *fw_path_length = request.path_length;
      /* UAVCANBootloader_v0.3 #22.4: fwSourceNodeID = source_node_id */
      *fw_source_node_id = request.source_node_id;
    }
  return status;
}

void file_getinfo(size_t * fw_image_size,
                             uint64_t * fw_image_crc,
                             const uint8_t * fw_path,
                             uint8_t fw_path_length, uint8_t fw_source_node_id)
{
  uavcan_getinfo_request_t request;
  uavcan_getinfo_response_t response;
  uint8_t transfer_id, retries, i;
  can_error_t status;

  for (i = 0; i < fw_path_length; i++)
    {
      request.path[i] = fw_path[i];
    }
  request.path_length = fw_path_length;

  /* UAVCANBootloader_v0.3 #25: SetRetryAndTimeout(3,1000MS) */
  retries = UAVCAN_SERVICE_RETRIES;
  transfer_id = 0;

  *fw_image_size = 0;
  *fw_image_crc = 0;

  while (retries)
    {
      /* UAVCANBootloader_v0.3 #26: 585.GetInfo.uavcan(path) */
      uavcan_tx_getinfo_request(bootloader.node_id, &request,
                                fw_source_node_id, transfer_id);

      /* UAVCANBootloader_v0.3 #28:
       * 585.GetInfo.uavcan(fw_path,fw_crc,fw_size...), */
      status =
        uavcan_rx_getinfo_response(bootloader.node_id, &response,
                                   fw_source_node_id, transfer_id,
                                   UAVCAN_SERVICE_TIMEOUT_MS);

      transfer_id++;

      /* UAVCANBootloader_v0.3 #27: validateFileInfo(file_info, &errorcode) */
      if (status == CAN_OK && response.error == UAVCAN_FILE_ERROR_OK &&
          (response.entry_type & UAVCAN_GETINFO_ENTRY_TYPE_FLAG_FILE) &&
          (response.entry_type & UAVCAN_GETINFO_ENTRY_TYPE_FLAG_READABLE) &&
          response.size > 0u && response.size < OPT_APPLICATION_IMAGE_LENGTH)
        {
          /* UAVCANBootloader_v0.3 #28.4: save(file_info) */
          *fw_image_size = response.size;
          *fw_image_crc = response.crc64;
          break;
        }
      else
        {
          retries--;
        }
    }
}

flash_error_t file_read_and_program(uint8_t fw_source_node_id,
                                               uint8_t fw_path_length,
                                               const uint8_t * fw_path,
                                               size_t fw_image_size,
                                               uint32_t * fw_word0)
{
  uavcan_read_request_t request;
  uavcan_read_response_t response;
  size_t bytes_written, i, write_length;
  can_error_t can_status;
  flash_error_t flash_status;
  uint32_t next_read_deadline;
  uint8_t transfer_id, retries, write_remainder[4], write_remainder_length,
    *data;

  /* Set up the read request */
  for (i = 0; i < fw_path_length; i++)
    {
      request.path[i] = fw_path[i];
    }
  request.path_length = fw_path_length;
  request.offset = 0u;

  bytes_written = 0u;
  write_remainder_length = 0u;
  transfer_id = 0u;
  next_read_deadline = 0u;

  do
    {
      /* 
       * Rate limiting on read requests: - 2/sec on a 125 Kbaud bus - 4/sec on
       * a 250 Kbaud bus - 8/sec on a 500 Kbaud bus - 16/sec on a 1 Mbaud bus */
      while (bootloader.uptime < next_read_deadline);
      next_read_deadline = bootloader.uptime + 6u;

      /* UAVCANBootloader_v0.3 #28.11: SetRetryAndTimeout(3,1000MS) */
      retries = UAVCAN_SERVICE_RETRIES;
      can_status = CAN_ERROR;
      while (retries && can_status != CAN_OK)
        {
          /* UAVCANBootloader_v0.3 #28.12: 588.Read.uavcan(0, path) */
          /* UAVCANBootloader_v0.3 #33: 588.Read.uavcan(N,path) */
          request.offset = bytes_written + write_remainder_length;
          uavcan_tx_read_request(bootloader.node_id, &request,
                                 fw_source_node_id, transfer_id);

          /* UAVCANBootloader_v0.3 #28.12.1: 588.Read.uavcan(0,path,data) */
          /* UAVCANBootloader_v0.3 #33.1: 583.Read.uavcan(0,path,data) */
          can_status = uavcan_rx_read_response(bootloader.node_id,
                                               &response, fw_source_node_id,
                                               transfer_id,
                                               UAVCAN_SERVICE_TIMEOUT_MS);

          transfer_id++;

          /* UAVCANBootloader_v0.3 #34: ValidateReadResp(resp) */
          if (can_status != CAN_OK || response.error != UAVCAN_FILE_ERROR_OK)
            {
              can_status = CAN_ERROR;

              /* UAVCANBootloader_v0.3 #35: [(retries != 0 && timeout) ||
               * !ValidReadReadResponse]:INDICATE_FW_UPDATE_INVALID_RESPONSE */
              board_indicate(fw_update_invalid_response);

              /* UAVCANBootloader_v0.3 #36: [(retries != 0 && timeout) ||
               * !ValidReadReadResponse]:1023.LogMessage.uavcan */
              send_log_message(bootloader.node_id,
                                          UAVCAN_LOGMESSAGE_LEVEL_ERROR,
                                          UAVCAN_LOGMESSAGE_STAGE_PROGRAM,
                                          UAVCAN_LOGMESSAGE_RESULT_FAIL);
              retries--;
            }
        }

      if (can_status != CAN_OK)
        {
          /* UAVCANBootloader_v0.3 #37: [(retries == 0 &&
           * timeout]:INDICATE_FW_UPDATE_TIMEOUT */
          board_indicate(fw_update_timeout);
          break;
        }

      data = response.data;
      write_length = response.data_length;

      /* 
       * If this is the last read (zero length) and there are bytes left to
       * write, pad the firmware image out with zeros to ensure a full-word
       * write. */
      if (write_length == 0u && write_remainder_length > 0u)
        {
          write_length = 4u - write_remainder_length;
          data[0] = data[1] = data[2] = 0u;
        }

      /* 
       * If the length of a previous read was not a multiple of 4, we'll have a 
       * few bytes left over which need to be combined with the next write as
       * all writes must be word-aligned and a whole number of words long. */
      flash_status = FLASH_OK;
      while (write_length && flash_status == FLASH_OK)
        {
          write_remainder[write_remainder_length++] = *(data++);
          write_length--;

          if (write_remainder_length == 4u)
            {
              if (bytes_written == 0u)
                {
                  /* UAVCANBootloader_v0.3 #30: SaveWord0 */
                  ((uint8_t *) fw_word0)[0] = write_remainder[0];
                  ((uint8_t *) fw_word0)[1] = write_remainder[1];
                  ((uint8_t *) fw_word0)[2] = write_remainder[2];
                  ((uint8_t *) fw_word0)[3] = write_remainder[3];
                }
              else
                {
                  flash_status =
                    bl_flash_write_word((uint32_t)
                                        (&bootloader.fw_image[bytes_written >> 2u]),
                                        write_remainder);
                }

              bytes_written += 4u;
              write_remainder_length = 0u;
            }
        }
    }
  while (bytes_written <= fw_image_size && response.data_length != 0u &&
         flash_status == FLASH_OK);

  /* 
   * Return success iff the last read succeeded, the last write succeeded, the
   * correct number of bytes were written, and the length of the last response
   * was zero. */
  if (can_status == CAN_OK && flash_status == FLASH_OK &&
      bytes_written == fw_image_size && response.data_length == 0u)
    {
      return FLASH_OK;
    }
  else
    {
      return FLASH_ERROR;
    }
}

void send_log_message(uint8_t node_id,
                                 uint8_t level, uint8_t stage, uint8_t status)
{
  uavcan_logmessage_t message;
  uavcan_frame_id_t frame_id;
  uint8_t payload[8];
  size_t frame_len;

  frame_id.transfer_id = 0;
  frame_id.last_frame = 1u;
  frame_id.frame_index = 0;
  frame_id.source_node_id = node_id;
  frame_id.transfer_type = MESSAGE_BROADCAST;
  frame_id.data_type_id = UAVCAN_LOGMESSAGE_DTID;

  message.level = level;
  message.message[0] = stage;
  message.message[1] = status;
  frame_len = uavcan_pack_logmessage(payload, &message);
  can_tx(uavcan_make_message_id(&frame_id), frame_len, payload, MBAll);
}

static uint8_t is_app_valid(uint32_t first_word)
{
  uint64_t crc;
  size_t i, length, crc_offset;
  uint8_t byte;

  find_descriptor();

  /* UAVCANBootloader_v0.3 #7: bool AppValid = (AppFlash[0] != 0xffffffff) &&
   * ComputeAppCRC(APP_LOADADDR, APP_INFO_OFFSET) */
  if (!bootloader.fw_image_descriptor || first_word == 0xFFFFFFFFu)
    {
      return 0u;
    }

  length = bootloader.fw_image_descriptor->image_size;
  crc_offset = (size_t) (&bootloader.fw_image_descriptor->image_crc) -
    (size_t) bootloader.fw_image;

  crc = CRC64_INITIAL;
  for (i = 0u; i < 4u; i++)
    {
      crc = crc64_add(crc, (uint8_t) (first_word >> (i << 3u)));
    }
  for (i = 4u; i < length; i++)
    {
      if (crc_offset <= i && i < crc_offset + 8u)
        {
          /* Zero out the CRC field while computing the CRC */
          byte = 0u;
        }
      else
        {
          byte = ((volatile uint8_t *)bootloader.fw_image)[i];
        }
      crc = crc64_add(crc, byte);
    }
  crc ^= CRC64_OUTPUT_XOR;

  return crc == bootloader.fw_image_descriptor->image_crc;
}

void find_descriptor(void)
{
  bootloader.fw_image_descriptor = NULL;
  uint64_t *p = (uint64_t *) APPLICATION_LOAD_ADDRESS;
  union
     {
       uint64_t ull;
       char text[sizeof(uint64_t)];
     } sig = {
          .text = {APP_DESCRIPTOR_SIGNATURE}
     };
   do {
      if (*p ==  sig.ull) {
          bootloader.fw_image_descriptor = (volatile app_descriptor_t *) p;
        }
    } while(bootloader.fw_image_descriptor == NULL && ++p < APPLICATION_LAST_64BIT_ADDRRESS);
}

static void do_jump(uint32_t stacktop, uint32_t entrypoint)
{
  asm volatile ("msr msp, %0    \n"
                "bx     %1      \n"::"r" (stacktop), "r"(entrypoint):);
  // just to keep noreturn happy
  for (;;);
}

void application_run(size_t fw_image_size)
{
  /* 
   * We refuse to program the first word of the app until the upload is marked
   * complete by the host.  So if it's not 0xffffffff, we should try booting it.
   
   * The second word of the app is the entrypoint; it must point within the
   * flash area (or we have a bad flash).
   */
  if (bootloader.fw_image[0] != 0xffffffff
      && bootloader.fw_image[1] > APPLICATION_LOAD_ADDRESS
      && bootloader.fw_image[1] < (APPLICATION_LOAD_ADDRESS + fw_image_size))
    {

      (void)irqsave();

      stm32_boarddeinitialize();

      /* kill the systick interrupt */
      up_disable_irq(STM32_IRQ_SYSTICK);
      putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT),
               NVIC_SYSTICK_CTRL);

      /* and set a specific LED pattern */
      board_indicate(jump_to_app);

      /* the interface */

      /* switch exception handlers to the application */
      __asm volatile ("dsb");
      __asm volatile ("isb");
      putreg32(APPLICATION_LOAD_ADDRESS, NVIC_VECTAB);
      __asm volatile ("dsb");
      /* extract the stack and entrypoint from the app vector table and go */
      do_jump(bootloader.fw_image[0], bootloader.fw_image[1]);
    }
}
