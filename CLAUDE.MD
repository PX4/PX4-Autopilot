AM32 ESC Flasher Module - Implementation Plan

 Context

 PX4 has no upstream support for AM32 ESC firmware updates. Matt's branch (jake/matt_firefly_am32_flasher) proves the concept works but has architectural issues: firmware baked into the build as a C
 array, ~100-state machine with poor error recovery, tight coupling to DShot via a 4-message uORB handshake, no GCS progress reporting, and ~826 lines of flashing logic injected into the DShot driver.

 Goal: Clean, upstream-quality esc_flasher module that reads firmware from SD card, is triggered via MAVLink command from QGC, reports per-ESC progress via a custom MAVLink message, and requires a
 reboot to resume normal DShot operation. Structured to allow future USB passthrough (4-way interface) extension.

 Architecture Overview

 QGC                          PX4
  │                            │
  ├─COMMAND_LONG──────────────►│ vehicle_command uORB
  │  MAV_CMD_ESC_FW_UPDATE     │         │
  │                            │    esc_flasher module
  │                            │    (dormant at boot, wakes on cmd)
  │                            │         │
  │                            │    1. Validate (disarmed, files exist)
  │                            │    2. ACK command
  │                            │    3. Stop DShot module
  │                            │    4. Take over motor GPIOs
  │                            │    5. For each ESC:
  │                            │       - Bootloader handshake
  │                            │       - Read device info → pick FW file
  │◄─ESC_FW_UPDATE_STATUS──────│       - Flash 256-byte chunks
  │   (per-ESC progress)       │       - Send RUN_APP
  │                            │    6. Publish REBOOT_REQUIRED
  │◄─COMMAND_ACK (ACCEPTED)────│    7. Go idle (reboot needed)


 New Files

 1. src/modules/esc_flasher/EscFlasher.hpp

 Module class inheriting ModuleBase<EscFlasher> + ModuleParams + ScheduledWorkItem.

 Key members:
 - _vehicle_command_sub — polls for MAV_CMD trigger
 - _esc_fw_update_status_pub — publishes per-ESC progress
 - _command_ack_pub — ACKs the initial command
 - State enum: Idle, StoppingDShot, InitGPIOs, ConnectingESC, ReadingInfo, Flashing, RunApp, Complete, Failed
 - Per-ESC state: _current_esc_index, _esc_count, _bytes_written, _firmware_size
 - GPIO pin array: uint32_t _gpio_pins[MAX_MOTORS]
 - File handle for current firmware

 2. src/modules/esc_flasher/EscFlasher.cpp

 Main module implementation (~600-800 lines, much simpler than Matt's ~1,900).

 Module lifecycle:
 - Auto-started at boot on supported boards (Kconfig opt-in)
 - Runs at 1Hz when idle (vehicle_command poll only — negligible overhead)
 - Transitions to 100Hz active polling during flashing
 - Returns to idle after completion (reboot required to restore DShot)

 DShot shutdown:
 - Calls system("dshot stop") via NuttX shell — DShot destructor calls up_dshot_arm(false)
 - Waits up to 2s for DShot to exit, then proceeds
 - Simple, no coupling — DShot module has no knowledge of the flasher

 GPIO takeover:
 - After DShot stops, configures motor pins as GPIO outputs via px4_arch_configgpio(io_timer_channel_get_gpio_output(ch))
 - Drives all pins HIGH for 600ms to force ESCs into bootloader mode on reboot
 - ESC power cycle not required — AM32 bootloader detects signal-high at startup

 Flashing loop (per ESC):
 1. Select ESC (drive target pin, keep others HIGH)
 2. Send bootloader handshake → get 9-byte device info
 3. Read firmware tag (CMD_READ_FLASH at tag address) → extract hardware target string
 4. Open /fs/microsd/am32/<target>.bin (e.g., AM32_G071_48K.bin)
 5. For each 256-byte chunk:
   - CMD_SET_ADDRESS
   - CMD_SET_BUFFER + data + CRC
   - CMD_PROG_FLASH → wait ACK
   - Publish progress (chunk_index / total_chunks * 100)
 6. Write firmware tag section
 7. CMD_RUN → ESC reboots to application
 8. Move to next ESC

 3. src/modules/esc_flasher/am32_protocol.hpp / .cpp

 Extracted and cleaned AM32 bootloader protocol implementation.

 Reused from Matt's code (protocol is correct):
 - crc16_ccitt() — CRC-16 with polynomial 0xA001
 - bitbang_send_packet() — bit-banged UART TX/RX at 19200 baud
   - 52µs bit time, 26µs half-bit for sampling
   - px4_enter_critical_section() for timing accuracy
   - hrt_absolute_time() busy-wait loops
 - Bootloader handshake packet: {0x00 * 8, 0x0D, 'B','L','H','e','l','i', CRC}
 - Command encoding: SET_ADDRESS(0xFF), SET_BUFFER(0xFE), PROG_FLASH(0x01), RUN(0x00), READ_FLASH(0x03)

 Cleaned up vs Matt's code:
 - Function-based API instead of inline state machine code
 - Proper error return codes instead of magic numbers
 - Configurable timeouts instead of hardcoded values
 - Separated from module state machine logic

 Public API:
 namespace am32 {
     int handshake(uint32_t gpio, uint8_t device_info[9]);
     int set_address(uint32_t gpio, uint16_t address);
     int write_chunk(uint32_t gpio, const uint8_t *data, uint16_t len);
     int read_flash(uint32_t gpio, uint16_t address, uint8_t *buf, uint16_t len);
     int run_app(uint32_t gpio);
     uint16_t crc16(const uint8_t *buf, uint16_t len);
 }

 4. src/modules/esc_flasher/CMakeLists.txt

 px4_add_module(
     MODULE modules__esc_flasher
     MAIN esc_flasher
     SRCS
         EscFlasher.cpp
         am32_protocol.cpp
     DEPENDS
         px4_work_queue
 )

 5. src/modules/esc_flasher/Kconfig

 menuconfig MODULES_ESC_FLASHER
     bool "esc_flasher"
     default n
     ---help---
         AM32 ESC firmware update module


 6. msg/EscFirmwareUpdateStatus.msg

 uint64 timestamp

 uint8 esc_index           # Current ESC being flashed (0-based)
 uint8 esc_count            # Total ESCs being flashed
 uint8 progress_pct         # 0-100 for current ESC
 uint8 overall_progress_pct # 0-100 for entire operation

 uint8 status
 uint8 STATUS_IDLE = 0
 uint8 STATUS_CONNECTING = 1
 uint8 STATUS_READING_INFO = 2
 uint8 STATUS_ERASING = 3
 uint8 STATUS_WRITING = 4
 uint8 STATUS_COMPLETE = 5
 uint8 STATUS_FAILED = 6
 uint8 STATUS_REBOOT_REQUIRED = 7

 uint8 error
 uint8 ERROR_NONE = 0
 uint8 ERROR_NOT_DISARMED = 1
 uint8 ERROR_NO_FIRMWARE_FILE = 2
 uint8 ERROR_COMMS_TIMEOUT = 3
 uint8 ERROR_CRC_MISMATCH = 4
 uint8 ERROR_VERIFICATION_FAILED = 5
 uint8 ERROR_UNSUPPORTED_HARDWARE = 6
 uint8 ERROR_DSHOT_STOP_FAILED = 7

 uint32 current_firmware_version  # Currently installed version
 uint32 new_firmware_version      # Version being written


 Modified Files

 7. msg/versioned/VehicleCommand.msg

 Add command constant in PX4-internal range (for development; upstream MAVLink later):

 uint32 VEHICLE_CMD_ESC_FIRMWARE_UPDATE = 100002


 - param1: ESC index (0-based, 255 = all ESCs)
 - param2: ESC type (0 = AM32)

 8. MAVLink message definition (development.xml in mavlink submodule)

 Add ESC_FIRMWARE_UPDATE_STATUS message definition. This is in the MAVLink submodule — we'll fork/PR it separately. For initial development, stream the uORB message as a generic STATUSTEXT fallback.

 9. src/modules/mavlink/streams/ESC_FIRMWARE_UPDATE_STATUS.hpp

 New MAVLink stream class that subscribes to esc_firmware_update_status uORB and sends the MAVLink message. Follow the pattern of existing streams like ESC_STATUS.hpp.

 10. src/modules/mavlink/mavlink_messages.cpp

 Register the new stream.

 11. Board configs

 Enable MODULES_ESC_FLASHER on boards that use AM32 ESCs (e.g., ARK boards). Add esc_flasher start to the board's startup script (rc.board_defaults or similar).

 Key Design Decisions

 Why stop DShot entirely (not just pause)?
 - DShot uses DMA + timer hardware that conflicts with GPIO bit-bang
 - Clean shutdown via destructor guarantees all DMA released, timers freed
 - Reboot to restore is simple and safe — avoids complex reinit edge cases
 - ESCs need power cycle anyway after reflash

 Why system("dshot stop") instead of uORB handshake?
 - Zero coupling — DShot module doesn't know flasher exists
 - Works with any output driver (PWM, OneShot, future drivers)
 - DShot's existing ModuleBase::stop() + destructor already handles cleanup
 - Matt's 4-message uORB handshake added 826 lines to DShot for this

 Why auto-start + idle instead of on-demand spawn?
 - Module is already loaded and subscribed to vehicle_command — instant response
 - 1Hz idle poll is negligible overhead
 - Avoids complexity of commander spawning external modules
 - Kconfig opt-in means boards that don't need it pay zero cost

 Why separate am32_protocol from module?
 - Clean separation of protocol logic from module orchestration
 - Protocol code can be reused by future 4-way passthrough implementation
 - Easier to test protocol functions in isolation

 Firmware File Convention

 Files in /fs/microsd/am32/:
 - Named by AM32 hardware target: AM32_G071_48K.bin, AM32_F051_32K.bin, etc.
 - The flasher connects to each ESC bootloader, reads the firmware tag area to determine hardware target
 - Maps tag → filename, opens file, flashes
 - If no matching file found, reports ERROR_NO_FIRMWARE_FILE for that ESC and moves on

 State Machine (simplified)

 Idle ──(MAV_CMD)──► StoppingDShot ──(dshot stopped)──► InitGPIOs
                                                            │
                                                     (pins HIGH, 600ms)
                                                            │
                                                     ┌──► ConnectESC
                                                     │      │
                                                     │  (handshake OK)
                                                     │      │
                                                     │   ReadInfo ──► LoadFirmware ──► Flash
                                                     │                                  │
                                                     │                           (all chunks done)
                                                     │                                  │
                                                     │   RunApp ◄──────────────────────┘
                                                     │      │
                                                     │  (next ESC?)
                                                     │      │
                                                     └──YES─┘
                                                            │ NO
                                                            ▼
                                                       Complete
                                                    (REBOOT_REQUIRED)


 Error from any flashing state → Failed (with error code), skip to next ESC or abort.

 Implementation Order

 1. am32_protocol.cpp/hpp — Extract and clean up Matt's protocol code (CRC, bitbang, commands)
 2. EscFirmwareUpdateStatus.msg — uORB message definition
 3. VehicleCommand.msg — Add command constant
 4. EscFlasher.hpp/cpp — Module skeleton with state machine
 5. CMakeLists.txt + Kconfig — Build integration
 6. MAVLink stream — ESC_FIRMWARE_UPDATE_STATUS.hpp + register in mavlink_messages.cpp
 7. Board config — Enable on a test board
 8. Test — SITL smoke test (state machine logic), then hardware test

 Verification

 1. Build: make ark_fmu-v6x_default (or target board) — verify clean compile
 2. SITL smoke test: Module starts, responds to vehicle command, reports error (no GPIOs in SITL)
 3. Hardware test:
   - Place AM32 firmware on SD card
   - Send MAV_CMD_ESC_FIRMWARE_UPDATE via commander shell or MAVLink
   - Verify DShot stops, ESCs enter bootloader, firmware flashes, progress reported
   - Reboot, verify ESCs running new firmware
 4. QGC integration: Verify COMMAND_ACK and status messages appear in MAVLink inspector

 Future Extensions (not in this PR)

 - 4-way interface passthrough: Add USB serial passthrough mode using the same am32_protocol infrastructure. The flasher module would proxy between USB serial (4-way protocol) and GPIO bit-bang.
 - Runtime DShot restart: Instead of requiring reboot, add dshot start after flashing. Needs careful timer/DMA reinit validation.
 - ESC settings read/write: Extend am32_protocol with EEPROM read/write commands.
 - Orchestrator module: If hot-switching between DShot and passthrough becomes needed, add a motor_output_mgr similar to CDC ACM.
