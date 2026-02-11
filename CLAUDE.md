# Bitbang DShot Prototype - Design Specification

## Goal

Implement bitbang DShot (including bidirectional) as an alternative to the existing DMA burst/capture approach in the STM32H7 platform layer. The implementation is generic - it works on any STM32H7 board by dynamically discovering port/pin groupings from `timer_io_channels[]` at init time. The **ARK FPV** board is the test target.

Bitbang DShot uses a pacer timer to trigger DMA transfers to/from GPIO registers (BSRR for output, IDR for input), instead of using timer PWM channels with DMA burst to CCR registers. This decouples motor signal generation from timer channel hardware and enables simultaneous capture of all motors on a GPIO port (eliminating round-robin).

## Key Advantage Over Current Approach

The current BDShot implementation captures **one motor per timer per cycle** (round-robin) because each capture channel needs its own DMA stream. At 200Hz update rate, each motor gets RPM updates at only 50Hz (200/4).

Bitbang capture reads the entire GPIO IDR register on every sample, capturing **all motors on a port simultaneously**. Every cycle produces RPM data for every motor - a 4x improvement in per-motor RPM update rate.

## Architecture Overview

### Current (DMA Burst/Capture)
```
Timer (PWM mode) ──► CCR1-CCR4 (via DMA burst to DMAR) ──► Pin AF output
Timer (Capture mode) ──► CCR edge timestamps (via per-channel DMA) ──► one channel at a time
```

### Bitbang
```
Timer (counter mode) ──► Update event triggers DMA ──► writes BSRR words to GPIO port
Timer (counter mode) ──► Update event triggers DMA ──► reads IDR from GPIO port (all pins)
```

## Port Group Discovery

At init time, the implementation iterates `timer_io_channels[]` and groups channels by GPIO port. The port and pin are extracted from the existing `gpio_out` field using NuttX bit encoding:

```c
#include <stm32_gpio.h>  // GPIO_PORT_MASK, GPIO_PORT_SHIFT, GPIO_PIN_MASK, GPIO_PIN_SHIFT

// Extract port index and pin number from timer_io_channels[ch].gpio_out
uint8_t port_index = (gpio_out & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;  // 0=A, 1=B, ... 7=H, 8=I
uint8_t pin_number = (gpio_out & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;    // 0-15

// Get GPIO peripheral base address from NuttX lookup table
uint32_t gpio_base = g_gpiobase[port_index];  // e.g., STM32_GPIOH_BASE = 0x58021C00
```

### Port Group Init Algorithm

```c
// Pseudocode for up_dshot_init()
for (uint8_t ch = 0; ch < MAX_TIMER_IO_CHANNELS; ch++) {
    if (!(channel_mask & (1 << ch))) continue;

    uint32_t gpio_out = timer_io_channels[ch].gpio_out;
    uint8_t port_index = (gpio_out & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
    uint8_t pin_num = (gpio_out & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
    uint8_t timer_index = timer_io_channels[ch].timer_index;

    // Find or create port group for this GPIO port
    port_group_t *pg = find_or_create_port_group(port_index);
    pg->gpio_base = g_gpiobase[port_index];
    pg->pin_mask |= (1 << pin_num);
    pg->moder_mask |= (3 << (pin_num * 2));
    pg->moder_output |= (1 << (pin_num * 2));  // 01 = output mode
    pg->pin_numbers[pg->pin_count] = pin_num;
    pg->output_channels[pg->pin_count] = ch;
    pg->pin_count++;

    // Assign a pacer timer - use the first timer associated with any motor on this port.
    // On H7 with DMAMUX the timer just needs a TIMx_UP DMA mapping; any timer works.
    if (pg->timer_index == UINT8_MAX) {
        pg->timer_index = timer_index;
    }
}
```

This works on any board. Motors on the same GPIO port share a port group regardless of which timer they were originally assigned to.

### ARK FPV Example (test target)

On the ARK FPV, the above algorithm produces two port groups:

| Motor   | GPIO  | Pin | Port Group | Pacer Timer |
|---------|-------|-----|------------|-------------|
| FMU_CH1 | PI0   | 0   | Port I     | TIM5 (first timer seen on Port I) |
| FMU_CH2 | PH12  | 12  | Port H     | TIM5        |
| FMU_CH3 | PH11  | 11  | Port H     | TIM5        |
| FMU_CH4 | PH10  | 10  | Port H     | TIM5        |
| FMU_CH5 | PI5   | 5   | Port I     | (already assigned) |
| FMU_CH6 | PI6   | 6   | Port I     | (already assigned) |
| FMU_CH7 | PI7   | 7   | Port I     | (already assigned) |
| FMU_CH8 | PI2   | 2   | Port I     | (already assigned) |

- **Port H group**: 3 motors, pin_mask=0x1C00, pacer=TIM5
- **Port I group**: 5 motors, pin_mask=0xE5, pacer=TIM5 (first timer found; could also be TIM8)

Note: The pacer timer just needs a valid `dma_map_up` entry. The first timer encountered for a port group is used. Since DMAMUX on H7 can route any timer update event to any DMA stream, any timer with DMA configured works as a pacer.

## GPIO Registers (STM32H7)

```
Offset 0x00: MODER   - Mode register (2 bits/pin: 00=input, 01=output, 10=AF, 11=analog)
Offset 0x10: IDR     - Input data register (read-only, bits [15:0] = pin state)
Offset 0x14: ODR     - Output data register
Offset 0x18: BSRR    - Bit set/reset register (write-only, atomic)
                        Bits [15:0]  = SET (writing 1 sets pin HIGH)
                        Bits [31:16] = RESET (writing 1 sets pin LOW)
                        Writing 0 = no change
```

GPIO base addresses are looked up at runtime via `g_gpiobase[port_index]` (NuttX global array).

### MODER Manipulation

Pin masks and MODER masks are computed dynamically during port group init from the discovered pin numbers:

```c
// For each pin in the port group:
pg->pin_mask |= (1 << pin_num);                 // for BSRR set/reset
pg->moder_mask |= (3 << (pin_num * 2));         // which MODER bits to touch
pg->moder_output |= (1 << (pin_num * 2));       // 01 = general purpose output
// moder_input is always 0 (clear the bits = input mode)
```

To switch to output: `modifyreg32(gpio_base + STM32_GPIO_MODER_OFFSET, pg->moder_mask, pg->moder_output)`
To switch to input:  `modifyreg32(gpio_base + STM32_GPIO_MODER_OFFSET, pg->moder_mask, 0)`

## DMA Configuration

The DMA request source (DMAMUX) stays the same as current - TIMx_UP triggers each DMA transfer. The only change is the **peripheral address** and **transfer direction**.

### Output DMA (Memory → GPIO BSRR)
```c
// Peripheral address is GPIO BSRR instead of TIM DMAR
px4_stm32_dmasetup(pg->dma_handle,
    pg->gpio_base + STM32_GPIO_BSRR_OFFSET,       // e.g., 0x58021C18 for Port H
    (uint32_t)bsrr_output_buffer[pg_index],
    SUBPERIODS_PER_BIT * DSHOT_FRAME_BITS,          // 20 * 16 = 320 transfers (+ 1 trailing zero)
    DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS |
    DMA_SCR_MINC | DMA_SCR_DIR_M2P | DMA_SCR_TCIE);
```

### Input DMA (GPIO IDR → Memory)
```c
px4_stm32_dmasetup(pg->dma_handle,
    pg->gpio_base + STM32_GPIO_IDR_OFFSET,          // e.g., 0x58021C10 for Port H
    (uint32_t)idr_capture_buffer[pg_index],
    CAPTURE_SAMPLE_COUNT,                            // enough samples for GCR frame (~512)
    DMA_SCR_PRIHI | DMA_SCR_MSIZE_16BITS | DMA_SCR_PSIZE_16BITS |
    DMA_SCR_MINC | DMA_SCR_DIR_P2M | DMA_SCR_TCIE);
```

### DMAMUX Routing (unchanged)

Each port group uses its pacer timer's `dma_map_up` for DMAMUX routing:
```c
pg->dma_handle = stm32_dmachannel(io_timers[pg->timer_index].dshot.dma_map_up);
```

The DMAMUX determines **when** the DMA fires (on timer update events). The DMA stream registers determine **what** gets transferred (BSRR/IDR addresses). These are independent - the DMA request source and transfer target don't need to be the same peripheral.

## Timer Configuration

Each port group's pacer timer is configured as a simple upcounter. No PWM output, no capture-compare - just generate periodic update events.

```c
// Same prescaler as current DShot implementation
rPSC(timer) = ((timer_clock_freq / dshot_pwm_freq) / DSHOT_MOTOR_PWM_BIT_WIDTH) - 1;
rARR(timer) = DSHOT_MOTOR_PWM_BIT_WIDTH - 1;  // 19 (20 counts per sub-period)
// Enable update DMA request
rDIER(timer) |= ATIM_DIER_UDE;
// Enable counter
rCR1(timer) |= GTIM_CR1_CEN;
```

With `DSHOT_MOTOR_PWM_BIT_WIDTH = 20`, each timer update fires DMA once per sub-period. One DShot bit = 20 sub-periods.

## Output Buffer Layout (BSRR Words)

Each DShot frame is 16 bits. Each bit has 20 sub-periods. The buffer contains one 32-bit BSRR word per sub-period.

**DShot bit waveform (sub-periods 0-19):**
- Bit '1' (70% duty): SET at sub-period 0, RESET at sub-period 14, zeros elsewhere
- Bit '0' (35% duty): SET at sub-period 0, RESET at sub-period 7, zeros elsewhere

**Buffer structure per port:**
```
[bit0_subperiod0] [bit0_subperiod1] ... [bit0_subperiod19]
[bit1_subperiod0] [bit1_subperiod1] ... [bit1_subperiod19]
...
[bit15_subperiod0] [bit15_subperiod1] ... [bit15_subperiod19]
[trailing_zero]  ← ensures all pins return LOW at frame end
```

Total: (16 * 20) + 1 = 321 BSRR words = 1284 bytes per port.

**Encoding example** (3 motors on pins 10, 11, 12 of some port, sending bits 1, 0, 1):
```
Sub-period 0:  BSRR = (1<<10) | (1<<11) | (1<<12)  = 0x00001C00  (SET all three)
Sub-period 7:  BSRR = (1<<(11+16))                  = 0x00080000  (RESET pin11 only - it's '0')
Sub-period 14: BSRR = (1<<(10+16)) | (1<<(12+16))   = 0x14000000  (RESET pin10, pin12 - they're '1')
All other sub-periods: BSRR = 0x00000000 (no change)
```

### Motor Data Set Function

The port group index and pin number for each output channel are determined at init time and stored in a lookup table. No per-frame discovery needed.

```c
// Populated during up_dshot_init() from timer_io_channels[ch].gpio_out
static uint8_t _channel_to_port_group[MAX_TIMER_IO_CHANNELS];
static uint8_t _channel_to_pin[MAX_TIMER_IO_CHANNELS];

void dshot_motor_data_set(uint8_t output_channel, uint16_t packet)
{
    uint8_t pg_index = _channel_to_port_group[output_channel];
    uint8_t pin_num = _channel_to_pin[output_channel];
    uint32_t set_mask = (1 << pin_num);
    uint32_t reset_mask = (1 << (pin_num + 16));

    uint32_t *buffer = bsrr_output_buffer[pg_index];

    for (int bit = 0; bit < 16; bit++) {
        int base = bit * SUBPERIODS_PER_BIT;
        bool is_one = (packet >> (15 - bit)) & 1;  // MSB first
        int reset_point = is_one ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;

        buffer[base + 0] |= set_mask;               // SET at start of bit
        buffer[base + reset_point] |= reset_mask;    // RESET at duty cycle end
    }
}
```

Note: Buffer must be zeroed before building each frame, then all motors OR their masks into the shared per-port buffer.

## Capture Buffer Layout (IDR Samples)

After transmitting, GPIO pins switch to input mode. DMA samples the IDR register at the same timer rate (20 sub-periods per DShot bit period).

**GCR response timing:**
- GCR bit rate = DShot bit rate * 5/4 (750kHz for DShot600)
- GCR bit period = 1.333us
- Sample period = 83.3ns (at DShot600 with 20 sub-periods)
- Samples per GCR bit = ~16 (heavy oversampling, good for edge detection)
- GCR frame = 21 bits = ~336 samples minimum
- Use 400-512 samples to be safe

**Buffer:** Array of uint16_t, one per IDR sample.
```
[idr_sample_0] [idr_sample_1] ... [idr_sample_N]
```

Each sample contains all 16 pins of the port. Extract per-motor bitstream by masking:
```c
bool pin_state = (idr_sample >> pin_num) & 1;
```

### Capture Processing

Convert IDR samples to edge intervals, then reuse existing GCR decoder:

```c
void process_bitbang_capture(uint8_t port_group, uint8_t pin_num, uint8_t output_channel)
{
    uint16_t *buffer = idr_capture_buffer[port_group];
    uint32_t intervals[32];
    unsigned interval_count = 0;

    // Extract edges from IDR samples
    bool prev_state = (buffer[0] >> pin_num) & 1;
    unsigned last_edge = 0;

    for (unsigned i = 1; i < CAPTURE_SAMPLE_COUNT; i++) {
        bool state = (buffer[i] >> pin_num) & 1;
        if (state != prev_state) {
            // Edge detected
            if (last_edge > 0 && interval_count < 32) {
                intervals[interval_count++] = i - last_edge;
            }
            last_edge = i;
            prev_state = state;
        }
    }

    // Now decode using same GCR logic as existing code
    // intervals[] contains sample counts between edges
    // (equivalent to CCR timestamp differences in current impl)
}
```

The existing `convert_edge_intervals_to_bitstream()` logic (adaptive base interval, bit counting, RLL/GCR decode, CRC check) can be reused with minor adaptation since the interval units are now sample counts instead of timer ticks. The ratio math is identical.

## Sequence of Operations

### Per-cycle flow (called from `up_dshot_trigger()`):

```
1. TRANSMIT PHASE
   a. Zero the BSRR output buffers for all port groups
   b. Build BSRR buffers (call dshot_motor_data_set for each motor)
      - Each motor ORs its pin masks into its port group's shared buffer
   c. Configure GPIO pins as output (MODER = 01) for each port group
   d. Flush dcache on output buffers
   e. For each port group:
      - Allocate DMA via stm32_dmachannel(io_timers[pg->timer_index].dshot.dma_map_up)
      - Setup DMA: memory→BSRR, 32-bit, 321 transfers
      - Enable timer UDE (update DMA request)
      - Start DMA with burst_finished_callback
      - Enable timer counter
   f. ~27us later (16 bits × 20 sub-periods × 83.3ns), DMA complete fires

2. TRANSITION PHASE (in burst_finished_callback, per port group)
   a. Stop DMA, free DMA handle
   b. Disable timer
   c. Switch GPIO pins to input mode (MODER = 00)
   d. Schedule capture start via hrt_call_after(30us)

3. CAPTURE PHASE (in capture_start_callback, per port group)
   a. Zero capture buffer for this port group
   b. Flush dcache
   c. Allocate DMA via stm32_dmachannel(dma_map_up) [reuse same DMAMUX route]
   d. Setup DMA: IDR→memory, 16-bit, 512 transfers
   e. Enable timer UDE, start DMA, enable timer
   f. Schedule capture_complete via hrt_call_after(frame_time + margin)

4. PROCESSING PHASE (in capture_complete_callback, per port group)
   a. Stop DMA, free DMA handle
   b. Disable timer
   c. Invalidate dcache on capture buffer
   d. For EACH motor on the port (not round-robin!):
      - Extract per-pin bitstream from IDR samples using pin_numbers[]
      - Detect edges, compute intervals
      - Decode GCR → eRPM/EDT
   e. Switch GPIO back to output mode (MODER = 01)
   f. Mark port group cycle complete

```

## File Structure

### New file: `platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot_bitbang.c`

Implements the same `up_dshot_*` API as `dshot.c`:
- `up_dshot_init()` - Initialize port groups, allocate DMA, configure timers
- `up_dshot_trigger()` - Start the TX→wait→RX→process cycle
- `dshot_motor_data_set()` - Build BSRR words into port group buffers
- `up_dshot_arm()` - Enable/disable GPIO output
- `up_bdshot_get_erpm()` - Return captured eRPM (unchanged)
- `up_bdshot_channel_online()` - Return online status (unchanged)
- `up_bdshot_status()` - Print diagnostics

### New/modified: `platforms/nuttx/src/px4/stm/stm32_common/dshot/CMakeLists.txt`

Add build option to select `dshot.c` vs `dshot_bitbang.c`.

### Board-level config

No board-specific changes needed. The bitbang implementation reads the same `io_timers[]` and `timer_io_channels[]` structures that every board already defines. It extracts GPIO port/pin from `timer_io_channels[].gpio_out` (which `initIOTimerChannel()` already populates with `getGPIOPort(pin.port) | getGPIOPin(pin.pin)`) and the DMA mapping from `io_timers[].dshot.dma_map_up`.

A board opts into bitbang mode via a build flag or Kconfig option that selects `dshot_bitbang.c` instead of `dshot.c`.

### What stays the same
- `drv_dshot.h` API - no changes
- `DShot.cpp` / `DShot.h` (the driver module) - no changes
- `DShotTelemetry.cpp` / `DShotTelemetry.h` - no changes
- GCR decoding logic - copied/adapted from dshot.c but the core algorithm is identical
- All higher-level consumers of eRPM data

## Data Structures

```c
#define MAX_PORT_GROUPS 4  // Max distinct GPIO ports with motor pins (typically 1-3)
#define MAX_PINS_PER_GROUP 8

// Port group: one per GPIO port that has motor pins. Built dynamically at init.
typedef struct port_group_t {
    uint32_t gpio_base;                          // GPIO port base address (from g_gpiobase[])
    uint8_t  port_index;                         // NuttX port index (0=A, 7=H, 8=I, etc.)
    uint32_t pin_mask;                           // Bitmask of motor pins on this port
    uint32_t moder_mask;                         // MODER register mask for mode switching
    uint32_t moder_output;                       // MODER value for output mode (01 per pin)
    uint8_t  pin_count;                          // Number of motor pins on this port
    uint8_t  pin_numbers[MAX_PINS_PER_GROUP];    // Pin numbers (0-15)
    uint8_t  output_channels[MAX_PINS_PER_GROUP]; // Corresponding output channel indices
    uint8_t  timer_index;                        // Pacer timer index (into io_timers[])
    DMA_HANDLE dma_handle;                       // DMA stream handle (reused between TX/RX)
    bool     bidirectional;                      // BDShot enabled for this group
    bool     cycle_complete;                     // Ready for next trigger
} port_group_t;

static port_group_t _port_groups[MAX_PORT_GROUPS] = {};
static uint8_t _num_port_groups = 0;

// Per-channel lookup tables (populated at init from timer_io_channels[].gpio_out)
static uint8_t _channel_to_port_group[MAX_TIMER_IO_CHANNELS];
static uint8_t _channel_to_pin[MAX_TIMER_IO_CHANNELS];
```

## DMA Resource Usage Comparison

### Current (DMA burst + capture)
- **Output**: 1 DMA stream per timer = N_timers streams
- **Capture**: 1 DMA stream per captured channel (round-robin, 1 at a time per timer)
- DMA streams are freed/reallocated between burst and capture phases

### Bitbang
- **Output**: 1 DMA stream per port group = N_ports streams
- **Capture**: 1 DMA stream per port group (same streams, reconfigured)
- DMA streams freed/reallocated between phases (same pattern as current)

DMA resource usage is equivalent (typically N_timers == N_ports or close). The DMAMUX routing uses the same timer update event mappings.

## Memory Usage

### Current
- Output: ~272 bytes per timer (4 channels * 17 words * 4 bytes, cache-aligned)
- Capture: ~256 bytes per timer (4 channels * 32 samples * 2 bytes)

### Bitbang
- Output: ~1284 bytes per port group (321 words * 4 bytes, cache-aligned)
- Capture: ~1024 bytes per port group (512 samples * 2 bytes, cache-aligned)
- Scales with number of port groups, not number of motors
- Higher per-group cost, but acceptable for H7 (1MB+ SRAM)

## Open Questions / Risks

1. **GPIO output speed**: BSRR writes through AHB4 bus on H7. At 83.3ns per DMA transfer (12MHz), is there enough bus bandwidth? Should be fine - AHB4 runs at 240MHz, and BSRR writes are single-cycle.

2. **DMA priority vs other peripherals**: SPI1 (IMU) is on DMA1. Timer DMA is already high priority. No change expected.

3. **GPIO input threshold**: When reading IDR for bidir, the input threshold depends on GPIO input configuration (Schmitt trigger, pull-up/down). The current capture approach uses timer input capture which has different input characteristics. May need to configure GPIO pull-up to match idle-high behavior.

4. **Signal integrity**: Timer PWM output has precise edge timing locked to the timer clock. BSRR writes via DMA may have slight jitter due to bus arbitration. This is likely negligible (< 10ns on H7) but should be verified.

5. **OTYPER configuration**: For bidirectional DShot, the pin needs to be open-drain (or at least not actively driven when in input mode). The MODER switch to input (00) handles this, but verify that switching MODER during/after a frame doesn't cause glitches.

6. **Cache coherency**: BSRR output buffers need dcache clean before DMA. IDR capture buffers need dcache invalidate after DMA. Same pattern as current code.

## Implementation Order

1. **Port group initialization**: Parse `timer_io_channels[]` to build `port_group_t` structures grouped by GPIO port instead of timer
2. **Output-only DShot**: Implement BSRR buffer building and DMA output. Verify with logic analyzer that DShot frames are correct
3. **Bidirectional capture**: Implement IDR sampling and per-pin edge extraction. Verify GCR decode produces valid eRPM
4. **Integration**: Wire up to the existing `up_dshot_*` API so `DShot.cpp` works unchanged
5. **Testing**: Compare eRPM data quality (CRC error rate, update rate) vs current implementation

## Reference: Betaflight Bitbang Implementation

- [Betaflight dshot_bitbang.c](https://github.com/betaflight/betaflight/blob/master/src/platform/STM32/dshot_bitbang.c)
- [Betaflight dshot_bitbang_impl.h](https://github.com/betaflight/betaflight/blob/master/src/main/drivers/dshot_bitbang_impl.h)
- [Original GPIO banging PR #7446](https://github.com/betaflight/betaflight/pull/7446)
- [Bidirectional bitbang PR #8779](https://github.com/betaflight/betaflight/pull/8779)

## Reference: Current PX4 Implementation Files

- DShot platform layer: `platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c`
- IO timer layer: `platforms/nuttx/src/px4/stm/stm32_common/io_pins/io_timer.c`
- IO timer struct defs: `platforms/nuttx/src/px4/stm/stm32_common/include/px4_arch/io_timer.h`
- GPIO/pin helpers: `platforms/nuttx/src/px4/stm/stm32_common/include/px4_arch/hw_description.h`
- H7 timer/DMA descriptions: `platforms/nuttx/src/px4/stm/stm32h7/include/px4_arch/io_timer_hw_description.h`
- H7 GPIO register defs: NuttX `arch/arm/src/stm32h7/stm32_gpio.h` (GPIO_PORT_MASK, GPIO_PIN_MASK, g_gpiobase[])
- H7 GPIO HW registers: NuttX `arch/arm/src/stm32h7/hardware/stm32h7x3xx_gpio.h` (STM32_GPIO_MODER_OFFSET, etc.)
- DShot API header: `src/drivers/drv_dshot.h`
- DShot driver module: `src/drivers/dshot/DShot.cpp`

## Reference: Test Target (ARK FPV)

- Timer config: `boards/ark/fpv/src/timer_config.cpp`
- DMA map: `boards/ark/fpv/nuttx-config/include/board_dma_map.h`
- Board config: `boards/ark/fpv/src/board_config.h`
