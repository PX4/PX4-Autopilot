# BDShot Timing Jitter Analyzer - Technical Specification

## Overview

Build a Python command-line tool to analyze BDShot (Bidirectional DShot) signal timing from Saleae Logic Analyzer CSV exports. The tool measures scheduling consistency and jitter in PX4 flight controller DShot output timing.

## Background

### DShot Protocol Basics

DShot is a digital protocol for communication between flight controllers (FC) and ESCs (Electronic Speed Controllers). Each frame consists of 16 bits encoding an 11-bit throttle value, 1 telemetry request bit, and 4-bit CRC.

**DShot Variants and Timing:**
| Variant   | Bit Period | Frame Duration (16 bits) |
|-----------|------------|--------------------------|
| DShot150  | 6.67µs     | ~107µs                   |
| DShot300  | 3.33µs     | ~53µs                    |
| DShot600  | 1.67µs     | ~27µs                    |
| DShot1200 | 0.83µs     | ~13µs                    |

Each bit is encoded as a fixed-length pulse where:
- Logic 1: ~75% duty cycle (high for 3/4 of bit period)
- Logic 0: ~37.5% duty cycle (high for 3/8 of bit period)

### BDShot (Bidirectional DShot)

BDShot extends DShot to receive telemetry data (eRPM) back from the ESC on the same signal wire. The ESC responds after each command frame with a 21-bit GCR (Group Code Recording) encoded response.

**BDShot Transaction Structure:**
```
         Command Frame (FC→ESC)      Response Frame (ESC→FC)
         ┌─────────────────────┐     ┌──────────────────────────┐
IDLE ────┤  16 bits DShot      ├─────┤  21 bits GCR telemetry   ├──── IDLE
  HIGH   │  (FC drives line)   │ TT  │  (ESC drives line)       │    HIGH
         └─────────────────────┘     └──────────────────────────┘
                                  ^
                             Turnaround time
                             (line direction switch)
```

**Approximate BDShot300 Timing:**
- Command frame: ~53µs (16 bits × 3.33µs)
- Turnaround gap: ~25-30µs (varies by ESC)
- Response frame: ~50-70µs (21 GCR bits)
- Total transaction: ~130-150µs

The idle state is HIGH (pulled up). Frames begin with a falling edge and end when the line returns to idle HIGH.

## Hardware Setup Context

### PX4 Timer Architecture

The user is running PX4 firmware on a flight controller with the following DShot output configuration:

- **Timer 1:** Drives motor outputs 1-4
- **Timer 2:** Drives motor outputs 5-8
- **DMA Strategy:** Single DMA peripheral handles both timers sequentially
  - First: Burst capture for Timer 1 (all 4 channels)
  - Then: Burst capture for Timer 2 (all 4 channels)

This sequential triggering means there should be a consistent gap between when Timer 1's transaction completes and Timer 2's transaction begins.

### Capture Setup

- **Saleae Logic Analyzer:** Capturing two BDShot channels
- **Channel A:** Motor output 1 (Timer 1)
- **Channel B:** Motor output 5 (Timer 2)
- **Sample Rate:** Recommend 24+ MS/s for DShot300
- **Capture Duration:** 1-2 minutes typical
- **PX4 Loop Rate:** 800 Hz (1250µs period)
- **DShot Variant:** DShot300

## Analysis Requirements

### Measurements Needed

#### 1. Per-Channel Frame Interval Statistics

For each channel independently, measure the time between consecutive BDShot transactions (command start to next command start).

```
CH1: ──┬═══TRANSACTION═══┬────────────────────────┬═══TRANSACTION═══┬──
       │                 │                        │                 │
       t1                                         t2
       
       Frame Interval = t2 - t1 (expected: 1250µs at 800Hz loop rate)
```

**Statistics to compute:**
- Minimum interval
- Maximum interval
- Mean interval
- Standard deviation
- Jitter (deviation from expected interval)

#### 2. Inter-Channel Gap Statistics

Measure the time from Channel A (CH1) transaction END to Channel B (CH5) transaction START. This quantifies the sequential timer scheduling overhead.

```
CH1: ──┬═══TRANSACTION═══┬─────────────────────────────────────────────
       │                 │
       │                 └─ CH1 Response End (t_ch1_end)
       │
CH5: ──┴─────────────────────┬═══TRANSACTION═══┬───────────────────────
                             │
                             └─ CH5 Command Start (t_ch5_start)
                             
       Inter-Channel Gap = t_ch5_start - t_ch1_end
```

**Statistics to compute:**
- Minimum gap
- Maximum gap  
- Mean gap
- Standard deviation

#### 3. Visualization

Generate timing histograms for:
- Channel A frame intervals
- Channel B frame intervals
- Inter-channel gaps

## Input Format

### Saleae Logic 2 CSV Export

The tool should accept CSV exports from Saleae Logic 2's "Export Raw Data" feature. The format for digital channels is edge-based (only transitions are recorded):

**Single Channel Export:**
```csv
Time [s],Channel 0
0.000000000,1
0.000023456,0
0.000025890,1
...
```

**Multi-Channel Export:**
```csv
Time [s],Channel 0,Channel 1
0.000000000,1,1
0.000023456,0,1
0.000025890,1,1
0.000025900,1,0
...
```

**Key characteristics:**
- Timestamps in seconds (floating point, ~9 decimal places)
- Only rows where at least one channel changed state
- Channel values: 1 = HIGH, 0 = LOW
- Idle state for BDShot is HIGH (1)

## Implementation Specification

### Command-Line Interface

```bash
# Basic usage
python bdshot_analyzer.py capture.csv

# With options
python bdshot_analyzer.py capture.csv \
    --dshot-rate 300 \
    --loop-rate 800 \
    --channel-a 0 \
    --channel-b 1 \
    --output-dir ./results
```

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `input_file` | required | Path to Saleae CSV export |
| `--dshot-rate` | 300 | DShot variant (150/300/600/1200) |
| `--loop-rate` | 800 | Expected loop rate in Hz |
| `--channel-a` | 0 | CSV column index for first channel |
| `--channel-b` | 1 | CSV column index for second channel |
| `--output-dir` | `./` | Directory for output files |
| `--no-plots` | false | Disable histogram generation |

### Algorithm

#### Step 1: Parse CSV and Extract Edges

Read the CSV and build edge lists for each channel:
```python
edges_a = [(timestamp, new_state), ...]  # Channel A edges
edges_b = [(timestamp, new_state), ...]  # Channel B edges
```

#### Step 2: Detect BDShot Transactions

A BDShot transaction consists of a command frame followed by a response frame. We can detect transaction boundaries by looking for idle gaps.

**Transaction Detection Logic:**

1. Find all falling edges (HIGH→LOW transitions) - these are potential frame starts
2. Find all rising edges (LOW→HIGH transitions) - these are potential frame ends
3. Group edges into transactions:
   - A transaction STARTS with a falling edge after a long idle period (> threshold)
   - A transaction ENDS when the line returns HIGH and stays HIGH for > threshold

**Idle Gap Threshold:**

For DShot300, bit period is 3.33µs. Use a gap threshold of ~20µs to detect transaction boundaries (much longer than any inter-bit gap but shorter than inter-frame idle).

```python
@dataclass
class BDShotTransaction:
    command_start: float    # Timestamp of first falling edge
    command_end: float      # Timestamp when command frame ends (estimate)
    response_start: float   # Timestamp when response frame begins (estimate)  
    response_end: float     # Timestamp of final rising edge before idle
    
    @property
    def start(self) -> float:
        return self.command_start
    
    @property
    def end(self) -> float:
        return self.response_end
```

**Simplified approach:** If precise command/response boundary detection is difficult, we can treat each transaction as:
- `start`: First falling edge after idle
- `end`: Last rising edge before next idle

This is sufficient for the timing measurements we need.

#### Step 3: Compute Per-Channel Statistics

For each channel, compute frame intervals:
```python
intervals_a = [t[i+1].start - t[i].start for i in range(len(transactions_a) - 1)]
```

Calculate statistics:
- `min`, `max`, `mean`, `std` using numpy
- Expected interval = 1_000_000 / loop_rate (in µs)
- Jitter = measured - expected

#### Step 4: Compute Inter-Channel Gap Statistics

Match Channel A transactions with the next Channel B transaction:
```python
gaps = []
for tx_a in transactions_a:
    # Find first CH_B transaction that starts after this CH_A ends
    tx_b = find_next_transaction(transactions_b, after=tx_a.end)
    if tx_b and is_same_cycle(tx_a, tx_b):  # within same loop iteration
        gaps.append(tx_b.start - tx_a.end)
```

**Cycle Matching Logic:**

Since CH1 and CH5 fire sequentially within the same loop iteration, we need to match them correctly. The gap should be small (tens of µs), not a full loop period. If `tx_b.start - tx_a.end` is close to the expected loop period, we've matched wrong transactions.

#### Step 5: Generate Output

**Console Output:**
```
BDShot Timing Analysis
======================
Input: capture.csv
DShot Rate: 300 (bit period: 3.33µs)
Expected Loop Rate: 800 Hz (1250.00µs interval)
Capture Duration: 120.34s
Transactions Detected: CH_A=96268, CH_B=96268

Channel A (Motor 1) Frame Intervals
───────────────────────────────────
  Count:     96267
  Min:       1247.32 µs  (-2.68 µs from nominal)
  Max:       1254.21 µs  (+4.21 µs from nominal)
  Mean:      1250.01 µs  (+0.01 µs from nominal)
  Std Dev:   1.43 µs
  
Channel B (Motor 5) Frame Intervals  
───────────────────────────────────
  Count:     96267
  Min:       1248.01 µs  (-1.99 µs from nominal)
  Max:       1253.87 µs  (+3.87 µs from nominal)
  Mean:      1250.02 µs  (+0.02 µs from nominal)
  Std Dev:   1.21 µs

Inter-Channel Gap (CH_A End → CH_B Start)
─────────────────────────────────────────
  Count:     96267
  Min:       12.34 µs
  Max:       18.76 µs
  Mean:      15.23 µs
  Std Dev:   0.87 µs

Histograms saved to: ./results/
```

**Histogram Plots:**

Generate three histogram plots using matplotlib:

1. `channel_a_intervals.png` - Channel A frame interval distribution
2. `channel_b_intervals.png` - Channel B frame interval distribution  
3. `inter_channel_gaps.png` - Inter-channel gap distribution

**Histogram Requirements:**
- Title indicating what's being measured
- X-axis: Time in microseconds
- Y-axis: Count (frequency)
- Vertical line showing expected/mean value
- Statistics annotation (min/max/mean/std) in legend or text box
- Reasonable bin count (~50-100 bins)
- Save as PNG at 150+ DPI

### Error Handling

- Validate CSV format and provide clear error messages
- Handle edge cases:
  - Incomplete transactions at start/end of capture
  - Corrupted/noisy signals (transactions with wrong number of edges)
  - Missing channel data
- Report number of malformed/skipped transactions

### Dependencies

```
numpy
matplotlib
```

Standard library only otherwise (csv, argparse, dataclasses, pathlib, etc.)

## Example Usage Session

```bash
# Capture 2 minutes of BDShot data in Saleae Logic 2
# Export as CSV: File → Export Raw Data → CSV

# Run analysis
$ python bdshot_analyzer.py ~/captures/bdshot_test.csv --loop-rate 800 --dshot-rate 300

BDShot Timing Analysis
======================
Input: /home/user/captures/bdshot_test.csv
...

# Check generated histograms
$ ls *.png
channel_a_intervals.png
channel_b_intervals.png  
inter_channel_gaps.png
```

## Deliverables

1. **`bdshot_analyzer.py`** - Main analysis script
2. **`README.md`** - Usage instructions and examples

## Notes for Implementation

- Use microseconds (µs) for all timing displays (convert from seconds in CSV)
- Handle large files efficiently (100k+ transactions for 2-minute capture)
- The inter-channel gap measurement is the most interesting metric - it shows the overhead of sequential DMA scheduling
- Consider adding a `--verbose` flag for debugging transaction detection
- Consider outputting raw statistics to JSON for further processing
