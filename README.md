# LSA08 PID Line-Following Robot

A PID-controlled line-following robot using the **LSA08 sensor array** (6 sensors) on an **Arduino Uno**. The robot detects and handles multiple track patterns — straight lines, curves, cross junctions, T-junctions, sharp 90° turns, circles, gaps, and horizontal bars — using a **Finite State Machine (FSM)** that switches between specialised behaviours.

---

## Table of Contents

- [Hardware Setup](#hardware-setup)
- [File Structure](#file-structure)
- [Dependencies](#dependencies)
- [How It Works](#how-it-works)
  - [Sensor Reading](#1-sensor-reading-lsa08)
  - [Line Position Calculation](#2-line-position-calculation)
  - [PID Controller](#3-pid-controller)
  - [Pattern Detection](#4-pattern-detection)
  - [Finite State Machine](#5-finite-state-machine-fsm)
- [Pattern Detection Details](#pattern-detection-details)
- [Tuning Guide](#tuning-guide)
- [Serial Monitor Output](#serial-monitor-output)
- [Build & Upload](#build--upload)

---

## Hardware Setup

| Component       | Arduino Pin | Notes                          |
|-----------------|-------------|--------------------------------|
| LSA08 TX        | D4          | SoftwareSerial RX              |
| LSA08 RX        | D7          | SoftwareSerial TX              |
| Motor A — IN1   | D10         | Left motor direction           |
| Motor A — IN2   | D8          | Left motor direction           |
| Motor A — PWM   | D5          | Left motor speed (PWM capable) |
| Motor B — IN1   | D6          | Right motor direction          |
| Motor B — IN2   | D9          | Right motor direction          |
| Motor B — PWM   | D7          | Right motor speed (PWM capable)|

> The motor driver is assumed to be a TB6612FNG or L298N-style H-bridge with `IN1/IN2` for direction and a `PWM` pin for speed.

---

## File Structure

```
lsa08_PID/
├── lsa08_PID.ino    # Main sketch — setup, loop, FSM, PID control
├── Motors.h         # Motor pin definitions & motor control functions
├── Patterns.h       # Pattern detection functions (cross, T-junction, sharp turns, circles, etc.)
└── README.md        # This file
```

---

## Dependencies

| Library          | Source                        | Notes                              |
|------------------|-------------------------------|------------------------------------|
| `SoftwareSerial` | Bundled with Arduino AVR core | Used for LSA08 serial communication|
| `Arduino.h`      | Arduino framework             | Core functions (pinMode, etc.)     |

**No external libraries are needed.** Everything runs on the stock Arduino AVR toolchain.

### Installing with arduino-cli

```bash
# Install arduino-cli (if not already installed)
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh

# Install the AVR core (includes SoftwareSerial)
arduino-cli core update-index
arduino-cli core install arduino:avr
```

---

## How It Works

### 1. Sensor Reading (LSA08)

The LSA08 sensor array sends data packets over serial at **9600 baud**. Each packet:

```
[0x0D] [header] [s1] [s2] [s3] [s4] [s5] [s6] [checksum]
```

- `0x0D` is the start byte
- `s1` through `s6` are 6 sensor readings (0–255 each)
- **Low value** (near 0) = sensor over **white** surface (no line)
- **High value** (near 255) = sensor over **black** line

The function `readLsaSensorData()` parses this stream byte-by-byte using a simple state machine. It returns `true` only when a complete valid frame is available.

Sensor layout (looking from behind the robot):

```
  LEFT                          RIGHT
  [s1]  [s2]  [s3]  [s4]  [s5]  [s6]
   ←─────── line direction ──────────→
```

### 2. Line Position Calculation

Each sensor has a fixed **weight** representing its physical position:

```
SENSOR_WEIGHTS = { -5, -3, -1, +1, +3, +5 }
                   s1   s2   s3   s4   s5   s6
```

The **weighted average** gives a single floating-point position:

```
position = Σ(sensor[i] × weight[i]) / Σ(sensor[i])
```

- **Negative position** → line is to the **left**
- **Positive position** → line is to the **right**
- **Zero** → line is centred

If all sensors read 0, the function returns `false` (line lost).

### 3. PID Controller

The PID controller produces a **correction** value applied differentially to the two motors:

```
error       = SETPOINT(0) - position
P_term      = Kp × error
I_term      = Ki × ∫error·dt        (clamped to ±100)
D_term      = Kd × d(error)/dt

raw         = P_term + I_term + D_term
correction  = SMOOTHING × raw + (1 − SMOOTHING) × last_correction

left_motor  = BASE_SPEED + correction
right_motor = BASE_SPEED − correction
```

**Key design choices:**

| Parameter  | Value  | Purpose                                                       |
|------------|--------|---------------------------------------------------------------|
| `Kp`       | 400    | Main steering response — how aggressively the bot corrects    |
| `Ki`       | 0      | Integral term disabled (prevents windup on tight tracks)      |
| `Kd`       | 0.1    | Dampens oscillation — reacts to rate of error change          |
| `SMOOTHING`| 0.5    | Exponential moving average on correction (reduces jitter)     |
| `BASE_SPEED`| 60    | Forward speed when error is zero (PWM 0–255)                  |

The PID function is **parameterised** — different states can call it with different `Kp` and `BASE_SPEED` values:

| State        | Kp Used | Speed Used | Why                                          |
|--------------|---------|------------|----------------------------------------------|
| Normal       | 400     | 60         | Standard line following                      |
| Circle       | 500     | 50         | Higher gain to track the curve; slower to avoid drift-out |
| Sharp Turn   | 600     | 45         | (Used after re-acquire) Aggressive snap-back |

### 4. Pattern Detection

All pattern detectors live in `Patterns.h`. They analyse the **6-sensor array** (and optionally the **sensor history**) to classify what the robot is seeing.

#### Thresholds

```c
LINE_THRESHOLD      = 50    // sensor must read > 50 to count as "seeing the line"
CROSS_THRESHOLD     = 5     // >= 5 sensors active = cross or horizontal bar
FULL_HIT_THRESHOLD  = 150   // all sensors > 150 = solid horizontal bar
EDGE_THRESHOLD      = 100   // edge sensor "strongly lit" for sharp turn detection
SHARP_TURN_INNER    = 30    // opposite-side sensors must be < 30 (dark)
```

### 5. Finite State Machine (FSM)

The robot operates as a state machine. Transitions happen **only from `STATE_FOLLOWING_LINE`** (except GAP recovery). This prevents conflicting state changes.

```
                    ┌──────────────────────────────┐
                    │     STATE_FOLLOWING_LINE      │
                    │     (normal PID control)      │
                    └──┬───┬───┬───┬───┬───┬───────┘
                       │   │   │   │   │   │
          ┌────────────┘   │   │   │   │   └─────────────┐
          ▼                ▼   ▼   │   ▼                 ▼
   ┌─────────────┐  ┌──────┐ ┌──┐ │ ┌───────────┐ ┌───────────┐
   │ SHARP_LEFT  │  │T_JUNC│ │CR│ │ │SHARP_RIGHT│ │  CIRCLE   │
   │ hard pivot  │  │drv300│ │OS│ │ │ hard pivot │ │boosted PID│
   │  ← then     │  │→line │ │S │ │ │  then →   │ │until break│
   │  re-acquire │  └──┬───┘ └┬─┘ │ │ re-acquire│ └─────┬─────┘
   └──────┬──────┘     │      │   │ └─────┬─────┘       │
          │            │      │   │       │              │
          └────────────┴──────┴───┼───────┴──────────────┘
                                  │            line re-acquired
                                  ▼            or timeout
                           ┌──────────┐
                           │   GAP    │
                           │ decay    │
                           │ correct. │
                           └────┬─────┘
                                │ > 3 seconds
                                ▼
                           ┌──────────┐
                           │ STOPPED  │
                           │ motors=0 │
                           └──────────┘
```

---

## Pattern Detection Details

### Straight Line / Gentle Curves

**No special detection needed** — handled entirely by PID. As the line drifts left or right, the weighted position changes, and the PID generates a proportional correction.

```
Sensor reading (centre line):
[low] [low] [HIGH] [HIGH] [low] [low]
 s1    s2    s3      s4    s5    s6

Position ≈ 0 → error ≈ 0 → drive straight
```

```
Sensor reading (line drifting right):
[low] [low] [low] [HIGH] [HIGH] [low]
 s1    s2    s3     s4     s5    s6

Position > 0 → PID steers left
```

---

### Cross Junction

**Detection:** 5 or more sensors read above `LINE_THRESHOLD` (50), meaning the bot has driven over an intersection where lines cross.

```
Sensor pattern:
[HIGH] [HIGH] [HIGH] [HIGH] [HIGH] [low/HIGH]
```

**Behaviour (STATE_CROSS):** Drive straight at `BASE_SPEED` for **300ms** to clear the junction, then reset PID and resume normal line following. This prevents the bot from mistakenly turning at the cross.

---

### Horizontal Bar

**Detection:** ALL 6 sensors read above `FULL_HIT_THRESHOLD` (150) — a solid black bar across the full width of the track.

```
Sensor pattern:
[HIGH] [HIGH] [HIGH] [HIGH] [HIGH] [HIGH]   (all > 150)
```

**Behaviour:** Stays in `STATE_FOLLOWING_LINE`, drives straight at `BASE_SPEED`. Skips PID and all other pattern checks for that frame to avoid false triggers.

---

### T-Junction

**Detection:** Both outer edges are lit, centre sensors are lit, at least 4 sensors active, but it is NOT a complete horizontal bar (which would mean all 6 are uniformly strong).

```
Sensor pattern (example — T with forward and right paths):
[HIGH] [med] [HIGH] [HIGH] [low] [HIGH]
  s1    s2     s3     s4    s5    s6
```

The key difference from a cross: a T-junction has both edges active with the centre, but typically not all 6 sensors uniformly strong like a horizontal bar.

**Behaviour (STATE_T_JUNCTION):** Drive straight at `BASE_SPEED` for **400ms** (slightly longer than a cross to clear the wider opening), then reset PID and resume. The robot goes straight through — if your track requires turning at T-junctions, adjust `T_JUNCTION_DRIVE_MS` or add logic to pick a direction.

---

### Sharp Left Turn (90°)

**Detection:** Left-side edge sensors (`s1`/`s2`) read strongly (> `EDGE_THRESHOLD` = 100), while the right side (`s5`/`s6`) AND centre (`s3`/`s4`) are dark.

```
Sensor pattern:
[HIGH] [HIGH] [low] [low] [dark] [dark]
  s1     s2    s3    s4    s5      s6
```

This means the line has gone sharply left and only the outer-left sensors can still see it.

**Behaviour (STATE_SHARP_LEFT):**
1. **Pivot phase** (first 350ms): Left motor runs **backward**, right motor runs **forward** → robot rotates on the spot to the left
2. **Re-acquire phase** (after 350ms): Check if any sensors see the line
   - If YES → reset PID, resume `STATE_FOLLOWING_LINE`
   - If NO → keep pivoting at 70% power (extended search)
3. **Safety timeout** at 1.5 seconds → force return to line following

---

### Sharp Right Turn (90°)

Mirror of sharp left:

```
Sensor pattern:
[dark] [dark] [low] [low] [HIGH] [HIGH]
  s1     s2    s3    s4     s5     s6
```

**Behaviour (STATE_SHARP_RIGHT):** Same logic as sharp left but pivoting in the opposite direction (left motor forward, right motor backward).

---

### Circle (Sustained Curvature)

**Detection:** Uses the **sensor history buffer** — a ring buffer of the last 25 sensor readings. The detector calculates the weighted position for each historical frame and checks if all 25 positions are consistently biased to one side:

- All positions < `-1.5` → **circling left** (returns `-1`)
- All positions > `+1.5` → **circling right** (returns `+1`)
- Any break in the streak → **not a circle** (returns `0`)

This is fundamentally different from other patterns — it's not about what the sensors see *right now*, but about the **trend over time**.

**Behaviour (STATE_CIRCLE):**
- Runs PID with **boosted Kp (500)** and **reduced speed (50)** for tighter curve tracking
- Re-checks the curvature streak every frame
- When the streak breaks (the bot exits the circular arc), resets PID and returns to normal following
- Debug prints `[PAT] Circle LEFT/RIGHT` on entry and `[PAT] Circle END` on exit

---

### Gap / Discontinuous Line

**Detection:** 5 or more sensors read below `LINE_THRESHOLD` — the robot has completely lost the line.

```
Sensor pattern:
[dark] [dark] [dark] [dark] [dark] [dark]   (5+ below 50)
```

**Behaviour (STATE_GAP):**
1. **Snapshot** the last PID correction at the moment the line is lost
2. **Decay phase** (first 600ms): Linearly decay the saved correction toward 0. This makes the bot continue in its last known direction, gradually straightening out
3. **Freeze phase** (600ms–3000ms): Drive straight at `BASE_SPEED` with zero correction, hoping the line reappears
4. **If line reappears** at any point → reset PID, resume `STATE_FOLLOWING_LINE`
5. **If 3 seconds pass** without finding the line → enter `STATE_STOPPED` (motors off)

```
Timeline:
 0ms          600ms               3000ms
  ├── decay ──┤── straight drive ──┤── STOP
  correction  correction = 0       motors = 0
  fading → 0
```

---

## Tuning Guide

### PID Tuning

| Parameter | What it does | If too low | If too high |
|-----------|-------------|------------|-------------|
| `kp` (400) | Proportional — main steering force | Bot won't turn enough, drifts off | Bot oscillates/wobbles side-to-side |
| `ki` (0)   | Integral — corrects long-term drift | Slow to centre on gentle curves | Windup → overshoot, unstable |
| `kd` (0.1) | Derivative — dampens oscillation | More wobbly, over-correction | Sluggish response, may miss turns |
| `BASE_SPEED` (60) | Forward speed | Slow but stable | Fast but may overshoot turns |
| `SMOOTHING` (0.5) | Low-pass filter on correction | Jerkier motor commands | Slower to react to changes |

**Tuning steps:**
1. Start with `ki=0` and low `kd` (0.05–0.2)
2. Increase `kp` until the bot follows a straight line with minimal wobble
3. If wobbling, increase `kd` to dampen
4. Only add `ki` if the bot consistently drifts on long gentle curves
5. Increase `BASE_SPEED` once straight-line tracking is solid

### Pattern Thresholds

| Parameter | Default | Adjust if... |
|-----------|---------|-------------|
| `LINE_THRESHOLD` | 50 | False triggers (lower) or missing the line (raise) |
| `FULL_HIT_THRESHOLD` | 150 | Horizontal bars not detected (lower) or false horiz (raise) |
| `EDGE_THRESHOLD` | 100 | Sharp turns not detected (lower) or false sharp turns (raise) |
| `SHARP_TURN_INNER` | 30 | Opposite side not dark enough on turns (raise) |
| `MIN_CIRCLE_FRAMES` | 20 | False circle detections (raise) or circles not detected (lower) |
| `CIRCLE_POS_THRESHOLD` | 1.5 | Gentle curves wrongly flagged as circles (raise) |

### Timing Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `SHARP_TURN_PIVOT_MS` | 350ms | How long to hard-pivot on a 90° turn. Increase if the bot doesn't rotate enough |
| `T_JUNCTION_DRIVE_MS` | 400ms | How far to drive through a T-junction before resuming PID |
| `GAP_DECAY_MS` | 600ms | How long the bot continues in its last direction after losing the line |
| `GAP_FREEZE_MS` | 3000ms | Total time searching before stopping |

---

## Serial Monitor Output

Open the Serial Monitor at **115200 baud** to see debug output:

```
--- LSA08 PID v2 ---
KP: 400.00 KD: 0.10 BASE: 60.00
[PAT] Sharp LEFT
[PAT] T-Junction
[PAT] Cross
[PAT] Circle LEFT
[PAT] Circle END
[PAT] Gap
```

Each `[PAT]` line tells you which pattern was detected and which FSM state the robot entered.

---

## Build & Upload

### Using arduino-cli

```bash
# Compile
arduino-cli compile --fqbn arduino:avr:uno lsa08_PID.ino

# Upload (replace /dev/ttyACM0 with your port)
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno lsa08_PID.ino

# Monitor serial output
arduino-cli monitor -p /dev/ttyACM0 --config baudrate=115200
```

### Using Arduino IDE

1. Open `lsa08_PID.ino` in Arduino IDE
2. Select **Board → Arduino Uno**
3. Select the correct **Port**
4. Click **Upload** (✓)
5. Open **Tools → Serial Monitor** at 115200 baud

### Build Stats (Arduino Uno)

```
Sketch uses 8872 bytes (27%) of program storage space. Maximum is 32256 bytes.
Global variables use 529 bytes (25%) of dynamic memory, leaving 1519 bytes for local variables.
```

Plenty of room for additional features.

---

## Memory Layout

| Data                | Size     | Notes                              |
|---------------------|----------|------------------------------------|
| `sensorHistory`     | 150 bytes| 25 frames × 6 sensors × 1 byte    |
| PID state variables | ~24 bytes| floats: integral, error, correction|
| FSM state           | ~12 bytes| enum + timer + gap correction      |
| Motor pins array    | 12 bytes | 6 × 2 byte int                    |
| **Total SRAM**      | **529 B**| 25% of 2048 bytes on Uno           |

---

## License

This project is for educational and competition use. Modify freely for your track and hardware.
