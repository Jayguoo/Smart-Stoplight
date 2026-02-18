# Smart Stoplight System (FreeRTOS)

A real-time smart traffic light system built on an ESP32 using FreeRTOS. The system manages two-way traffic with PIR-based vehicle detection, pedestrian crosswalk functionality, and a 7-segment countdown display — all driven by software timers and a finite state machine.

🔗 **[Simulate on Wokwi](https://wokwi.com/projects/447547556902086657)**

## Features

- **Two-direction traffic control** — Full green → yellow → red cycle with 18s green, 4s yellow, and 2s all-red safety windows
- **PIR vehicle detection** — Detects cars waiting at a red light and shortens the opposing green phase to 8 seconds max
- **Pedestrian crosswalk** — Push-button activated, triggers during the next all-red window with a 20-second crossing period
- **7-segment countdown display** — TM1637 display shows remaining crosswalk time, updated at 4 Hz
- **Serial gatekeeper** — All serial output routed through a FreeRTOS queue to prevent race conditions

## Architecture

### Finite State Machine

```
INIT (2s) → G1 (18s) → Y1 (4s) → R1 (2s) → G2 (18s) → Y2 (4s) → R2 (2s) → G1 ...
                                      ↓                                  ↓
                                   CW (22s) ─────────────────────────→ G1
```

- **G1/G2**: Green phase for direction 1/2. PIR detection can shorten to 8s remaining.
- **Y1/Y2**: Yellow warning phase (4s).
- **R1/R2**: All-red safety window (2s). Crosswalk activates here if requested.
- **CW**: Crosswalk mode — both directions red for 20s + 2s safety buffer.

### FreeRTOS Components

| Component | Type | Purpose |
|-----------|------|---------|
| `trafficTimerCallback` | Software Timer (1 Hz) | Drives FSM state transitions |
| `displayTimerCallback` | Software Timer (4 Hz) | Updates TM1637 crosswalk countdown |
| `gatekeeperTask` | Task (Priority 3) | Serializes all UART output via queue |
| `crosswalkConsumerTask` | Task (Priority 2) | Handles crosswalk button semaphore |
| `serialQ` | Queue (10 × 96 bytes) | Thread-safe serial message buffer |
| `crosswalkSem` | Binary Semaphore | Button ISR → crosswalk task signaling |
| PIR ISRs | Hardware Interrupt | Sets detection flags for FSM |
| Button ISRs | Hardware Interrupt (falling edge) | Signals crosswalk semaphore with 200ms debounce |

## Pin Mapping

| Pin | Component |
|-----|-----------|
| 42, 41, 40 | Direction 1 — Red, Yellow, Green LEDs |
| 37, 38, 39 | Direction 2 — Red, Yellow, Green LEDs |
| 36 | Crosswalk LED (white) |
| 14 | PIR Sensor — Direction 1 |
| 15 | PIR Sensor — Direction 2 |
| 12 | Crosswalk Button — Direction 1 |
| 13 | Crosswalk Button — Direction 2 |
| 2, 4 | TM1637 Display — CLK, DIO |

## How to Run

### Wokwi (Recommended)
1. Open the [Wokwi project link](https://wokwi.com/projects/447547556902086657)
2. Click **Start Simulation**
3. Click the PIR sensors to simulate vehicle detection
4. Click the green push buttons to request a crosswalk

### Local (ESP32-S2)
1. Install [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support
2. Install the **Grove 4-Digit Display** library
3. Flash `sketch.ino` to an ESP32-S2-DevKitM-1
4. Wire components according to the pin mapping above
5. Open Serial Monitor at 115200 baud

## File Structure

```
smart-stoplight-rtos/
├── sketch.ino          # Main application code
├── diagram.json        # Wokwi circuit schematic
├── libraries.txt       # Required libraries
├── wokwi-project.txt   # Wokwi project link
└── README.md
```

## Built With

- **ESP32-S2** — Microcontroller
- **FreeRTOS** — Real-time operating system (tasks, timers, queues, semaphores)
- **TM1637** — 4-digit 7-segment display
- **Wokwi** — Circuit simulation
