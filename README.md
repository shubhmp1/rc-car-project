# Dual-ESP32 RC Car: Custom PCB, 3D-Printed Chassis, With Standard Driving and Auto-Driving Modes

A fully custom-built RC car, designed from scratch: hand-routed PCBs, 3D-printed chassis and wheels, and firmware for two ESP32s talking over nRF24L01 RF. Built solo from schematic to final assembly.

![Final car](docs/images/final-car-hero.jpg)
<!-- swap in your best final-photo filename above -->

---

## Why I built this

Going into engineering, I felt lost, since there are so many different paths (mechanical, electrical, software, embedded) and so much to learn that it was hard to know where to actually start. So instead of trying to learn everything separately, I picked one project that would force me to touch all of it, end to end: CAD, PCB design, firmware, RF communication, sensors, and 3D printing.

Every single one of those was a first for me. I'd never designed a mechanical assembly, never laid out a PCB, never written embedded firmware for a microcontroller, never worked with RF modules or these sensors before this project. I learned all of it here, from scratch, because I wanted proof (mostly to myself) that I could pick up any of these disciplines if I actually committed to it.

And I picked an RC car specifically because outside of computers, cars are what I love most. This project let me combine both.

---

## Demo

**Video:** _[YouTube link coming soon]_

<!-- once uploaded, replace with: [![Watch the demo](docs/images/thumbnail.jpg)](https://youtube.com/your-link) -->

---

## What it does

The car runs on two ESP32s (one in the handheld remote, one on the car), communicating over **nRF24L01 RF**. The remote has a joystick, a TFT display showing speed/mode/sensor data, and three mode buttons with illuminated LEDs.

**Modes:**
- **Standard Driving Mode (done):** manual driving. Joystick controls throttle (with exponential curve, deadzone, and slew-rate limiting) and steering.
- **Auto-Driving Mode (done):** a front-facing ToF sensor slows/stops the car as it approaches obstacles, while two IR line sensors keep it centered between track markers (tested using black tape "lanes" on the ground).
- **AI Gesture Mode (planned, not built):** connect the car to a PC over WiFi and drive it using hand-gesture recognition from a model running locally. Documented as a future addition in the [build notes](docs/Rough_Dev_Journal_And_Notes.docx).

## Hardware

Full parts list and exact values are in the [Specs](#specs) table below. At a high level: two ESP32s (one per board), custom Altium PCBs for both the car and remote, a brushed DC gear motor through an L298N driver, servo-based Ackermann steering, a ToF sensor for obstacle detection, IR sensors for line tracking, and a 3D-printed chassis/wheel set iterated across several print revisions.

## Specs

| Spec | Value |
|---|---|
| MCU | 2× ESP32 (one in the remote, one on the car) |
| Communication | nRF24L01, 250kbps, with ACK-payload telemetry back to the remote |
| Motor | GA12-N20, 6V, 200RPM, all-metal gearbox, 3mm shaft |
| Motor Driver | L298N |
| Steering | Standard positional servo, Ackermann-style linkage |
| Joystick | PS2-style dual-axis breakout module |
| Display | Waveshare 2.4" SPI LCD, 240×320, 65K color |
| Obstacle Sensor | VL53L1X ToF (long-range mode) |
| Line Sensors | 2× TCRT5000 IR reflectance sensors |
| Chassis / Wheels | 3D-printed (PLA), iterated through multiple print revisions |
| PCBs | Custom, designed in Altium (one for the car, one for the remote) |

## How it works

**Communication:** the remote and car each run their own ESP32, talking over an nRF24L01 RF link. Every ~50ms the remote sends a small packet containing motor speed, steering angle, current mode, and (in auto-driving mode) an armed flag and target cruise speed. The car replies on the same exchange using nRF24's ACK-payload feature, sending back live sensor readings (IR states, ToF distance, current speed, and status flags) so the remote's display can show what the car is actually doing in real time, not just what was last commanded.

**Standard driving mode:** joystick input is normalized, run through a small deadzone, shaped with an exponential curve (so small stick movements give fine control, large movements ramp up quickly), and slew-rate limited before being sent, so the motor doesn't jump instantly between speeds or slam through zero when reversing.

**Auto-driving mode:** the car ignores direct joystick input entirely and drives itself off two inputs: two IR sensors on the underside detect track-edge markers and nudge steering back toward center when either sensor sees the line, and a ToF sensor up front continuously calculates a dynamic safe-stopping distance based on the car's current speed (faster speed → longer required stopping distance), slowing or fully stopping the car as it approaches obstacles. The remote is used first to set a target cruise speed, then the car handles both throttle and steering on its own within that mode.

**Displays and modes:** the remote's screen (driven over SPI) shows different HUDs depending on the active mode, live speed/direction/steering in standard mode, or live sensor telemetry and braking status in auto-driving mode, so you can see what the car's sensors are reading even while it's driving itself.

Full firmware for both boards is in [`firmware/`](firmware/).

## Repo structure

```
├── docs/
│   ├── Rough_Dev_Journal_And_Notes.docx   ← full unfiltered build log/notes
│   └── images/                             ← screenshots & progress photos
├── hardware/
│   ├── cad/           ← final STEP files (chassis, wheels, mounts)
│   ├── pcb/            ← Altium projects + gerbers for both boards
│   │   ├── RC_Car/
│   │   └── RC_Remote/
│   └── 3d-print/       ← .3mf print files (all iterations kept)
├── firmware/
│   ├── Car/            ← car-side Arduino sketch
│   └── RC/             ← remote-side Arduino sketch
├── media/
│   ├── build-photos/   ← progress photos throughout the build
│   └── final-photos/   ← finished car
└── README.md
```

## Build photos

A few shots from along the way. See [`media/build-photos`](media/build-photos) for the full set.

<!-- Consider dropping 2-3 of your favorite in-progress photos inline here, e.g.: -->
<!-- ![Soldering the PCB](media/build-photos/photo1.jpg) -->

## Lessons learned

Pulled from the full [build journal](docs/Rough_Dev_Journal_And_Notes.docx), short version:

- **Bluetooth's signal backlog caused an infinite loop** in early remote-to-car communication. Switched to nRF24 RF instead, which was far more reliable for this use case.
- **PWM-driven motor control glitched under per-loop calculation load:** the ESP32 couldn't keep up doing full throttle/LED calculations every cycle. Fixed by throttling the calculation rate (~25Hz instead of 100Hz) and reducing floating-point math in the hot path.
- **Motor driver testing with a bare BJT/MOSFET failed:** couldn't draw enough current. Confirmed the need for a proper motor driver IC (L298N) rather than a hand-built switching circuit.
- **A GND/5V short during power-supply assembly** was a good reminder to double- and triple-check polarity before powering up a new board, since one bad connection can take out multiple components at once.
- **Hand-soldering fine-pitch parts (buck converters, ESP32 headers) without hot air or flux paste** led to several repeated attempts and a few damaged boards/pins, showing the value of having the right soldering tools before starting dense PCB assembly.
- **PCB trace width mattered more than expected:** signal traces at 0.2mm, power at 0.5mm, and battery lines at 0.8mm, with careful separation between noisy traces (motor lines) and sensitive ones (RF module) to avoid interference.
- **3D-printed part tolerances needed iteration:** the first wheel rim print was a couple mm undersized on outer diameter despite correct shaft/width dimensions, so it took a second revision to dial in.
- **Debugging RF instability came down to reworked solder joints:** several intermittent sensor/module failures traced back to damaged pins from desoldering, ultimately solved by fresh ESP32 boards rather than continuing to chase individual pin issues.

## Status

Core build (standard driving mode + auto-driving mode) is complete and working. AI gesture mode is a planned future addition; see the notes for early thinking on that.

## License

[MIT](LICENSE). Feel free to reference, fork, or build on this. Attribution appreciated.
