# Pen servo

`pen_servo` contains the C++ nonblocking pen-button state machine and a separate
ESP-IDF firmware project. The ROS executor owns DO0 requests; this package adds
no SDK connection, ROS servo PWM loop, or independent output publisher.

| Logical operation | Flexiv DO0 | ESP32 GPIO35 | GPIO18 servo reference |
| --- | --- | --- | --- |
| Extrude | HIGH / about 24 V | LOW / about 0.2 V | 135° |
| Release | LOW / about 0 V | HIGH / about 3.3 V | 110° |

The measured circuit inversion belongs at GPIO35 decoding. Do not invert the
ROS command a second time. GPIO35 is input-only and lacks internal pull
resistors; verify an external HIGH inactive bias, including disconnected-wire
and interface-power-loss cases. Servo power comes from its suitable supply,
with the documented common reference or isolation.

## Firmware configuration and lifecycle

Target board: ESP32 DevKit V1, ESP-IDF **v5.4.2**, classic `esp32` target. Confirm
the actual module and flash size before deployment. Commands use angles:
**release = 110°; press = 135°**. Firmware converts each angle to a pulse using
configured pulse widths at 0° and 180°. The supplied Arduino sketch sets **50 Hz** and **500–2400 µs** endpoints;
these are now configured. The requested 110°/135° commands map to approximately
1661/1925 µs. The sketch’s initial 90°/110° angles are superseded by the user’s
110°/135° choice; its 500 ms cycle delays are not flow calibration. Debounce and
maximum continuous press time remain TBD. Set these before enabling
`PEN_CALIBRATED`. The default build leaves GPIO18 as input and cannot press.
It does not promise physical release without a calibrated release pulse.

With calibration configured, boot commands RELEASE and waits for stable HIGH
on GPIO35 before a later LOW can press. An inactive input releases immediately.
A held-active timeout releases and latches a fault; send uppercase `R` through
the serial console with stable inactive input to rearm, or reset and repeat the
inactive-before-arm sequence. GPIO level alone cannot immediately detect a
frozen host. The maximum press cap bounds stale-high DO0. The reusable state
machine has a tested heartbeat gate for a future serial protocol; the baseline
firmware does not claim that such a heartbeat connection is installed.

## Build and deployment

Build host logic through colcon and run `colcon test --packages-select pen_servo`.
The nested firmware folder has `COLCON_IGNORE`; it uses its own toolchain:

```bash
git clone --branch v5.4.2 --depth 1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf-v5.4.2
~/esp/esp-idf-v5.4.2/install.sh esp32
source ~/esp/esp-idf-v5.4.2/export.sh
cd ~/robotics_ws/src/pen_servo/firmware/pen_servo_esp32
idf.py set-target esp32
idf.py menuconfig  # Pen servo measured configuration
idf.py build
```

Flashing is a separate deliberate operation after identifying the actual USB
serial device and establishing the mechanism's released state:

```bash
idf.py -p /dev/serial/by-id/CONFIRMED_DEVICE flash monitor
```

This replaces the previous board application. Neither colcon nor robot/browser
startup flashes firmware. Once installed, the firmware runs from flash at every
power-up; Mocha need not keep an IDE open. Mechanical release and actual flow
must be bench-verified. Servo commands and GPIO states are not flow feedback.

## Verified build boundary

The host state-machine test and ESP-IDF v5.4.2 `esp32` firmware build passed on
2026-10-24. The toolchain is installed under `~/esp/esp-idf-v5.4.2`, with its
tools/Python environment under `~/.espressif`. The generated default binary has
calibration disabled and PWM disabled. The connected ESP32 was flashed on
2026-10-24; flash verification and boot logs confirm it starts with PWM disabled.
No servo actuation was commanded.
Build evidence: `~/robotics_setup/validation/pen-hardware/result.json`.
Flash evidence: `~/robotics_setup/validation/pen-hardware/flash-result.json`.

References: [ESP-IDF v5.4.2 LEDC](https://docs.espressif.com/projects/esp-idf/en/v5.4.2/esp32/api-reference/peripherals/ledc.html),
[ESP32 GPIO restrictions](https://docs.espressif.com/projects/esp-idf/en/v5.4.2/esp32/api-reference/peripherals/gpio.html).

## Supervised bench test (2026-10-24)

Flashed with PWM enabled, 10 ms debounce and a 2000 ms maximum press.
These are initial bench settings, not qualified flow timing. With the arm
stopped/idle, DO0 produced GPIO35 LOW/PRESS then HIGH/RELEASE. A second
held request reached the 2000 ms cutoff and commanded RELEASE while the
input remained LOW; explicit rearm succeeded after DO0 OFF. DO0 was left OFF.
Serial evidence confirms commanded behavior, not physical horn motion.
Disconnect/floating-input behavior and loaded mechanism remain unverified.
Evidence: `~/robotics_setup/validation/pen-hardware/bench-result.json` and
`bench-serial.log`. Earlier PWM-disabled records describe previous builds.

Previous press command: **135°** (1925 µs), release **110°**. The earlier successful physical bench test used 130°; the revised position passed the DO0/serial bench test (normal release, 2-second timeout, and rearm). Physical observation is pending. Evidence: `~/robotics_setup/validation/pen-hardware/bench135-result.json`.

The operator subsequently selected a **600-second (10-minute)** continuous press
cap. Firmware configuration and the real calibration profile now use this value.
It includes the start dwell; an inactive input still releases immediately. The new
firmware was flashed with hash verification and ready/released startup confirmed.
The full-duration cutoff test remains pending; the historical tests above used
the 2-second cutoff. Evidence:
`~/robotics_setup/validation/pen-hardware/press600-result.json`.

Current firmware press command: **140°** (approximately 1978 µs), release **110°** (1661 µs). The 600-second cap is unchanged. Flash hash and ready/released boot were verified; evidence: `~/robotics_setup/validation/pen-hardware/press140-result.json`. The earlier 1-second flow-start observation was made at 135° and remains provisional at 140°.
