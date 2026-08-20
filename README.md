# Leanbot firmware bring-up

This is a first-stage motor test for an Arduino Nano (ATmega328P, 16 MHz) and
two STEP/DIR stepper drivers. Once the sketch starts, it holds both STEP outputs
low and waits for a serial motion command.

## Confirmed pin map

| Function | Nano pin |
| --- | --- |
| Left / motor 1 DIR | D7 |
| Left / motor 1 STEP | D6 |
| Right / motor 2 DIR | D4 |
| Right / motor 2 STEP | D5 |

The mapping matches the current Leanbot motor library. Each motor has its own
STEP signal, so the two wheel speeds can be controlled independently.

## Electrical checklist

- Connect Nano GND and both driver logic grounds together.
- Before applying motor power, verify that each driver STEP input has a hardware
  pull-down. Add 10 kOhm from STEP to GND if the board does not provide one;
  Nano pins float while the bootloader/reset sequence is running.
- Use a proper current-limited stepper driver and a separate motor supply.
  Do **not** power either motor from the Nano 5 V pin.
- Confirm the motor supply voltage, driver current limit, and microstep setting
  before commanding motion. Also verify the driver's logic-input voltage and
  that it advances on a rising STEP edge.
- The current Leanbot motor library controls no ENABLE pin; the LB module handles
  that in hardware. `S` and the timeout stop STEP pulses, but an enabled driver
  may keep holding torque and become warm.
- Keep the wheels clear of the table for the first test.
- Switch off the motor supply before connecting or disconnecting a motor or its
  driver. USB power to the Nano is enough for the initial serial check.

## Upload

Open `LEANBOTFW.ino` in Arduino IDE and select:

- Board: **Arduino Nano**
- Processor: **ATmega328P** (try **ATmega328P (Old Bootloader)** for some clones)
- The serial port belonging to the Nano

Upload, then open Serial Monitor at **115200 baud** with the line ending set to
**Newline** or **Both NL & CR**.

## Safe motor test

The firmware accepts these commands:

```text
M <left> <right>   speed in steps/s, from -2000 to 2000
S                  request stop (applied within one 50 us timer tick)
P                  print commanded speeds and step counts
Z                  zero the commanded-step counters
H                  print help
```

Motion automatically stops after two seconds unless another `M` command is
received. Invalid non-empty commands also stop both motors; blank lines are
ignored.

Test one wheel at a low rate first:

```text
M 150 0
M 0 150
M 150 150
S
```

This bring-up build does not ramp speed yet. Start around 100–300 steps/s;
jumping straight to 2000 steps/s or reversing at high speed can jerk or stall an
open-loop motor even though the electrical pulses are correct.

Positive speed is intended to mean robot-forward. Current Leanbot software uses
DIR low for positive speed on both motors, so both `*_DIRECTION_INVERTED`
constants start as `true`. If a positive one-wheel command turns the wrong way,
change that motor's constant near the top of `LEANBOTFW.ino`.

The reported step counts are commanded open-loop steps, not encoder feedback.

## Current timer choice

The step generator uses Timer2 at 20 kHz and supports independent speeds up to
2000 steps/s. This preserves Timer0 (`millis`, `micros`, and `delay`) and Timer1
for a future Servo-based gripper. It means Arduino `tone()` and PWM on D3/D11
cannot be used without redesigning the timer ownership; a future buzzer driver
can share this scheduler. D5/D6 must remain plain GPIO outputs—do not call
`analogWrite()` on either STEP pin. Timer1 is not claimed by this test, but
servo timing still needs measurement while the 20 kHz motor interrupt is active.

## Confirmed Leanbot details

The [official Leanbot brochure](https://pythaverse.space/wp-content/uploads/2023/07/Leanbot-Brochure-v1_print.pdf)
specifies an ATmega328P Nano-compatible controller, two geared stepper motors,
differential drive, approximately 2038 steps per output revolution, and no wheel
encoders. The D7/D6 and D4/D5 mapping is independently visible in this
[public Leanbot investigation](https://github.com/tungbuivn/leanbot-dtt#readme).
The exact LB driver IC and its electrical limits are still not confirmed.

## Still needed for full Leanbot firmware

Before expanding beyond motor bring-up, record:

- exact LB Stepper driver/board model or clear photos of both sides;
- ENABLE pin and active level, if present;
- STEP pulse and DIR setup/hold timing requirements;
- motor step angle, gearbox ratio, and microstep configuration;
- wheel diameter and wheel-to-wheel distance;
- which sensors, gripper servos, buzzer, RGB LEDs, Bluetooth module, and battery
  monitor are fitted, including their pin map.
