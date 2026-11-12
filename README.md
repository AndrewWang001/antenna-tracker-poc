# antenna-tracker-poc

Firmware for an automated antenna tracker that keeps a ground-station antenna
pointed at a rocket. It runs on an STM32F103 (NUCLEO-F103RB) under FreeRTOS:

1. **Receives GPS telemetry over CAN**, using the Waterloo Rocketry bus format
   (29-bit IDs, `GPS_LATITUDE` / `GPS_LONGITUDE` / `GPS_ALTITUDE` / `GPS_INFO`
   frames at 250 kbit/s).
2. **Computes azimuth/elevation** from the ground station to the rocket
   (WGS-84 geodetic → ECEF → local ENU).
3. **Drives a pan/tilt mount with two servos** through a 50 Hz closed loop:
   the servo setpoint is used as feed-forward, and a PID per axis corrects it
   from the pointing a BNO055 IMU actually measures over I²C.

## History

I started this in 2024–25 for Waterloo Rocketry. The team bench-tested it with
a 3D-printed mock antenna, aiming at a rocket sitting on the ground. That tested
version, with the CAN and PID integration, was lost. Only these pieces survived:
`azel.c`, a standalone `pid.c`, and a CubeMX project that swept one servo.

This repo is a **2026 rebuild** that puts those pieces back together and
re-implements the parts that were lost (the CAN telemetry path, IMU feedback,
the two-axis control loop, and the drivers). The commit history shows what
changed.

**Status:** builds with no warnings in STM32CubeIDE and with the command-line
Makefile, and the host unit tests pass. **It has not been run on hardware yet.**
The [hardware results](#hardware-results) section stays empty until it has.

## Architecture

```
 CAN RX ISR ──queue──▶ telemetry task ──1-slot mailbox──▶ control task (50 Hz) ──▶ 2× servo PWM
 (DWT timestamp)       assemble lat/lon/alt fix           read IMU (I²C, IRQ-driven)
                       azel_compute()                     slew-limit setpoint
                                                          PID per axis → servo angle
 B1 button EXTI ──queue────────────────────────────────▶ (re-zero azimuth)
 USART2 ISR ──stream buffer──▶ console task: status, sim, i2ctest, gains, ...
```

| File | What it does |
|---|---|
| `Core/Src/azel.c` | Geodetic → az/el/range (double precision, WGS-84) |
| `Core/Src/pid.c` | PID: setpoint weighting, filtered derivative-on-measurement, back-calculation anti-windup |
| `Core/Src/rocket_can.c` | Decodes and encodes the team's GPS CAN frames, and assembles a fix from separate lat/lon/alt frames. Pure C, unit-tested |
| `Core/Src/can_bus.c` | bxCAN at 250 kbit/s, hardware ID filters for the GPS message types, RX ISR → FreeRTOS queue |
| `Core/Src/i2c_drv.c` | Interrupt-driven I²C master: configurable pins/remap, a mutex for the shared bus, bus recovery (9-clock + STOP + SWRST), error stats |
| `Core/Src/uart_drv.c` | Interrupt-driven UART for any USART and its pin options, with RX/TX stream buffers |
| `Core/Src/bno055.c` | BNO055 in NDOF fusion mode: heading, pitch, calibration status |
| `Core/Src/servo.c` | Servo PWM; TIM3 ticks at 1 µs so the compare value is the pulse width |
| `Core/Src/tracker.c` | Tasks, control loop, serial console |
| `Core/Inc/app_config.h` | **Every tunable**: pins, CAN timing, gains, mount geometry, declination |

## Hardware

| Signal | Pin | NUCLEO header |
|---|---|---|
| Azimuth servo | PB4 (TIM3_CH1) | D5 |
| Elevation servo | PB5 (TIM3_CH2) | D4 |
| CAN RX / TX → transceiver (e.g. SN65HVD230) | PA11 / PA12 | CN10-14 / CN10-12 |
| IMU SCL / SDA (BNO055 at 0x28) | PB8 / PB9 | D15 / D14 |
| Debug console | USART2 → ST-LINK VCP | USB, 115200 8N1 |
| Re-zero azimuth | B1 (PC13) | blue button |

Power the servos from their own 5–6 V supply, with its ground tied to the
Nucleo's. Mount the IMU on the antenna and keep it away from the servo motors,
because their magnetic field corrupts the heading.

## Build & flash

* **STM32CubeIDE:** File → Import → *Existing Projects into Workspace*, select
  `NUCLEO_F103RB_Antenna_Tracker/`, then build and run.
* **Command line:** `make -C NUCLEO_F103RB_Antenna_Tracker`. This needs
  `arm-none-eabi-gcc` on your PATH (CubeIDE bundles one).
  `make flash` programs the board over ST-LINK.
* **Host unit tests:** `make -C tests` (any desktop gcc). These cover the
  az/el math against an independent Vincenty solution, the PID (including a
  closed-loop run against a servo model with a mounting offset), and CAN
  frame encode/decode and fix assembly.

The CubeMX `.ioc` is up to date (the TIM3 1 MHz tick, the `control` task, the
heap size). Code generated outside CubeMX (CAN, I²C, the second PWM channel)
lives in its own files or in `USER CODE` blocks, so regenerating from the
`.ioc` keeps it.

## Console

Open the ST-LINK COM port at 115200 baud:

```
status                      target, setpoints, IMU, CAN counters, latency
obs <lat> <lon> <alt_m>     set the ground-station position
track | point <az> <el>     auto tracking / manual pointing
zero                        re-measure the mount heading (same as the blue button)
gains <az|el> <kp> <ki> <kd>
sim <lat> <lon> <alt_m> [bus]   inject a GPS fix as real-format CAN frames
i2ctest [n]                 n IMU reads, reports packet loss + error breakdown
tasks                       stack high-water marks, free heap
```

`sim` pushes frames through the same queue the CAN interrupt feeds, so it
exercises everything after the bus with no rocket attached. `sim … bus`
transmits them on the CAN bus instead. In normal mode another node has to
acknowledge them (a USB-CAN adapter, for example), or you can set
`CAN_LOOPBACK 1` in `app_config.h`.

## Bring-up checklist

1. With no IMU attached: `status` should show `imu offline (open loop)`.
   `point 0 45` should move the servos.
2. Attach the IMU. Move the board around until `calib` reads 3 for gyro,
   accel and mag. Check that tilting the antenna up raises the elevation
   reading (if not, flip `IMU_EL_SIGN`).
3. Point the antenna at something with a known bearing and press **B1**.
   That sets the mount heading. Set `MAG_DECLINATION_DEG` for the site first.
4. `obs <your position>`, then `sim` a point about 100 m away and check it
   aims there.
5. Tune `gains` if it oscillates (lower Kp/Kd) or settles slowly (raise Ki).

## Hardware results

*To be filled in after bench testing. Nothing below is measured yet.*

| Test | How | Result |
|---|---|---|
| I²C packet loss | `i2ctest 10000` with the control loop running | — |
| CAN ISR → target latency | `status` → `latency` (max over a sim/real run) | — |
| Pointing error at rest | IMU az/el vs. target after settling | — |
| Stack margin | `tasks` after a full run | — |

## Notes

* The repo previously had a `float` port of `azel.c`. This rebuild goes back
  to the original `double` version. The Cortex-M3 has no FPU, so `float` saved
  little time. In the unit tests the `float` version is off by about 0.06° in
  elevation at 150 m range.
* Elevation from Euler angles is unreliable near vertical. Above 80°
  elevation, azimuth runs feed-forward only.
* `GPS_ALTITUDE` carries metres above mean sea level (NMEA GGA). Give the
  ground station's altitude in the same datum.
