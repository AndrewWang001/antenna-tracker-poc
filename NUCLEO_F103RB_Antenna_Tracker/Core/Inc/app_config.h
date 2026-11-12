/**
 * app_config.h - every tunable for the antenna tracker in one place.
 *
 * Pin map (NUCLEO-F103RB):
 *   PB4  TIM3_CH1  azimuth (pan) servo signal      Arduino D5
 *   PB5  TIM3_CH2  elevation (tilt) servo signal   Arduino D4
 *   PA11 CAN_RX    -> transceiver RXD              CN10 pin 14
 *   PA12 CAN_TX    -> transceiver TXD              CN10 pin 12
 *   PB8  I2C1_SCL  -> IMU SCL (remapped)           Arduino D15
 *   PB9  I2C1_SDA  -> IMU SDA (remapped)           Arduino D14
 *   PA2/PA3 USART2 -> ST-LINK virtual COM port (debug console, 115200 8N1)
 *   PC13 B1        blue button: re-zero azimuth mount heading
 *   PA5  LD2       status LED
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---------------- Ground station (observer) ------------------------------
 * Default antenna position. Override at runtime with the `obs` command.
 * Use the same altitude datum as the rocket GPS (NMEA GGA = metres above
 * mean sea level) so the geoid offset cancels out. */
#define OBS_DEFAULT_LAT_DEG          43.4723
#define OBS_DEFAULT_LON_DEG         (-80.5449)
#define OBS_DEFAULT_ALT_M            330.0

/* ---------------- CAN bus -------------------------------------------------
 * Waterloo Rocketry bus: 250 kbit/s (4 us bit time), 29-bit extended IDs.
 * APB1 = 32 MHz -> 32 MHz / 8 = 4 MHz tq clock, 1 + 13 + 2 = 16 tq per bit
 * -> 250 kbit/s, sample point 87.5 %. */
#define CAN_PRESCALER                8u
#define CAN_BS1_TQ                   13u
#define CAN_BS2_TQ                   2u
#define CAN_SJW_TQ                   1u
#define CAN_LOOPBACK                 0   /* 1 = bxCAN loopback, no transceiver needed */
#define CAN_RX_QUEUE_LEN             16u

/* Only assemble GPS fixes from this board type (rocketcan: 0x07 = COTS GPS
 * board). 0 accepts any board, which mixes sources if two GPSes are on the bus. */
#define GPS_SOURCE_BOARD_TYPE        0x07u
/* lat, lon and alt arrive as separate frames; they must all land within this
 * window to count as one fix. */
#define GPS_FIX_WINDOW_MS            500u
#define GPS_STALE_MS                 3000u

/* ---------------- IMU (BNO055 on I2C1) ----------------------------------- */
#define IMU_I2C_ADDR                 0x28u  /* 0x29 if ADR pin is high */
#define IMU_I2C_SPEED_HZ             100000u
/* BNO055 AXIS_MAP_CONFIG / AXIS_MAP_SIGN; datasheet table 3-5 (P1 = default). */
#define IMU_AXIS_MAP_CONFIG          0x24u
#define IMU_AXIS_MAP_SIGN            0x00u
/* Elevation = IMU_EL_SIGN * pitch + IMU_EL_OFFSET_DEG. Flip the sign if the
 * board is mounted so that tilting up reads negative. */
#define IMU_EL_SIGN                  (1.0)
#define IMU_EL_OFFSET_DEG            (0.0)
/* Magnetic declination (true = magnetic + decl). Waterloo ~ -9.5 deg.
 * Look up the launch site on the NOAA calculator before a launch. */
#define MAG_DECLINATION_DEG          (-9.5)

/* ---------------- Mount geometry ----------------------------------------- */
/* True heading the antenna faces when the pan servo is centred. The blue
 * button / `zero` command re-measures this from the IMU. */
#define AZ_MOUNT_HEADING_DEG         0.0
#define AZ_TRAVEL_DEG                90.0   /* +/- travel either side of centre */
#define EL_MIN_DEG                   0.0
#define EL_MAX_DEG                   90.0
#define PARK_EL_DEG                  20.0   /* where to point with no fix */

/* ---------------- Servos (TIM3 at 1 MHz tick, 50 Hz frame) --------------- */
#define SERVO_MIN_US                 500u
#define SERVO_MAX_US                 2500u
#define SERVO_RANGE_DEG              180.0
#define AZ_SERVO_CENTER_DEG          90.0   /* servo angle that faces mount heading */
#define AZ_SERVO_DIR                 (1.0)  /* -1 if +servo turns the antenna CCW */
#define EL_SERVO_HORIZON_DEG         0.0    /* servo angle when antenna is level */
#define EL_SERVO_DIR                 (1.0)
#define SERVO_MAX_RATE_DEG_S         120.0  /* setpoint slew limit; stops the jerky 30 deg jumps */

/* ---------------- Control loop ------------------------------------------- */
#define CONTROL_PERIOD_MS            20u    /* 50 Hz, matches the servo frame */

/* Outer-loop PID: output is a correction (deg) added to the feed-forward
 * servo angle. Starting values only; tune on the real mount. */
#define AZ_KP   0.6
#define AZ_KI   0.8
#define AZ_KD   0.02
#define EL_KP   0.6
#define EL_KI   0.8
#define EL_KD   0.02
#define PID_TAU_S          0.05   /* derivative low-pass time constant */
#define PID_BETA           1.0    /* setpoint weight on P */
#define PID_KB             2.0    /* anti-windup back-calculation gain (1/s) */
#define PID_OUT_LIMIT_DEG  20.0   /* max correction either way */

/* ---------------- Interrupt priorities -----------------------------------
 * Must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) because these
 * ISRs call FreeRTOS *FromISR APIs. */
#define IRQ_PRIO_CAN                 6u
#define IRQ_PRIO_I2C                 6u
#define IRQ_PRIO_UART                7u

#endif /* APP_CONFIG_H */
