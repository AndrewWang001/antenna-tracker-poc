/**
 * tracker.h - antenna tracker application.
 *
 *   CAN RX ISR --(queue)--> telemetry task --(1-slot mailbox)--> control task --> servos
 *                           GPS fix assembly                      50 Hz: IMU read,
 *                           + az/el solve                         PID per axis
 *   B1 EXTI ISR --(queue)----------------------------------------> (re-zero azimuth)
 *   USART2 ISR --(stream buffer)--> console task (status, sim, i2ctest, ...)
 */
#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>

/* From main.c USER CODE BEGIN 2: non-RTOS hardware (servos, cycle counter). */
void tracker_hw_init(void);

/* From main.c USER CODE BEGIN RTOS_THREADS: queues, mutexes, and the
 * telemetry + console tasks. */
void tracker_rtos_init(void);

/* Bodies of the CubeMX-generated tasks. */
void tracker_control_task(void *argument);
void tracker_led_task(void *argument);

#endif /* TRACKER_H */
