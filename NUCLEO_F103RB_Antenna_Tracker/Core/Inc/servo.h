/**
 * servo.h - hobby servo on a TIM PWM channel.
 * Expects the timer to tick at 1 MHz with a 20 ms period (see MX_TIM3_Init),
 * so the compare value is the pulse width in microseconds.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_us, max_us;   /* pulse at 0 deg and at range_deg */
    double   range_deg;
    double   angle_deg;        /* last commanded angle (after clamping) */
} servo_t;

void     servo_init(servo_t *s, TIM_HandleTypeDef *htim, uint32_t channel,
                    uint16_t min_us, uint16_t max_us, double range_deg, double start_deg);
void     servo_write_deg(servo_t *s, double deg);
uint16_t servo_pulse_us(const servo_t *s);

#endif /* SERVO_H */
