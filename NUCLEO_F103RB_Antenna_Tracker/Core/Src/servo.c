#include "servo.h"

void servo_init(servo_t *s, TIM_HandleTypeDef *htim, uint32_t channel,
                uint16_t min_us, uint16_t max_us, double range_deg, double start_deg)
{
    s->htim = htim;
    s->channel = channel;
    s->min_us = min_us;
    s->max_us = max_us;
    s->range_deg = range_deg;
    servo_write_deg(s, start_deg);           /* set CCR before the output starts */
    HAL_TIM_PWM_Start(htim, channel);
}

void servo_write_deg(servo_t *s, double deg)
{
    if (deg < 0.0) deg = 0.0;
    if (deg > s->range_deg) deg = s->range_deg;
    s->angle_deg = deg;

    const double span = (double)(s->max_us - s->min_us);
    const uint32_t pulse = (uint32_t)((double)s->min_us + deg / s->range_deg * span + 0.5);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, pulse);
}

uint16_t servo_pulse_us(const servo_t *s)
{
    return (uint16_t)__HAL_TIM_GET_COMPARE(s->htim, s->channel);
}
