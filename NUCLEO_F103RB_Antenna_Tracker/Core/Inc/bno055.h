/**
 * bno055.h - Bosch BNO055 9-axis IMU in NDOF fusion mode, over i2c_drv.
 * Gives absolute orientation (heading from the magnetometer, pitch/roll
 * from accel+gyro), which is the tracker's pointing feedback.
 *
 * Uses osDelay(): call only from a task.
 */
#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>
#include <stdint.h>
#include "i2c_drv.h"

typedef struct {
    double  heading_deg;   /* 0..360, clockwise from magnetic north */
    double  roll_deg;
    double  pitch_deg;
    uint8_t calib;         /* CALIB_STAT: [7:6] sys [5:4] gyro [3:2] accel [1:0] mag, 3 = full */
} bno055_euler_t;

bool bno055_init(i2c_bus_t *bus, uint8_t addr, uint8_t axis_map_config, uint8_t axis_map_sign);
bool bno055_read_euler(i2c_bus_t *bus, uint8_t addr, bno055_euler_t *out);
bool bno055_read_chip_id(i2c_bus_t *bus, uint8_t addr, uint8_t *id);

#define BNO055_CHIP_ID_VALUE  0xA0u

#endif /* BNO055_H */
