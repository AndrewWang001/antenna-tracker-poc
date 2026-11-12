/**
 * i2c_drv.h - interrupt-driven I2C master driver (I2C1/I2C2, STM32F1).
 *
 * - Pins / AFIO remap / speed / IRQ priority come from i2c_cfg_t.
 * - Transfers run on the HAL interrupt API; the calling task blocks on a
 *   semaphore the completion/error callbacks give, so the CPU is free while
 *   bytes move.
 * - A mutex serialises tasks sharing the bus.
 * - Bus recovery: clocks out up to 9 SCL pulses and forces a STOP, then
 *   soft-resets the peripheral. It runs at init if BUSY is already latched
 *   (a known STM32F1 I2C problem) and after any timeout or bus error.
 * - Every transfer is counted in i2c_stats_t, which the `i2ctest` console
 *   command uses to measure packet loss.
 *
 * Call i2c_init() from a task (after the scheduler has started).
 */
#ifndef I2C_DRV_H
#define I2C_DRV_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
    I2C_TypeDef  *instance;     /* I2C1 or I2C2 */
    uint32_t      speed_hz;     /* 100000 or 400000 */
    GPIO_TypeDef *scl_port;
    uint16_t      scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t      sda_pin;
    bool          remap;        /* I2C1 only: PB6/PB7 -> PB8/PB9 */
    uint32_t      irq_prio;
} i2c_cfg_t;

typedef enum {
    I2C_OK = 0,
    I2C_ERR_NACK,
    I2C_ERR_TIMEOUT,
    I2C_ERR_BUS,        /* bus error / arbitration lost / overrun */
    I2C_ERR_BUSY,       /* HAL refused to start: peripheral stuck busy */
} i2c_status_t;

typedef struct {
    uint32_t ok, nack, timeout, bus_err, busy, recoveries;
} i2c_stats_t;

typedef struct {
    i2c_cfg_t          cfg;
    I2C_HandleTypeDef  hi2c;
    SemaphoreHandle_t  lock;
    SemaphoreHandle_t  done;
    volatile uint32_t  hal_error;    /* HAL ErrorCode from the last transfer */
    i2c_stats_t        stats;
} i2c_bus_t;

bool         i2c_init(i2c_bus_t *bus, const i2c_cfg_t *cfg);
i2c_status_t i2c_mem_read(i2c_bus_t *bus, uint8_t addr7, uint8_t reg,
                          uint8_t *buf, uint16_t len, uint32_t timeout_ms);
i2c_status_t i2c_mem_write(i2c_bus_t *bus, uint8_t addr7, uint8_t reg,
                           const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
void         i2c_recover(i2c_bus_t *bus);
void         i2c_stats_reset(i2c_bus_t *bus);

#endif /* I2C_DRV_H */
