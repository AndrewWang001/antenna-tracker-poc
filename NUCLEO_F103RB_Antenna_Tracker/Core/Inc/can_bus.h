/**
 * can_bus.h - bxCAN driver for the rocket CAN bus (CAN1, PA11 RX / PA12 TX).
 *
 * Hardware filters pass only the GPS message types (0x0E..0x13) with 29-bit
 * IDs. The FIFO0 interrupt timestamps each frame with the DWT cycle counter
 * and posts it to a FreeRTOS queue; all parsing happens in a task.
 *
 * Call can_bus_init() from a task: leaving init mode needs 11 recessive bits
 * on RX, and HAL's timeout for that relies on the tick.
 */
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "rocket_can.h"

typedef struct {
    uint32_t prescaler;
    uint32_t bs1_tq, bs2_tq, sjw_tq;
    bool     loopback;
} can_cfg_t;

typedef struct {
    volatile uint32_t rx;           /* frames taken from the FIFO */
    volatile uint32_t rx_dropped;   /* software queue full */
    volatile uint32_t fifo_overrun; /* hardware FIFO overflowed before the ISR ran */
    volatile uint32_t tx, tx_fail;
    volatile uint32_t errors;       /* error-warning / passive / bus-off events */
    volatile uint32_t bus_off;
} can_stats_t;

bool               can_bus_init(const can_cfg_t *cfg, osMessageQueueId_t rx_queue);
bool               can_bus_send(const can_frame_t *f);
const can_stats_t *can_bus_stats(void);
uint32_t           can_bus_esr(void);   /* raw ESR: TEC/REC counters + last error code */

#endif /* CAN_BUS_H */
