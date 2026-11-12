#include "can_bus.h"
#include "board_util.h"
#include "app_config.h"
#include <string.h>

static CAN_HandleTypeDef  s_hcan;
static osMessageQueueId_t s_rx_q;
static can_stats_t        s_stats;

/* Build a 32-bit filter register value for a 29-bit ID: [31:3] ID, bit2 IDE. */
static uint32_t ext_filter_word(uint32_t id29)
{
    return (id29 << 3) | CAN_ID_EXT;
}

/* Accept extended frames whose message-type field (ID bits 26:20) matches
 * `type` in the bits set in `type_mask`. */
static bool add_type_filter(uint32_t bank, uint8_t type, uint8_t type_mask)
{
    const uint32_t id   = ext_filter_word((uint32_t)type << 20);
    const uint32_t mask = ext_filter_word((uint32_t)type_mask << 20);   /* IDE bit must match too */

    CAN_FilterTypeDef f = {0};
    f.FilterBank = bank;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = id >> 16;
    f.FilterIdLow = id & 0xFFFFu;
    f.FilterMaskIdHigh = mask >> 16;
    f.FilterMaskIdLow = mask & 0xFFFFu;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = 14;
    return HAL_CAN_ConfigFilter(&s_hcan, &f) == HAL_OK;
}

/* Called by HAL_CAN_Init. */
void HAL_CAN_MspInit(CAN_HandleTypeDef *h)
{
    if (h->Instance != CAN1) return;
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_11;                 /* CAN_RX */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;                /* recessive if the transceiver is unplugged */
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_12;                 /* CAN_TX */
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, IRQ_PRIO_CAN, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, IRQ_PRIO_CAN, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

bool can_bus_init(const can_cfg_t *cfg, osMessageQueueId_t rx_queue)
{
    /* Safe to call again after a failed start (retry path). */
    if (s_hcan.Instance) HAL_CAN_DeInit(&s_hcan);
    s_rx_q = rx_queue;

    s_hcan.Instance = CAN1;
    s_hcan.Init.Prescaler = cfg->prescaler;
    s_hcan.Init.Mode = cfg->loopback ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
    s_hcan.Init.SyncJumpWidth = (cfg->sjw_tq - 1u) << CAN_BTR_SJW_Pos;
    s_hcan.Init.TimeSeg1 = (cfg->bs1_tq - 1u) << CAN_BTR_TS1_Pos;
    s_hcan.Init.TimeSeg2 = (cfg->bs2_tq - 1u) << CAN_BTR_TS2_Pos;
    s_hcan.Init.TimeTriggeredMode = DISABLE;
    s_hcan.Init.AutoBusOff = ENABLE;        /* recover from bus-off by itself */
    s_hcan.Init.AutoWakeUp = DISABLE;
    s_hcan.Init.AutoRetransmission = ENABLE;
    s_hcan.Init.ReceiveFifoLocked = DISABLE; /* on overflow, newest frame wins */
    s_hcan.Init.TransmitFifoPriority = ENABLE;
    if (HAL_CAN_Init(&s_hcan) != HAL_OK) return false;

    /* GPS message types 0x0E..0x13 in two mask filters:
     *   0x10..0x13 (lon, alt, info, + stream status, dropped in software)
     *   0x0E..0x0F (timestamp, lat) */
    if (!add_type_filter(0, 0x10, 0x7C)) return false;
    if (!add_type_filter(1, 0x0E, 0x7E)) return false;

    if (HAL_CAN_ActivateNotification(&s_hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
            CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK) {
        return false;
    }
    return HAL_CAN_Start(&s_hcan) == HAL_OK;
}

bool can_bus_send(const can_frame_t *f)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(&s_hcan) == 0) { s_stats.tx_fail++; return false; }

    CAN_TxHeaderTypeDef hdr = {0};
    hdr.ExtId = f->id & 0x1FFFFFFFu;
    hdr.IDE = CAN_ID_EXT;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = f->len;
    hdr.TransmitGlobalTime = DISABLE;
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&s_hcan, &hdr, (uint8_t *)f->data, &mailbox) != HAL_OK) {
        s_stats.tx_fail++;
        return false;
    }
    s_stats.tx++;
    return true;
}

const can_stats_t *can_bus_stats(void) { return &s_stats; }
uint32_t can_bus_esr(void) { return CAN1->ESR; }

/* ------------------------------------------------- HAL callbacks (ISR) */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    CAN_RxHeaderTypeDef hdr;
    can_frame_t f;

    while (HAL_CAN_GetRxFifoFillLevel(h, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &hdr, f.data) != HAL_OK) break;
        f.rx_cycles = dwt_cycles();
        s_stats.rx++;
        if (hdr.IDE != CAN_ID_EXT || hdr.RTR != CAN_RTR_DATA) continue;
        f.id = hdr.ExtId;
        f.len = (uint8_t)hdr.DLC;
        /* osMessageQueuePut is ISR-safe with timeout 0; never block here. */
        if (osMessageQueuePut(s_rx_q, &f, 0, 0) != osOK) s_stats.rx_dropped++;
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h)
{
    const uint32_t e = h->ErrorCode;
    if (e & HAL_CAN_ERROR_RX_FOV0) s_stats.fifo_overrun++;
    if (e & (HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_BOF)) s_stats.errors++;
    if (e & HAL_CAN_ERROR_BOF) s_stats.bus_off++;
    HAL_CAN_ResetError(h);
}

void USB_LP_CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&s_hcan); }
void CAN1_SCE_IRQHandler(void)        { HAL_CAN_IRQHandler(&s_hcan); }
