#include "uart_drv.h"
#include "board_util.h"
#include <string.h>

static uart_t *s_ports[3];   /* USART1..3 -> driver instance, for the IRQ handlers */

static int port_index(USART_TypeDef *inst)
{
    if (inst == USART1) return 0;
    if (inst == USART2) return 1;
    if (inst == USART3) return 2;
    return -1;
}

static void tx_irq_enable(uart_t *u)
{
    taskENTER_CRITICAL();                 /* RMW of CR1 also done by the ISR */
    u->cfg.instance->CR1 |= USART_CR1_TXEIE;
    taskEXIT_CRITICAL();
}

bool uart_init(uart_t *u, const uart_cfg_t *cfg)
{
    const int idx = port_index(cfg->instance);
    if (idx < 0) return false;

    memset(u, 0, sizeof(*u));
    u->cfg = *cfg;
    if (!u->cfg.rx_buf_size) u->cfg.rx_buf_size = 128;
    if (!u->cfg.tx_buf_size) u->cfg.tx_buf_size = 256;

    u->rx = xStreamBufferCreate(u->cfg.rx_buf_size, 1);
    u->tx = xStreamBufferCreate(u->cfg.tx_buf_size, 1);
    u->tx_lock = xSemaphoreCreateMutex();
    if (!u->rx || !u->tx || !u->tx_lock) return false;

    USART_TypeDef *usart = cfg->instance;
    IRQn_Type irqn;
    uint32_t pclk;

    /* Peripheral clock + optional pin remap. */
    switch (idx) {
    case 0:
        __HAL_RCC_USART1_CLK_ENABLE();
        if (cfg->remap) __HAL_AFIO_REMAP_USART1_ENABLE();
        irqn = USART1_IRQn;
        pclk = HAL_RCC_GetPCLK2Freq();
        break;
    case 1:
        __HAL_RCC_USART2_CLK_ENABLE();
        if (cfg->remap) __HAL_AFIO_REMAP_USART2_ENABLE();
        irqn = USART2_IRQn;
        pclk = HAL_RCC_GetPCLK1Freq();
        break;
    default:
        __HAL_RCC_USART3_CLK_ENABLE();
        if (cfg->remap) __HAL_AFIO_REMAP_USART3_PARTIAL();
        irqn = USART3_IRQn;
        pclk = HAL_RCC_GetPCLK1Freq();
        break;
    }

    /* Pins: TX alternate-function push-pull, RX input with pull-up so a
     * disconnected line idles high instead of producing framing errors. */
    GPIO_InitTypeDef g = {0};
    gpio_clk_enable(cfg->tx_port);
    gpio_clk_enable(cfg->rx_port);
    g.Pin = cfg->tx_pin; g.Mode = GPIO_MODE_AF_PP; g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(cfg->tx_port, &g);
    g.Pin = cfg->rx_pin; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(cfg->rx_port, &g);

    /* 8N1, 16x oversampling: BRR = fPCLK / baud (mantissa.fraction in 1/16). */
    usart->CR1 = 0;
    usart->CR2 = 0;
    usart->CR3 = 0;
    usart->BRR = (pclk + cfg->baud / 2u) / cfg->baud;
    (void)usart->SR;
    (void)usart->DR;                       /* flush anything left from CubeMX init */

    s_ports[idx] = u;
    HAL_NVIC_SetPriority(irqn, cfg->irq_prio, 0);
    HAL_NVIC_EnableIRQ(irqn);

    usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    return true;
}

size_t uart_write(uart_t *u, const void *data, size_t len, uint32_t timeout_ms)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t sent = 0;

    if (xSemaphoreTake(u->tx_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return 0;
    while (sent < len) {
        size_t n = xStreamBufferSend(u->tx, p + sent, len - sent, pdMS_TO_TICKS(timeout_ms));
        if (n) tx_irq_enable(u);
        if (n == 0) break;                 /* buffer stayed full for timeout_ms */
        sent += n;
    }
    xSemaphoreGive(u->tx_lock);
    return sent;
}

void uart_puts(uart_t *u, const char *s)
{
    uart_write(u, s, strlen(s), 100);
}

int uart_getc(uart_t *u, uint32_t timeout_ms)
{
    uint8_t c;
    if (xStreamBufferReceive(u->rx, &c, 1, pdMS_TO_TICKS(timeout_ms)) == 1) return c;
    return -1;
}

/* ------------------------------------------------------------------- ISR */

static void uart_irq(uart_t *u)
{
    if (!u) return;
    USART_TypeDef *usart = u->cfg.instance;
    BaseType_t woken = pdFALSE;
    const uint32_t sr = usart->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        /* Reading SR then DR clears RXNE and the ORE/FE/NE error flags. */
        const uint8_t c = (uint8_t)usart->DR;
        if (sr & USART_SR_ORE) u->rx_overrun++;
        if (sr & (USART_SR_FE | USART_SR_NE)) {
            u->rx_framing++;
        } else if (sr & USART_SR_RXNE) {
            u->rx_bytes++;
            if (xStreamBufferSendFromISR(u->rx, &c, 1, &woken) != 1) u->rx_dropped++;
        }
    }

    if ((usart->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        uint8_t c;
        if (xStreamBufferReceiveFromISR(u->tx, &c, 1, &woken) == 1) {
            usart->DR = c;
        } else {
            usart->CR1 &= ~USART_CR1_TXEIE;  /* nothing left to send */
        }
    }

    portYIELD_FROM_ISR(woken);
}

void USART1_IRQHandler(void) { uart_irq(s_ports[0]); }
void USART2_IRQHandler(void) { uart_irq(s_ports[1]); }
void USART3_IRQHandler(void) { uart_irq(s_ports[2]); }
