#include "i2c_drv.h"
#include "board_util.h"
#include <string.h>

static i2c_bus_t *s_bus[2];   /* I2C1, I2C2 -> driver instance */

static int bus_index(I2C_TypeDef *inst)
{
    if (inst == I2C1) return 0;
    if (inst == I2C2) return 1;
    return -1;
}

static i2c_bus_t *bus_from_handle(I2C_HandleTypeDef *h)
{
    int i = bus_index(h->Instance);
    return (i < 0) ? NULL : s_bus[i];
}

static void irqs(const i2c_bus_t *bus, IRQn_Type *ev, IRQn_Type *er)
{
    if (bus->cfg.instance == I2C1) { *ev = I2C1_EV_IRQn; *er = I2C1_ER_IRQn; }
    else                           { *ev = I2C2_EV_IRQn; *er = I2C2_ER_IRQn; }
}

static void short_delay(void)   /* ~5 us: half an SCL period at 100 kHz */
{
    for (volatile uint32_t i = 0; i < SystemCoreClock / 1000000u; i++) { }
}

/* Called by HAL_I2C_Init. */
void HAL_I2C_MspInit(I2C_HandleTypeDef *h)
{
    i2c_bus_t *bus = bus_from_handle(h);
    if (!bus) return;
    const i2c_cfg_t *c = &bus->cfg;

    /* Clock the peripheral before switching the pins to AF open-drain; the
     * other order can leave BUSY latched on F1 parts. If it latches anyway,
     * i2c_init() runs i2c_recover(). */
    if (c->instance == I2C1) {
        __HAL_RCC_I2C1_CLK_ENABLE();
        if (c->remap) __HAL_AFIO_REMAP_I2C1_ENABLE();
    } else {
        __HAL_RCC_I2C2_CLK_ENABLE();
    }

    GPIO_InitTypeDef g = {0};
    gpio_clk_enable(c->scl_port);
    gpio_clk_enable(c->sda_port);
    g.Mode = GPIO_MODE_AF_OD;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = c->scl_pin; HAL_GPIO_Init(c->scl_port, &g);
    g.Pin = c->sda_pin; HAL_GPIO_Init(c->sda_port, &g);

    IRQn_Type ev, er;
    irqs(bus, &ev, &er);
    HAL_NVIC_SetPriority(ev, c->irq_prio, 0);
    HAL_NVIC_SetPriority(er, c->irq_prio, 0);
    HAL_NVIC_EnableIRQ(ev);
    HAL_NVIC_EnableIRQ(er);
}

static bool hal_init(i2c_bus_t *bus)
{
    I2C_HandleTypeDef *h = &bus->hi2c;
    h->Instance = bus->cfg.instance;
    h->Init.ClockSpeed = bus->cfg.speed_hz;
    h->Init.DutyCycle = I2C_DUTYCYCLE_2;
    h->Init.OwnAddress1 = 0;
    h->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    h->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    h->Init.OwnAddress2 = 0;
    h->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    h->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(h) == HAL_OK;
}

void i2c_recover(i2c_bus_t *bus)
{
    const i2c_cfg_t *c = &bus->cfg;
    IRQn_Type ev, er;
    irqs(bus, &ev, &er);
    HAL_NVIC_DisableIRQ(ev);
    HAL_NVIC_DisableIRQ(er);
    HAL_I2C_DeInit(&bus->hi2c);

    /* Take the pins as plain open-drain GPIO. */
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_WritePin(c->scl_port, c->scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(c->sda_port, c->sda_pin, GPIO_PIN_SET);
    g.Pin = c->scl_pin; HAL_GPIO_Init(c->scl_port, &g);
    g.Pin = c->sda_pin; HAL_GPIO_Init(c->sda_port, &g);
    short_delay();

    /* A slave stuck mid-byte holds SDA low; clock it out (max 9 bits). */
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(c->sda_port, c->sda_pin) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(c->scl_port, c->scl_pin, GPIO_PIN_RESET); short_delay();
        HAL_GPIO_WritePin(c->scl_port, c->scl_pin, GPIO_PIN_SET);   short_delay();
    }

    /* STOP: SDA low -> high while SCL is high. */
    HAL_GPIO_WritePin(c->scl_port, c->scl_pin, GPIO_PIN_RESET); short_delay();
    HAL_GPIO_WritePin(c->sda_port, c->sda_pin, GPIO_PIN_RESET); short_delay();
    HAL_GPIO_WritePin(c->scl_port, c->scl_pin, GPIO_PIN_SET);   short_delay();
    HAL_GPIO_WritePin(c->sda_port, c->sda_pin, GPIO_PIN_SET);   short_delay();

    /* Software-reset the peripheral to clear a latched BUSY, then re-init
     * (HAL_I2C_MspInit puts the pins back to AF open-drain). */
    if (c->instance == I2C1) __HAL_RCC_I2C1_CLK_ENABLE(); else __HAL_RCC_I2C2_CLK_ENABLE();
    c->instance->CR1 |= I2C_CR1_SWRST;
    c->instance->CR1 &= ~I2C_CR1_SWRST;
    hal_init(bus);

    bus->stats.recoveries++;
}

bool i2c_init(i2c_bus_t *bus, const i2c_cfg_t *cfg)
{
    const int idx = bus_index(cfg->instance);
    if (idx < 0) return false;

    memset(bus, 0, sizeof(*bus));
    bus->cfg = *cfg;
    bus->lock = xSemaphoreCreateMutex();
    bus->done = xSemaphoreCreateBinary();
    if (!bus->lock || !bus->done) return false;
    s_bus[idx] = bus;

    if (!hal_init(bus)) return false;
    if (__HAL_I2C_GET_FLAG(&bus->hi2c, I2C_FLAG_BUSY)) i2c_recover(bus);
    return !__HAL_I2C_GET_FLAG(&bus->hi2c, I2C_FLAG_BUSY);
}

void i2c_stats_reset(i2c_bus_t *bus)
{
    memset(&bus->stats, 0, sizeof(bus->stats));
}

static i2c_status_t transfer(i2c_bus_t *bus, bool read, uint8_t addr7, uint8_t reg,
                             uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (xSemaphoreTake(bus->lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        bus->stats.timeout++;
        return I2C_ERR_TIMEOUT;
    }

    (void)xSemaphoreTake(bus->done, 0);       /* drop a stale completion */
    bus->hal_error = HAL_I2C_ERROR_NONE;

    const uint16_t dev = (uint16_t)(addr7 << 1);
    HAL_StatusTypeDef hs = read
        ? HAL_I2C_Mem_Read_IT(&bus->hi2c, dev, reg, I2C_MEMADD_SIZE_8BIT, buf, len)
        : HAL_I2C_Mem_Write_IT(&bus->hi2c, dev, reg, I2C_MEMADD_SIZE_8BIT, buf, len);

    i2c_status_t st;
    if (hs != HAL_OK) {
        st = I2C_ERR_BUSY;
        bus->stats.busy++;
        i2c_recover(bus);
    } else if (xSemaphoreTake(bus->done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        st = I2C_ERR_TIMEOUT;
        bus->stats.timeout++;
        i2c_recover(bus);
    } else if (bus->hal_error == HAL_I2C_ERROR_NONE) {
        st = I2C_OK;
        bus->stats.ok++;
    } else if (bus->hal_error == HAL_I2C_ERROR_AF) {
        st = I2C_ERR_NACK;                    /* HAL already sent STOP */
        bus->stats.nack++;
    } else {
        st = I2C_ERR_BUS;
        bus->stats.bus_err++;
        i2c_recover(bus);
    }

    xSemaphoreGive(bus->lock);
    return st;
}

i2c_status_t i2c_mem_read(i2c_bus_t *bus, uint8_t addr7, uint8_t reg,
                          uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    return transfer(bus, true, addr7, reg, buf, len, timeout_ms);
}

i2c_status_t i2c_mem_write(i2c_bus_t *bus, uint8_t addr7, uint8_t reg,
                           const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    return transfer(bus, false, addr7, reg, (uint8_t *)buf, len, timeout_ms);
}

/* ------------------------------------------------- HAL callbacks (ISR) */

static void signal_done(I2C_HandleTypeDef *h, uint32_t err)
{
    i2c_bus_t *bus = bus_from_handle(h);
    if (!bus) return;
    bus->hal_error = err;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(bus->done, &woken);
    portYIELD_FROM_ISR(woken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h) { signal_done(h, HAL_I2C_ERROR_NONE); }
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *h) { signal_done(h, HAL_I2C_ERROR_NONE); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h)
{
    signal_done(h, h->ErrorCode ? h->ErrorCode : HAL_I2C_ERROR_BERR);
}

void I2C1_EV_IRQHandler(void) { if (s_bus[0]) HAL_I2C_EV_IRQHandler(&s_bus[0]->hi2c); }
void I2C1_ER_IRQHandler(void) { if (s_bus[0]) HAL_I2C_ER_IRQHandler(&s_bus[0]->hi2c); }
void I2C2_EV_IRQHandler(void) { if (s_bus[1]) HAL_I2C_EV_IRQHandler(&s_bus[1]->hi2c); }
void I2C2_ER_IRQHandler(void) { if (s_bus[1]) HAL_I2C_ER_IRQHandler(&s_bus[1]->hi2c); }
