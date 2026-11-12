#include "bno055.h"
#include "cmsis_os.h"

/* Register map, page 0 (BNO055 datasheet section 4.2) */
#define REG_CHIP_ID          0x00u
#define REG_PAGE_ID          0x07u
#define REG_EUL_HEADING_LSB  0x1Au   /* heading, roll, pitch: 3 x int16 LE, 16 LSB/deg */
#define REG_CALIB_STAT       0x35u
#define REG_UNIT_SEL         0x3Bu
#define REG_OPR_MODE         0x3Du
#define REG_PWR_MODE         0x3Eu
#define REG_SYS_TRIGGER      0x3Fu
#define REG_AXIS_MAP_CONFIG  0x41u
#define REG_AXIS_MAP_SIGN    0x42u

#define BNO_OPR_MODE_CONFIG      0x00u
#define BNO_OPR_MODE_NDOF        0x0Cu
#define BNO_PWR_MODE_NORMAL      0x00u
#define SYS_TRIGGER_RST_SYS  0x20u

#define I2C_TIMEOUT_MS       20u

static bool wr(i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t val)
{
    return i2c_mem_write(bus, addr, reg, &val, 1, I2C_TIMEOUT_MS) == I2C_OK;
}

bool bno055_read_chip_id(i2c_bus_t *bus, uint8_t addr, uint8_t *id)
{
    return i2c_mem_read(bus, addr, REG_CHIP_ID, id, 1, I2C_TIMEOUT_MS) == I2C_OK;
}

static bool wait_for_chip(i2c_bus_t *bus, uint8_t addr, uint32_t timeout_ms)
{
    for (uint32_t t = 0; t < timeout_ms; t += 50) {
        uint8_t id = 0;
        if (bno055_read_chip_id(bus, addr, &id) && id == BNO055_CHIP_ID_VALUE) return true;
        osDelay(50);
    }
    return false;
}

bool bno055_init(i2c_bus_t *bus, uint8_t addr, uint8_t axis_map_config, uint8_t axis_map_sign)
{
    /* Power-on to I2C ready is up to ~850 ms. */
    if (!wait_for_chip(bus, addr, 1000)) return false;

    if (!wr(bus, addr, REG_OPR_MODE, BNO_OPR_MODE_CONFIG)) return false;
    osDelay(25);                                     /* any -> CONFIG: 19 ms */

    /* Full reset so we always start from a known state after a warm MCU reset. */
    wr(bus, addr, REG_SYS_TRIGGER, SYS_TRIGGER_RST_SYS);   /* NACK during reset is normal */
    osDelay(650);
    if (!wait_for_chip(bus, addr, 1000)) return false;

    bool ok = true;
    ok &= wr(bus, addr, REG_PWR_MODE, BNO_PWR_MODE_NORMAL);
    osDelay(10);
    ok &= wr(bus, addr, REG_PAGE_ID, 0x00);
    ok &= wr(bus, addr, REG_UNIT_SEL, 0x00);         /* degrees, Windows orientation */
    ok &= wr(bus, addr, REG_AXIS_MAP_CONFIG, axis_map_config);
    ok &= wr(bus, addr, REG_AXIS_MAP_SIGN, axis_map_sign);
    ok &= wr(bus, addr, REG_SYS_TRIGGER, 0x00);      /* internal oscillator */
    osDelay(10);
    ok &= wr(bus, addr, REG_OPR_MODE, BNO_OPR_MODE_NDOF);
    osDelay(20);                                     /* CONFIG -> fusion: 7 ms */
    return ok;
}

bool bno055_read_euler(i2c_bus_t *bus, uint8_t addr, bno055_euler_t *out)
{
    uint8_t b[6];
    if (i2c_mem_read(bus, addr, REG_EUL_HEADING_LSB, b, sizeof b, I2C_TIMEOUT_MS) != I2C_OK) return false;

    const int16_t h = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    const int16_t r = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
    const int16_t p = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    out->heading_deg = (double)h / 16.0;
    out->roll_deg    = (double)r / 16.0;
    out->pitch_deg   = (double)p / 16.0;

    uint8_t cal = 0;
    if (i2c_mem_read(bus, addr, REG_CALIB_STAT, &cal, 1, I2C_TIMEOUT_MS) == I2C_OK) out->calib = cal;
    return true;
}
