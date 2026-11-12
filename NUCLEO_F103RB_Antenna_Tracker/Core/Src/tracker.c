#include "tracker.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app_config.h"
#include "azel.h"
#include "board_util.h"
#include "bno055.h"
#include "can_bus.h"
#include "i2c_drv.h"
#include "pid.h"
#include "rocket_can.h"
#include "servo.h"
#include "uart_drv.h"

extern TIM_HandleTypeDef htim3;
extern osThreadId_t controlHandle;   /* CubeMX-generated tasks (main.c) */
extern osThreadId_t blink01Handle;

/* ======================================================================= */
/*  Shared state                                                            */
/* ======================================================================= */

typedef enum { MODE_TRACK, MODE_MANUAL } track_mode_t;

typedef struct {
    AzEl     look;        /* az/el/range from the antenna to the rocket */
    GeoPos   rocket;
    uint32_t tick;        /* kernel tick when the fix completed */
} target_msg_t;

/* Written by console / button handling, read by the control and telemetry
 * tasks. Guarded by s_lock. */
typedef struct {
    GeoPos       observer;
    track_mode_t mode;
    double       manual_az, manual_el;
    double       mount_heading;           /* true heading of pan-servo centre */
    double       gains[2][3];             /* [az/el][kp/ki/kd] */
    bool         gains_dirty;
} settings_t;

/* Published by the control task once per cycle. Guarded by s_lock. */
typedef struct {
    bool         imu_ok;
    uint8_t      imu_calib;
    uint32_t     imu_errors;
    double       meas_az, meas_el;        /* true azimuth / elevation from the IMU */
    double       sp_az, sp_el;            /* absolute setpoints after limits + slew */
    double       u_az, u_el;              /* PID corrections */
    double       servo_az, servo_el;      /* servo angles written */
    bool         have_target;
    target_msg_t target;
    bool         az_limited;
    uint32_t     loop_overruns;
} status_t;

typedef struct {
    uint32_t last_us, max_us, count;
    uint64_t sum_us;
} latency_t;

static settings_t s_set;
static status_t   s_stat;
static osMutexId_t s_lock;

static osMessageQueueId_t s_can_q;      /* CAN ISR -> telemetry task */
static osMessageQueueId_t s_btn_q;      /* EXTI ISR -> control task */
static QueueHandle_t      s_target_q;   /* telemetry -> control, 1-slot overwrite mailbox */

static uart_t          s_con;
static volatile bool   s_con_ready;
static i2c_bus_t       s_i2c;
static volatile bool   s_i2c_ready;
static servo_t         s_srv_az, s_srv_el;
static gps_assembler_t s_gps;
static volatile bool   s_can_ok;
static latency_t       s_lat;           /* CAN ISR -> target posted, written by telemetry task */
static volatile uint32_t s_led_period_ms = 500;

static osThreadId_t s_telem_thread, s_console_thread;

#define LOCK()   osMutexAcquire(s_lock, osWaitForever)
#define UNLOCK() osMutexRelease(s_lock)

/* ======================================================================= */
/*  Helpers                                                                 */
/* ======================================================================= */

static double wrap360(double d) { d = fmod(d, 360.0); return d < 0.0 ? d + 360.0 : d; }
static double wrap180(double d) { d = wrap360(d); return d > 180.0 ? d - 360.0 : d; }
static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* newlib-nano's printf has no %f; format fixed-point by hand. */
static char *fx(char *buf, size_t n, double v, int dec)
{
    static const long p10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000 };
    if (dec < 0) dec = 0;
    if (dec > 6) dec = 6;
    const bool neg = v < 0.0;
    if (neg) v = -v;
    if (!(v * (double)p10[dec] < 2.0e9)) { snprintf(buf, n, "%s", neg ? "-ovf" : "ovf"); return buf; }
    const long scaled = lround(v * (double)p10[dec]);
    const long ip = scaled / p10[dec], fp = scaled % p10[dec];
    if (dec) snprintf(buf, n, "%s%ld.%0*ld", neg ? "-" : "", ip, dec, fp);
    else     snprintf(buf, n, "%s%ld", neg ? "-" : "", ip);
    return buf;
}

static void cprintf(const char *fmt, ...)
{
    if (!s_con_ready) return;
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) uart_write(&s_con, buf, (size_t)(n < (int)sizeof buf ? n : (int)sizeof buf - 1), 200);
}

/* ======================================================================= */
/*  Init                                                                    */
/* ======================================================================= */

void tracker_hw_init(void)
{
    dwt_cycle_counter_enable();
    /* TIM3 runs at 1 MHz with a 20 ms period (MX_TIM3_Init). Start both
     * servos at the park position so power-up doesn't swing the antenna. */
    servo_init(&s_srv_az, &htim3, TIM_CHANNEL_1, SERVO_MIN_US, SERVO_MAX_US, SERVO_RANGE_DEG,
               AZ_SERVO_CENTER_DEG);
    servo_init(&s_srv_el, &htim3, TIM_CHANNEL_2, SERVO_MIN_US, SERVO_MAX_US, SERVO_RANGE_DEG,
               EL_SERVO_HORIZON_DEG + EL_SERVO_DIR * PARK_EL_DEG);
}

static void telemetry_task(void *arg);
static void console_task(void *arg);

void tracker_rtos_init(void)
{
    s_lock = osMutexNew(NULL);

    s_set.observer.lat_deg = OBS_DEFAULT_LAT_DEG;
    s_set.observer.lon_deg = OBS_DEFAULT_LON_DEG;
    s_set.observer.h_m     = OBS_DEFAULT_ALT_M;
    s_set.mode             = MODE_TRACK;
    s_set.manual_az        = AZ_MOUNT_HEADING_DEG;
    s_set.manual_el        = PARK_EL_DEG;
    s_set.mount_heading    = AZ_MOUNT_HEADING_DEG;
    const double g[2][3] = { { AZ_KP, AZ_KI, AZ_KD }, { EL_KP, EL_KI, EL_KD } };
    memcpy(s_set.gains, g, sizeof g);

    s_can_q    = osMessageQueueNew(CAN_RX_QUEUE_LEN, sizeof(can_frame_t), NULL);
    s_btn_q    = osMessageQueueNew(4, sizeof(uint32_t), NULL);
    s_target_q = xQueueCreate(1, sizeof(target_msg_t));

    const osThreadAttr_t telem_attr = {
        .name = "telemetry", .stack_size = 1280, .priority = osPriorityHigh,
    };
    const osThreadAttr_t console_attr = {
        .name = "console", .stack_size = 1792, .priority = osPriorityBelowNormal,
    };
    s_telem_thread   = osThreadNew(telemetry_task, NULL, &telem_attr);
    s_console_thread = osThreadNew(console_task, NULL, &console_attr);
}

/* ======================================================================= */
/*  Telemetry task: CAN frames -> GPS fix -> az/el target                   */
/* ======================================================================= */

static void telemetry_task(void *arg)
{
    (void)arg;
    const can_cfg_t cfg = {
        .prescaler = CAN_PRESCALER, .bs1_tq = CAN_BS1_TQ, .bs2_tq = CAN_BS2_TQ,
        .sjw_tq = CAN_SJW_TQ, .loopback = CAN_LOOPBACK,
    };
    s_can_ok = can_bus_init(&cfg, s_can_q);
    gps_asm_init(&s_gps, GPS_SOURCE_BOARD_TYPE, GPS_FIX_WINDOW_MS);

    for (;;) {
        can_frame_t f;
        /* While CAN is down (e.g. transceiver unplugged at boot), wake every
         * 2 s to retry; `sim` frames still arrive through the queue meanwhile. */
        if (osMessageQueueGet(s_can_q, &f, NULL, s_can_ok ? osWaitForever : 2000u) != osOK) {
            if (!s_can_ok && (s_can_ok = can_bus_init(&cfg, s_can_q))) cprintf("\r\n[can] up\r\n");
            continue;
        }

        GeoPos fix;
        if (!gps_asm_feed(&s_gps, &f, osKernelGetTickCount(), &fix)) continue;

        LOCK();
        const GeoPos obs = s_set.observer;
        UNLOCK();

        target_msg_t t;
        t.rocket = fix;
        t.tick = osKernelGetTickCount();
        if (!azel_compute(&obs, &fix, &t.look)) continue;
        xQueueOverwrite(s_target_q, &t);

        /* Interrupt-to-target latency for the frame that completed the fix. */
        if (f.rx_cycles) {
            const uint32_t us = cycles_to_us(dwt_cycles() - f.rx_cycles);
            s_lat.last_us = us;
            if (us > s_lat.max_us) s_lat.max_us = us;
            s_lat.sum_us += us;
            s_lat.count++;
        }
    }
}

/* ======================================================================= */
/*  Control task: 50 Hz closed loop                                         */
/* ======================================================================= */

static bool imu_try_init(void)
{
    uint8_t id = 0;
    /* Cheap probe first so a missing IMU doesn't stall the loop for a second. */
    if (!bno055_read_chip_id(&s_i2c, IMU_I2C_ADDR, &id) || id != BNO055_CHIP_ID_VALUE) return false;
    return bno055_init(&s_i2c, IMU_I2C_ADDR, IMU_AXIS_MAP_CONFIG, IMU_AXIS_MAP_SIGN);
}

void tracker_control_task(void *argument)
{
    (void)argument;
    const double dt = CONTROL_PERIOD_MS / 1000.0;

    const i2c_cfg_t icfg = {
        .instance = I2C1, .speed_hz = IMU_I2C_SPEED_HZ,
        .scl_port = GPIOB, .scl_pin = GPIO_PIN_8,
        .sda_port = GPIOB, .sda_pin = GPIO_PIN_9,
        .remap = true, .irq_prio = IRQ_PRIO_I2C,
    };
    s_i2c_ready = i2c_init(&s_i2c, &icfg);

    PID pid_az, pid_el;
    PID_Init(&pid_az, AZ_KP, AZ_KI, AZ_KD, dt, PID_TAU_S, -PID_OUT_LIMIT_DEG, PID_OUT_LIMIT_DEG, PID_BETA, PID_KB);
    PID_Init(&pid_el, EL_KP, EL_KI, EL_KD, dt, PID_TAU_S, -PID_OUT_LIMIT_DEG, PID_OUT_LIMIT_DEG, PID_BETA, PID_KB);

    bool imu_ok = false;
    uint32_t imu_errors = 0, imu_fail_streak = 0, imu_next_try = 0, overruns = 0;
    bno055_euler_t eul = {0};

    bool have_target = false;
    target_msg_t target = {0};

    /* Rate-limited setpoints, relative to the mount (az) / horizon (el). */
    double sp_az_rel = 0.0, sp_el = PARK_EL_DEG;

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        const uint32_t now = osKernelGetTickCount();
        if ((uint32_t)(now - last_wake) > CONTROL_PERIOD_MS) overruns++;

        /* ---- IMU: (re)initialise, then read ---------------------------- */
        if (!imu_ok && s_i2c_ready && (int32_t)(now - imu_next_try) >= 0) {
            imu_ok = imu_try_init();
            imu_next_try = now + 5000;
            imu_fail_streak = 0;
            cprintf(imu_ok ? "\r\n[imu] BNO055 ready\r\n" : "\r\n[imu] BNO055 not found, open loop\r\n");
            PID_Reset(&pid_az);
            PID_Reset(&pid_el);
        }
        if (imu_ok) {
            if (bno055_read_euler(&s_i2c, IMU_I2C_ADDR, &eul)) {
                imu_fail_streak = 0;
            } else {
                imu_errors++;
                if (++imu_fail_streak >= 10) {
                    imu_ok = false;            /* fall back to open loop, retry later */
                    cprintf("\r\n[imu] lost, open loop\r\n");
                }
            }
        }

        /* ---- inputs ----------------------------------------------------- */
        target_msg_t t;
        if (xQueueReceive(s_target_q, &t, 0) == pdTRUE) { target = t; have_target = true; }

        bool zero_request = false;
        uint32_t ev;
        while (osMessageQueueGet(s_btn_q, &ev, NULL, 0) == osOK) zero_request = true;

        LOCK();
        const track_mode_t mode = s_set.mode;
        const double mount = s_set.mount_heading;
        const double man_az = s_set.manual_az, man_el = s_set.manual_el;
        if (s_set.gains_dirty) {
            PID_SetGains(&pid_az, s_set.gains[0][0], s_set.gains[0][1], s_set.gains[0][2]);
            PID_SetGains(&pid_el, s_set.gains[1][0], s_set.gains[1][1], s_set.gains[1][2]);
            s_set.gains_dirty = false;
        }
        UNLOCK();

        /* ---- where do we want to point? -------------------------------- */
        double want_az, want_el;
        const bool fresh = have_target && (uint32_t)(now - target.tick) < GPS_STALE_MS;
        if (mode == MODE_MANUAL) {
            want_az = man_az;
            want_el = man_el;
        } else if (have_target) {
            want_az = target.look.az_deg;    /* stale: keep pointing at the last fix */
            want_el = target.look.el_deg;
        } else {
            want_az = mount;
            want_el = PARK_EL_DEG;
        }

        double rel = wrap180(want_az - mount);
        const bool az_limited = fabs(rel) > AZ_TRAVEL_DEG;
        rel = clampd(rel, -AZ_TRAVEL_DEG, AZ_TRAVEL_DEG);
        want_el = clampd(want_el, EL_MIN_DEG, EL_MAX_DEG);

        /* Slew limit so a new fix doesn't slam the servos. */
        const double step = SERVO_MAX_RATE_DEG_S * dt;
        sp_az_rel += clampd(rel - sp_az_rel, -step, step);
        sp_el     += clampd(want_el - sp_el, -step, step);

        /* ---- closed loop ------------------------------------------------ */
        double meas_az = 0.0, meas_el = 0.0, u_az = 0.0, u_el = 0.0;
        if (imu_ok) {
            meas_az = wrap360(eul.heading_deg + MAG_DECLINATION_DEG);
            meas_el = IMU_EL_SIGN * eul.pitch_deg + IMU_EL_OFFSET_DEG;
            const double meas_az_rel = wrap180(meas_az - mount);

            u_el = PID_Update(&pid_el, sp_el, meas_el);
            /* Heading from Euler angles is meaningless near vertical:
             * run azimuth feed-forward only up there. */
            if (meas_el < 80.0) {
                u_az = PID_Update(&pid_az, sp_az_rel, meas_az_rel);
            } else {
                PID_Reset(&pid_az);
            }
        } else {
            PID_Reset(&pid_az);
            PID_Reset(&pid_el);
        }

        /* Feed-forward (setpoint) + PID correction -> servo angle. */
        const double srv_az = AZ_SERVO_CENTER_DEG + AZ_SERVO_DIR * (sp_az_rel + u_az);
        const double srv_el = EL_SERVO_HORIZON_DEG + EL_SERVO_DIR * (sp_el + u_el);
        servo_write_deg(&s_srv_az, srv_az);
        servo_write_deg(&s_srv_el, srv_el);

        /* ---- button / `zero`: re-measure the mount heading -------------- *
         * Where the IMU says the antenna points, minus where the pan servo
         * thinks it is relative to centre. */
        if (zero_request) {
            if (imu_ok) {
                const double servo_rel = (s_srv_az.angle_deg - AZ_SERVO_CENTER_DEG) / AZ_SERVO_DIR;
                const double new_mount = wrap360(meas_az - servo_rel);
                LOCK();
                s_set.mount_heading = new_mount;
                UNLOCK();
                sp_az_rel = wrap180(sp_az_rel + mount - new_mount);
                PID_Reset(&pid_az);
                char a[16];
                cprintf("\r\n[zero] mount heading = %s deg\r\n", fx(a, sizeof a, new_mount, 1));
            } else {
                cprintf("\r\n[zero] needs the IMU\r\n");
            }
        }

        /* ---- publish ---------------------------------------------------- */
        LOCK();
        s_stat.imu_ok = imu_ok;
        s_stat.imu_calib = eul.calib;
        s_stat.imu_errors = imu_errors;
        s_stat.meas_az = meas_az;
        s_stat.meas_el = meas_el;
        s_stat.sp_az = wrap360(sp_az_rel + mount);
        s_stat.sp_el = sp_el;
        s_stat.u_az = u_az;
        s_stat.u_el = u_el;
        s_stat.servo_az = s_srv_az.angle_deg;
        s_stat.servo_el = s_srv_el.angle_deg;
        s_stat.have_target = have_target;
        s_stat.target = target;
        s_stat.az_limited = az_limited;
        s_stat.loop_overruns = overruns;
        UNLOCK();

        s_led_period_ms = !imu_ok ? 1000u : (fresh ? 100u : 500u);
    }
}

void tracker_led_task(void *argument)
{
    (void)argument;
    for (;;) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        osDelay(s_led_period_ms);
    }
}

/* B1 (PC13) EXTI: debounce, then hand off to the control task. */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    static uint32_t last;
    if (pin != B1_Pin || !s_btn_q) return;
    const uint32_t now = osKernelGetTickCount();
    if ((uint32_t)(now - last) < 250u) return;
    last = now;
    (void)osMessageQueuePut(s_btn_q, &now, 0, 0);
}

/* ======================================================================= */
/*  Console task                                                            */
/* ======================================================================= */

static bool parse_d(const char *s, double *out)
{
    if (!s) return false;
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end) return false;
    *out = v;
    return true;
}

static void cmd_help(void)
{
    cprintf("commands:\r\n"
            "  status                     tracker, IMU, CAN and latency state\r\n"
            "  obs <lat> <lon> <alt_m>    set antenna position\r\n");
    cprintf("  track | point <az> <el>    auto tracking / manual pointing\r\n"
            "  zero                       re-measure mount heading (same as blue button)\r\n"
            "  gains <az|el> <kp> <ki> <kd>\r\n");
    cprintf("  sim <lat> <lon> <alt_m> [bus]  inject a GPS fix as CAN frames\r\n"
            "                             (queue path by default, 'bus' = transmit on CAN)\r\n"
            "  i2ctest [n]                n IMU chip-id reads, reports packet loss\r\n");
    cprintf("  tasks                      stack high-water marks\r\n"
            "  reset\r\n");
}

static void cmd_status(void)
{
    LOCK();
    const status_t st = s_stat;
    const settings_t se = s_set;
    UNLOCK();
    const can_stats_t *cs = can_bus_stats();
    char a[16], b[16], c[16], d[16];

    cprintf("mode %s  observer %s, %s, %s m  mount %s deg\r\n",
            se.mode == MODE_TRACK ? "TRACK" : "MANUAL",
            fx(a, sizeof a, se.observer.lat_deg, 6), fx(b, sizeof b, se.observer.lon_deg, 6),
            fx(c, sizeof c, se.observer.h_m, 1), fx(d, sizeof d, se.mount_heading, 1));
    if (st.have_target) {
        cprintf("target az %s el %s range %s m  age %lu ms%s\r\n",
                fx(a, sizeof a, st.target.look.az_deg, 2), fx(b, sizeof b, st.target.look.el_deg, 2),
                fx(c, sizeof c, st.target.look.range_m, 1),
                (unsigned long)(osKernelGetTickCount() - st.target.tick),
                st.az_limited ? "  (outside az travel)" : "");
        cprintf("rocket  %s, %s, %s m\r\n",
                fx(a, sizeof a, st.target.rocket.lat_deg, 6), fx(b, sizeof b, st.target.rocket.lon_deg, 6),
                fx(c, sizeof c, st.target.rocket.h_m, 1));
    } else {
        cprintf("target none (parked)\r\n");
    }
    cprintf("setpt   az %s el %s   servo az %s el %s deg\r\n",
            fx(a, sizeof a, st.sp_az, 2), fx(b, sizeof b, st.sp_el, 2),
            fx(c, sizeof c, st.servo_az, 1), fx(d, sizeof d, st.servo_el, 1));
    if (st.imu_ok) {
        cprintf("imu     az %s el %s   pid u_az %s u_el %s  calib s%u g%u a%u m%u\r\n",
                fx(a, sizeof a, st.meas_az, 2), fx(b, sizeof b, st.meas_el, 2),
                fx(c, sizeof c, st.u_az, 2), fx(d, sizeof d, st.u_el, 2),
                (st.imu_calib >> 6) & 3u, (st.imu_calib >> 4) & 3u, (st.imu_calib >> 2) & 3u, st.imu_calib & 3u);
    } else {
        cprintf("imu     offline (open loop)  read errors %lu\r\n", (unsigned long)st.imu_errors);
    }
    const uint32_t esr = can_bus_esr();
    cprintf("can     %s%s  rx %lu dropped %lu fifo_ovr %lu  err %lu busoff %lu  TEC %lu REC %lu\r\n",
            s_can_ok ? "up" : "INIT FAILED", CAN_LOOPBACK ? " (loopback)" : "",
            (unsigned long)cs->rx, (unsigned long)cs->rx_dropped, (unsigned long)cs->fifo_overrun,
            (unsigned long)cs->errors, (unsigned long)cs->bus_off,
            (unsigned long)((esr >> 16) & 0xFFu), (unsigned long)(esr >> 24));
    cprintf("gps     frames ok %lu bad %lu  fixes %lu rejected %lu  sats %u q %u\r\n",
            (unsigned long)s_gps.frames_ok, (unsigned long)s_gps.frames_rejected,
            (unsigned long)s_gps.fixes, (unsigned long)s_gps.fixes_rejected,
            s_gps.num_sat, s_gps.quality);
    const uint32_t n = s_lat.count;
    cprintf("latency CAN ISR -> target: last %lu us  avg %lu us  max %lu us  (n=%lu)\r\n",
            (unsigned long)s_lat.last_us, (unsigned long)(n ? (uint32_t)(s_lat.sum_us / n) : 0u),
            (unsigned long)s_lat.max_us, (unsigned long)n);
    cprintf("loop    overruns %lu\r\n", (unsigned long)st.loop_overruns);
}

static void cmd_sim(char **argv, int argc)
{
    double lat, lon, alt;
    if (argc < 4 || !parse_d(argv[1], &lat) || !parse_d(argv[2], &lon) || !parse_d(argv[3], &alt)) {
        cprintf("usage: sim <lat> <lon> <alt_m> [bus]\r\n");
        return;
    }
    const bool on_bus = (argc >= 5 && strcmp(argv[4], "bus") == 0);
    const uint8_t board = GPS_SOURCE_BOARD_TYPE ? GPS_SOURCE_BOARD_TYPE : RC_BOARD_TYPE_GPS;
    const uint16_t ts = (uint16_t)osKernelGetTickCount();

    can_frame_t f[4];   /* same order the GPS board sends: lat, lon, info, alt */
    rc_encode_coord(RC_MSG_GPS_LATITUDE, lat, ts, board, &f[0]);
    rc_encode_coord(RC_MSG_GPS_LONGITUDE, lon, ts, board, &f[1]);
    rc_encode_info(8, 1, ts, board, &f[2]);
    rc_encode_alt(alt, ts, board, &f[3]);

    for (int i = 0; i < 4; i++) {
        if (on_bus) {
            if (!can_bus_send(&f[i])) { cprintf("tx failed (mailboxes full / bus down)\r\n"); return; }
            osDelay(2);
        } else {
            f[i].rx_cycles = dwt_cycles();
            if (osMessageQueuePut(s_can_q, &f[i], 0, 10) != osOK) { cprintf("queue full\r\n"); return; }
        }
    }
    cprintf("sent 4 frames (%s)\r\n", on_bus ? "CAN bus" : "rx queue");
}

static void cmd_i2ctest(char **argv, int argc)
{
    double nd = 1000;
    if (argc >= 2 && (!parse_d(argv[1], &nd) || nd < 1 || nd > 100000)) {
        cprintf("usage: i2ctest [1..100000]\r\n");
        return;
    }
    if (!s_i2c_ready) { cprintf("i2c bus not initialised\r\n"); return; }
    const uint32_t n = (uint32_t)nd;
    const i2c_stats_t before = s_i2c.stats;
    uint32_t ok = 0, wrong = 0;

    cprintf("reading BNO055 CHIP_ID %lu times (control loop keeps running)...\r\n", (unsigned long)n);
    const uint32_t t0 = osKernelGetTickCount();
    for (uint32_t i = 0; i < n; i++) {
        uint8_t id = 0;
        if (bno055_read_chip_id(&s_i2c, IMU_I2C_ADDR, &id)) {
            if (id == BNO055_CHIP_ID_VALUE) ok++; else wrong++;
        }
    }
    const uint32_t ms = osKernelGetTickCount() - t0;
    const i2c_stats_t after = s_i2c.stats;
    const uint32_t failed = n - ok;
    char a[16];

    cprintf("ok %lu  failed %lu  loss %s %%  (wrong data %lu)\r\n",
            (unsigned long)ok, (unsigned long)failed,
            fx(a, sizeof a, 100.0 * (double)failed / (double)n, 3), (unsigned long)wrong);
    cprintf("bus-wide during test: nack %lu timeout %lu bus_err %lu busy %lu recoveries %lu\r\n",
            (unsigned long)(after.nack - before.nack), (unsigned long)(after.timeout - before.timeout),
            (unsigned long)(after.bus_err - before.bus_err), (unsigned long)(after.busy - before.busy),
            (unsigned long)(after.recoveries - before.recoveries));
    cprintf("%lu ms, %lu transfers/s\r\n", (unsigned long)ms,
            (unsigned long)(ms ? (uint32_t)((uint64_t)n * 1000u / ms) : 0u));
}

static void cmd_tasks(void)
{
    const struct { const char *name; TaskHandle_t h; } t[] = {
        { "telemetry", (TaskHandle_t)s_telem_thread },
        { "console",   (TaskHandle_t)s_console_thread },
        { "control",   (TaskHandle_t)controlHandle },
        { "blink01",   (TaskHandle_t)blink01Handle },
    };
    for (size_t i = 0; i < sizeof t / sizeof t[0]; i++) {
        if (!t[i].h) continue;
        cprintf("%-10s free stack %lu bytes\r\n", t[i].name,
                (unsigned long)(uxTaskGetStackHighWaterMark(t[i].h) * sizeof(StackType_t)));
    }
    cprintf("heap free %lu bytes (min ever %lu)\r\n",
            (unsigned long)xPortGetFreeHeapSize(), (unsigned long)xPortGetMinimumEverFreeHeapSize());
}

static void dispatch(char *line)
{
    char *argv[6];
    int argc = 0;
    char *save = NULL;
    for (char *tok = strtok_r(line, " \t", &save); tok && argc < 6; tok = strtok_r(NULL, " \t", &save)) {
        argv[argc++] = tok;
    }
    if (argc == 0) return;
    const char *cmd = argv[0];
    double a, b, c;

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        cmd_help();
    } else if (!strcmp(cmd, "status") || !strcmp(cmd, "s")) {
        cmd_status();
    } else if (!strcmp(cmd, "obs")) {
        if (argc == 4 && parse_d(argv[1], &a) && parse_d(argv[2], &b) && parse_d(argv[3], &c) &&
            fabs(a) <= 90.0 && fabs(b) <= 180.0) {
            LOCK();
            s_set.observer.lat_deg = a; s_set.observer.lon_deg = b; s_set.observer.h_m = c;
            UNLOCK();
            cprintf("observer set\r\n");
        } else {
            cprintf("usage: obs <lat> <lon> <alt_m>\r\n");
        }
    } else if (!strcmp(cmd, "point")) {
        if (argc == 3 && parse_d(argv[1], &a) && parse_d(argv[2], &b)) {
            LOCK();
            s_set.mode = MODE_MANUAL; s_set.manual_az = wrap360(a); s_set.manual_el = b;
            UNLOCK();
            cprintf("manual pointing\r\n");
        } else {
            cprintf("usage: point <az> <el>\r\n");
        }
    } else if (!strcmp(cmd, "track")) {
        LOCK(); s_set.mode = MODE_TRACK; UNLOCK();
        cprintf("tracking\r\n");
    } else if (!strcmp(cmd, "zero")) {
        uint32_t now = osKernelGetTickCount();
        osMessageQueuePut(s_btn_q, &now, 0, 0);
    } else if (!strcmp(cmd, "gains")) {
        int axis = (argc == 5) ? (!strcmp(argv[1], "az") ? 0 : (!strcmp(argv[1], "el") ? 1 : -1)) : -1;
        if (axis >= 0 && parse_d(argv[2], &a) && parse_d(argv[3], &b) && parse_d(argv[4], &c)) {
            LOCK();
            s_set.gains[axis][0] = a; s_set.gains[axis][1] = b; s_set.gains[axis][2] = c;
            s_set.gains_dirty = true;
            UNLOCK();
            cprintf("gains updated\r\n");
        } else {
            cprintf("usage: gains <az|el> <kp> <ki> <kd>\r\n");
        }
    } else if (!strcmp(cmd, "sim")) {
        cmd_sim(argv, argc);
    } else if (!strcmp(cmd, "i2ctest")) {
        cmd_i2ctest(argv, argc);
    } else if (!strcmp(cmd, "tasks")) {
        cmd_tasks();
    } else if (!strcmp(cmd, "reset")) {
        cprintf("resetting\r\n");
        osDelay(50);
        NVIC_SystemReset();
    } else {
        cprintf("unknown command '%s' (try help)\r\n", cmd);
    }
}

static void console_task(void *arg)
{
    (void)arg;
    const uart_cfg_t cfg = {
        .instance = USART2, .baud = 115200,
        .tx_port = GPIOA, .tx_pin = GPIO_PIN_2,
        .rx_port = GPIOA, .rx_pin = GPIO_PIN_3,
        .remap = false, .irq_prio = IRQ_PRIO_UART,
        .rx_buf_size = 128, .tx_buf_size = 512,
    };
    if (!uart_init(&s_con, &cfg)) {
        vTaskDelete(NULL);
    }
    s_con_ready = true;

    cprintf("\r\n\r\nantenna tracker (NUCLEO-F103RB)  type 'help'\r\n");
    cprintf("CAN %s%s, 250 kbit/s\r\n", s_can_ok ? "up" : "not up yet", CAN_LOOPBACK ? " [loopback]" : "");
    cprintf("> ");

    char line[96];
    size_t len = 0;
    for (;;) {
        const int ch = uart_getc(&s_con, osWaitForever);
        if (ch < 0) continue;
        if (ch == '\r' || ch == '\n') {
            if (ch == '\n' && len == 0) continue;   /* LF after CR */
            cprintf("\r\n");
            line[len] = '\0';
            dispatch(line);
            len = 0;
            cprintf("> ");
        } else if (ch == 0x7F || ch == '\b') {
            if (len) { len--; cprintf("\b \b"); }
        } else if (ch >= 0x20 && ch < 0x7F && len < sizeof line - 1) {
            line[len++] = (char)ch;
            const char echo = (char)ch;
            uart_write(&s_con, &echo, 1, 50);
        }
    }
}
