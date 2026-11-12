/*
 * Host-side unit tests for the hardware-independent modules:
 * azel.c (geodetic -> az/el), pid.c (controller), rocket_can.c (CAN GPS parsing).
 *
 *   make -C tests        (needs any desktop gcc)
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "azel.h"
#include "pid.h"
#include "rocket_can.h"

static int g_fail, g_pass;

#define CHECK(cond) do { if (cond) g_pass++; else { g_fail++; \
    printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a = (a), _b = (b); \
    if (fabs(_a - _b) <= (tol)) g_pass++; else { g_fail++; \
    printf("  FAIL %s:%d  %s = %.6f, expected %.6f +/- %g\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); } } while (0)

/* The float-precision port that was in the repo before this rebuild,
 * compiled from tests/azel_float_legacy.c, kept only to measure its error. */
bool azel_compute_float_legacy(double obs_lat, double obs_lon, double obs_h,
                               double tgt_lat, double tgt_lon, double tgt_h,
                               double *az, double *el);

static double ang_diff(double a, double b)
{
    double d = fmod(a - b + 540.0, 360.0) - 180.0;
    return fabs(d);
}

/* ------------------------------------------------------------------ azel */
static void test_azel(void)
{
    puts("azel");
    AzEl r;
    GeoPos obs = { 43.4723, -80.5449, 300.0 };

    /* Waterloo -> CN Tower. Reference azimuth/distance from an independent
     * Vincenty inverse solution (WGS-84): 78.16738 deg, 95443.6 m. */
    GeoPos cn = { 43.6426, -79.3871, 553.3 };
    CHECK(azel_compute(&obs, &cn, &r));
    CHECK_NEAR(r.az_deg, 78.16738, 0.01);
    CHECK_NEAR(r.range_m, 95443.6, 60.0);   /* chord + height vs geodesic */
    /* Over 95 km the curvature drop (~715 m) beats the 253 m height gain. */
    CHECK_NEAR(r.el_deg, -0.277, 0.02);

    /* Cardinal directions, 1 km out at the same height. */
    const double dlat_1km = 1000.0 / 111132.0;
    const double dlon_1km = 1000.0 / (111320.0 * cos(43.4723 * M_PI / 180.0));
    GeoPos n = { obs.lat_deg + dlat_1km, obs.lon_deg, obs.h_m };
    GeoPos e = { obs.lat_deg, obs.lon_deg + dlon_1km, obs.h_m };
    GeoPos s = { obs.lat_deg - dlat_1km, obs.lon_deg, obs.h_m };
    GeoPos w = { obs.lat_deg, obs.lon_deg - dlon_1km, obs.h_m };
    azel_compute(&obs, &n, &r); CHECK(ang_diff(r.az_deg, 0.0)   < 0.05);
    azel_compute(&obs, &e, &r); CHECK(ang_diff(r.az_deg, 90.0)  < 0.05);
    azel_compute(&obs, &s, &r); CHECK(ang_diff(r.az_deg, 180.0) < 0.05);
    azel_compute(&obs, &w, &r); CHECK(ang_diff(r.az_deg, 270.0) < 0.05);
    CHECK(r.az_deg >= 0.0 && r.az_deg < 360.0);

    /* Straight up. */
    GeoPos up = { obs.lat_deg, obs.lon_deg, obs.h_m + 3000.0 };
    azel_compute(&obs, &up, &r);
    CHECK_NEAR(r.el_deg, 90.0, 0.01);
    CHECK_NEAR(r.range_m, 3000.0, 0.01);

    /* Typical launch geometry: rocket 1.5 km downrange, 3 km up -> ~63.4 deg. */
    GeoPos apogee = { obs.lat_deg + 1.5 * dlat_1km, obs.lon_deg, obs.h_m + 3000.0 };
    azel_compute(&obs, &apogee, &r);
    CHECK_NEAR(r.el_deg, atan2(3000.0, 1500.0) * 180.0 / M_PI, 0.1);

    CHECK(!azel_compute(NULL, &cn, &r));
    CHECK(!azel_compute(&obs, &cn, NULL));

    /* Float vs double: the stationary-rocket test geometry (~150 m away). */
    GeoPos pad = { obs.lat_deg + 0.00100, obs.lon_deg + 0.00130, obs.h_m + 2.0 };
    AzEl ref;
    azel_compute(&obs, &pad, &ref);
    double az_f, el_f;
    azel_compute_float_legacy(obs.lat_deg, obs.lon_deg, obs.h_m,
                              pad.lat_deg, pad.lon_deg, pad.h_m, &az_f, &el_f);
    printf("  info: pad at %.0f m  double az %.3f el %.3f | float az %.3f el %.3f "
           "(error %.2f / %.2f deg)\n",
           ref.range_m, ref.az_deg, ref.el_deg, az_f, el_f,
           ang_diff(az_f, ref.az_deg), fabs(el_f - ref.el_deg));
}

/* ------------------------------------------------------------------- pid */
static void test_pid(void)
{
    puts("pid");
    PID p;
    const double dt = 0.02;
    PID_Init(&p, 0.6, 0.8, 0.02, dt, 0.05, -20.0, 20.0, 1.0, 2.0);

    /* No derivative kick on the first sample. */
    double u0 = PID_Update(&p, 10.0, 0.0);
    CHECK_NEAR(u0, 0.6 * 10.0 + 0.8 * dt * 10.0, 1e-9);

    /* Saturation always respected. */
    PID_Reset(&p);
    for (int i = 0; i < 500; i++) {
        double u = PID_Update(&p, 1000.0, 0.0);
        CHECK(u <= 20.0 && u >= -20.0);
    }
    /* Anti-windup: with back-calculation the integrator settles where
     * Ki*e = Kb*(u_unsat - u) instead of growing for as long as the error lasts. */
    double i_500 = p.integrator;
    for (int i = 0; i < 500; i++) PID_Update(&p, 1000.0, 0.0);
    CHECK_NEAR(p.integrator, i_500, 1e-6);
    /* fixed point (I is stored before this step adds Ki*dt*e): u_max + Ki*e/Kb - P - Ki*dt*e */
    CHECK_NEAR(p.integrator, 20.0 + 0.8 * 1000.0 / 2.0 - 0.6 * 1000.0 - 0.8 * dt * 1000.0, 1e-3);

    /* Closed loop against the tracker's actual structure:
     *   servo command = setpoint + PID correction
     *   servo is a first-order lag (T = 0.15 s) with a 6 deg mounting offset
     * The integrator has to cancel the offset; with these gains the error
     * decays with time constant (1+Kp)/Ki = 2 s. */
    PID_Reset(&p);
    double y = 0.0, sp = 30.0, offset = -6.0;
    for (int i = 0; i < (int)(10.0 / dt); i++) {
        double cmd = sp + PID_Update(&p, sp, y);
        y += ((cmd + offset) - y) * dt / 0.15;
    }
    CHECK_NEAR(y, sp, 0.05);

    /* Without the PID the same plant sits 6 deg off: that's the error the
     * outer loop is there to remove. */
    double y_ol = 0.0;
    for (int i = 0; i < (int)(6.0 / dt); i++) y_ol += ((sp + offset) - y_ol) * dt / 0.15;
    CHECK_NEAR(y_ol, sp + offset, 0.05);
}

/* ------------------------------------------------------------ rocket_can */
static void test_rocket_can(void)
{
    puts("rocket_can");
    can_frame_t f;

    /* ID packing matches rocketlib build_sid(PRIO_HIGH, MSG_GPS_LATITUDE, ...),
     * board type 0x07 (GPS), inst 0. */
    CHECK(rc_build_id(1, 0x0F, 0x07, 0, 0) == 0x08F1C000u);

    /* 43.4723 N = 43 deg 28.3380 min -> bytes match the GPS board's encoding. */
    rc_encode_coord(RC_MSG_GPS_LATITUDE, 43.4723, 0x1234, RC_BOARD_TYPE_GPS, &f);
    CHECK(f.id == 0x08F1C000u);
    CHECK(f.len == 7);
    CHECK(f.data[0] == 0x12 && f.data[1] == 0x34);
    CHECK(f.data[2] == 43 && f.data[3] == 28);
    CHECK(((f.data[4] << 8) | f.data[5]) == 3380);
    CHECK(f.data[6] == 'N');

    double v;
    CHECK(rc_decode_coord(&f, RC_MSG_GPS_LATITUDE, &v) == RC_OK);
    CHECK_NEAR(v, 43.4723, 1e-6);
    CHECK(rc_decode_coord(&f, RC_MSG_GPS_LONGITUDE, &v) == RC_WRONG_TYPE);

    rc_encode_coord(RC_MSG_GPS_LONGITUDE, -80.5449, 0, RC_BOARD_TYPE_GPS, &f);
    CHECK(f.data[6] == 'W');
    CHECK(rc_decode_coord(&f, RC_MSG_GPS_LONGITUDE, &v) == RC_OK);
    CHECK_NEAR(v, -80.5449, 1e-6);   /* 1e-4 arcmin resolution ~ 0.2 m */

    /* Rounding carry: 59.99999 minutes must roll into the next degree. */
    rc_encode_coord(RC_MSG_GPS_LATITUDE, 10.9999999, 0, RC_BOARD_TYPE_GPS, &f);
    CHECK(f.data[2] == 11 && f.data[3] == 0);

    /* Corrupt fields are rejected. */
    rc_encode_coord(RC_MSG_GPS_LATITUDE, 43.0, 0, RC_BOARD_TYPE_GPS, &f);
    f.data[6] = 0;            CHECK(rc_decode_coord(&f, RC_MSG_GPS_LATITUDE, &v) == RC_BAD_FIELD);
    f.data[6] = 'N'; f.data[3] = 60;  CHECK(rc_decode_coord(&f, RC_MSG_GPS_LATITUDE, &v) == RC_BAD_FIELD);
    f.data[3] = 0;   f.len = 6;       CHECK(rc_decode_coord(&f, RC_MSG_GPS_LATITUDE, &v) == RC_BAD_LEN);

    rc_encode_alt(1234.56, 0, RC_BOARD_TYPE_GPS, &f);
    CHECK(rc_decode_alt(&f, &v) == RC_OK);
    CHECK_NEAR(v, 1234.56, 1e-9);

    uint8_t ns, q;
    rc_encode_info(9, 1, 0, RC_BOARD_TYPE_GPS, &f);
    CHECK(rc_decode_info(&f, &ns, &q) == RC_OK && ns == 9 && q == 1);

    /* ---- assembler ---- */
    gps_assembler_t a;
    GeoPos fix;
    can_frame_t lat, lon, alt, info;
    rc_encode_coord(RC_MSG_GPS_LATITUDE, 43.48, 0, RC_BOARD_TYPE_GPS, &lat);
    rc_encode_coord(RC_MSG_GPS_LONGITUDE, -80.55, 0, RC_BOARD_TYPE_GPS, &lon);
    rc_encode_alt(1500.0, 0, RC_BOARD_TYPE_GPS, &alt);
    rc_encode_info(8, 1, 0, RC_BOARD_TYPE_GPS, &info);

    /* Same order the GPS board transmits: lat, lon, info, alt. */
    gps_asm_init(&a, RC_BOARD_TYPE_GPS, 500);
    CHECK(!gps_asm_feed(&a, &lat, 1000, &fix));
    CHECK(!gps_asm_feed(&a, &lon, 1002, &fix));
    CHECK(!gps_asm_feed(&a, &info, 1003, &fix));
    CHECK(gps_asm_feed(&a, &alt, 1004, &fix));
    CHECK_NEAR(fix.lat_deg, 43.48, 1e-6);
    CHECK_NEAR(fix.lon_deg, -80.55, 1e-6);
    CHECK_NEAR(fix.h_m, 1500.0, 1e-6);
    /* A lone repeated alt must not re-emit the consumed set. */
    CHECK(!gps_asm_feed(&a, &alt, 1010, &fix));

    /* Stale part: lat from 2 s ago can't pair with a new lon/alt. */
    gps_asm_init(&a, RC_BOARD_TYPE_GPS, 500);
    gps_asm_feed(&a, &lat, 1000, &fix);
    gps_asm_feed(&a, &lon, 3000, &fix);
    CHECK(!gps_asm_feed(&a, &alt, 3001, &fix));
    CHECK(gps_asm_feed(&a, &lat, 3002, &fix));    /* fresh lat completes it */

    /* Fix quality 0 (no satellite fix) is dropped. */
    can_frame_t nofix;
    rc_encode_info(0, 0, 0, RC_BOARD_TYPE_GPS, &nofix);
    gps_asm_init(&a, RC_BOARD_TYPE_GPS, 500);
    gps_asm_feed(&a, &nofix, 10, &fix);
    gps_asm_feed(&a, &lat, 10, &fix);
    gps_asm_feed(&a, &lon, 10, &fix);
    CHECK(!gps_asm_feed(&a, &alt, 10, &fix));
    CHECK(a.fixes_rejected == 1);

    /* Frames from another board type are ignored when filtering. */
    can_frame_t other;
    rc_encode_coord(RC_MSG_GPS_LATITUDE, 10.0, 0, 0x08, &other);
    gps_asm_init(&a, RC_BOARD_TYPE_GPS, 500);
    gps_asm_feed(&a, &other, 0, &fix);
    CHECK(!a.have_lat);

    /* Timer wrap-around. */
    gps_asm_init(&a, RC_BOARD_TYPE_GPS, 500);
    gps_asm_feed(&a, &lat, 0xFFFFFF00u, &fix);
    gps_asm_feed(&a, &lon, 0xFFFFFFF0u, &fix);
    CHECK(gps_asm_feed(&a, &alt, 0x00000010u, &fix));
}

int main(void)
{
    test_azel();
    test_pid();
    test_rocket_can();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
