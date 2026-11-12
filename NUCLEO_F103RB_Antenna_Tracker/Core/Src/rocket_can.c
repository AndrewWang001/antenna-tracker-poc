#include "rocket_can.h"
#include <math.h>
#include <string.h>

uint32_t rc_build_id(uint8_t prio, uint8_t msg_type, uint8_t board_type,
                     uint8_t board_inst, uint8_t metadata)
{
    return ((uint32_t)(prio & 0x3u) << 27) | ((uint32_t)(msg_type & 0x7Fu) << 20) |
           ((uint32_t)(board_type & 0x3Fu) << 14) | ((uint32_t)(board_inst & 0x3Fu) << 8) |
           metadata;
}

static void put_timestamp(can_frame_t *f, uint16_t ts)
{
    f->data[0] = (uint8_t)(ts >> 8);
    f->data[1] = (uint8_t)(ts & 0xFFu);
}

/* ---------------------------------------------------------------- decode */

rc_status_t rc_decode_coord(const can_frame_t *f, uint8_t expected_type, double *deg)
{
    if (!f || !deg) return RC_BAD_FIELD;
    if (rc_msg_type(f->id) != expected_type) return RC_WRONG_TYPE;
    if (f->len != 7) return RC_BAD_LEN;

    const uint8_t  d    = f->data[2];
    const uint8_t  m    = f->data[3];
    const uint16_t frac = (uint16_t)(((uint16_t)f->data[4] << 8) | f->data[5]);
    const char     dir  = (char)f->data[6];

    const bool is_lat = (expected_type == RC_MSG_GPS_LATITUDE);
    if (d > (is_lat ? 90u : 180u) || m >= 60u || frac >= 10000u) return RC_BAD_FIELD;

    double negative;
    if (is_lat) {
        if (dir == 'N') negative = 1.0;
        else if (dir == 'S') negative = -1.0;
        else return RC_BAD_FIELD;
    } else {
        if (dir == 'E') negative = 1.0;
        else if (dir == 'W') negative = -1.0;
        else return RC_BAD_FIELD;
    }

    *deg = negative * ((double)d + ((double)m + (double)frac / 10000.0) / 60.0);
    return RC_OK;
}

rc_status_t rc_decode_alt(const can_frame_t *f, double *alt_m)
{
    if (!f || !alt_m) return RC_BAD_FIELD;
    if (rc_msg_type(f->id) != RC_MSG_GPS_ALTITUDE) return RC_WRONG_TYPE;
    if (f->len != 7) return RC_BAD_LEN;

    const uint32_t whole = ((uint32_t)f->data[2] << 24) | ((uint32_t)f->data[3] << 16) |
                           ((uint32_t)f->data[4] << 8)  |  (uint32_t)f->data[5];
    const uint8_t  hundredths = f->data[6];
    if (hundredths >= 100u) return RC_BAD_FIELD;

    *alt_m = (double)whole + (double)hundredths / 100.0;
    return RC_OK;
}

rc_status_t rc_decode_info(const can_frame_t *f, uint8_t *num_sat, uint8_t *quality)
{
    if (!f || !num_sat || !quality) return RC_BAD_FIELD;
    if (rc_msg_type(f->id) != RC_MSG_GPS_INFO) return RC_WRONG_TYPE;
    if (f->len != 4) return RC_BAD_LEN;
    *num_sat = f->data[2];
    *quality = f->data[3];
    return RC_OK;
}

/* ---------------------------------------------------------------- encode */

void rc_encode_coord(uint8_t msg_type, double deg, uint16_t ts, uint8_t board_type, can_frame_t *f)
{
    const bool is_lat = (msg_type == RC_MSG_GPS_LATITUDE);
    const bool neg = deg < 0.0;
    double a = fabs(deg);

    uint32_t d = (uint32_t)a;
    double minutes = (a - (double)d) * 60.0;
    uint32_t m = (uint32_t)minutes;
    uint32_t frac = (uint32_t)lround((minutes - (double)m) * 10000.0);
    if (frac >= 10000u) { frac -= 10000u; m++; }
    if (m >= 60u)       { m -= 60u; d++; }

    memset(f, 0, sizeof(*f));
    f->id  = rc_build_id(RC_PRIO_HIGH, msg_type, board_type, 0, 0);
    f->len = 7;
    put_timestamp(f, ts);
    f->data[2] = (uint8_t)d;
    f->data[3] = (uint8_t)m;
    f->data[4] = (uint8_t)(frac >> 8);
    f->data[5] = (uint8_t)(frac & 0xFFu);
    f->data[6] = (uint8_t)(is_lat ? (neg ? 'S' : 'N') : (neg ? 'W' : 'E'));
}

void rc_encode_alt(double alt_m, uint16_t ts, uint8_t board_type, can_frame_t *f)
{
    if (alt_m < 0.0) alt_m = 0.0;      /* wire format is unsigned */
    uint32_t whole = (uint32_t)alt_m;
    uint32_t hundredths = (uint32_t)lround((alt_m - (double)whole) * 100.0);
    if (hundredths >= 100u) { hundredths -= 100u; whole++; }

    memset(f, 0, sizeof(*f));
    f->id  = rc_build_id(RC_PRIO_HIGH, RC_MSG_GPS_ALTITUDE, board_type, 0, 0);
    f->len = 7;
    put_timestamp(f, ts);
    f->data[2] = (uint8_t)(whole >> 24);
    f->data[3] = (uint8_t)(whole >> 16);
    f->data[4] = (uint8_t)(whole >> 8);
    f->data[5] = (uint8_t)whole;
    f->data[6] = (uint8_t)hundredths;
}

void rc_encode_info(uint8_t num_sat, uint8_t quality, uint16_t ts, uint8_t board_type, can_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id  = rc_build_id(RC_PRIO_HIGH, RC_MSG_GPS_INFO, board_type, 0, 0);
    f->len = 4;
    put_timestamp(f, ts);
    f->data[2] = num_sat;
    f->data[3] = quality;
}

/* ------------------------------------------------------------- assembler */

void gps_asm_init(gps_assembler_t *a, uint8_t source_board, uint32_t window_ms)
{
    memset(a, 0, sizeof(*a));
    a->source_board = source_board;
    a->window_ms = window_ms;
}

static bool fresh(uint32_t now, uint32_t t, uint32_t window)
{
    return (uint32_t)(now - t) <= window;   /* wrap-safe */
}

bool gps_asm_feed(gps_assembler_t *a, const can_frame_t *f, uint32_t now_ms, GeoPos *fix)
{
    if (!a || !f) return false;
    if (a->source_board != 0 && rc_board_type(f->id) != a->source_board) return false;

    rc_status_t st;
    switch (rc_msg_type(f->id)) {
    case RC_MSG_GPS_LATITUDE:
        st = rc_decode_coord(f, RC_MSG_GPS_LATITUDE, &a->lat);
        if (st == RC_OK) { a->have_lat = true; a->lat_ms = now_ms; }
        break;
    case RC_MSG_GPS_LONGITUDE:
        st = rc_decode_coord(f, RC_MSG_GPS_LONGITUDE, &a->lon);
        if (st == RC_OK) { a->have_lon = true; a->lon_ms = now_ms; }
        break;
    case RC_MSG_GPS_ALTITUDE:
        st = rc_decode_alt(f, &a->alt);
        if (st == RC_OK) { a->have_alt = true; a->alt_ms = now_ms; }
        break;
    case RC_MSG_GPS_INFO:
        st = rc_decode_info(f, &a->num_sat, &a->quality);
        if (st == RC_OK) a->have_info = true;
        if (st == RC_OK) a->frames_ok++; else a->frames_rejected++;
        return false;           /* info never completes a fix on its own */
    default:
        return false;           /* not a GPS position message */
    }

    if (st != RC_OK) { a->frames_rejected++; return false; }
    a->frames_ok++;

    if (!(a->have_lat && a->have_lon && a->have_alt)) return false;
    if (!fresh(now_ms, a->lat_ms, a->window_ms) ||
        !fresh(now_ms, a->lon_ms, a->window_ms) ||
        !fresh(now_ms, a->alt_ms, a->window_ms)) {
        return false;           /* one part is left over from an older cycle */
    }

    /* Consume this set so each GPS cycle produces exactly one fix. */
    a->have_lat = a->have_lon = a->have_alt = false;

    /* Quality 0 = "no fix" in NMEA GGA; the board still sends (zeroed) coords. */
    if (a->have_info && a->quality == 0) { a->fixes_rejected++; return false; }
    if (a->lat == 0.0 && a->lon == 0.0)  { a->fixes_rejected++; return false; }

    if (fix) {
        fix->lat_deg = a->lat;
        fix->lon_deg = a->lon;
        fix->h_m     = a->alt;
    }
    a->fixes++;
    return true;
}
