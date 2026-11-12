/**
 * rocket_can.h - decode Waterloo Rocketry GPS telemetry from CAN frames.
 *
 * Wire format follows waterloo-rocketry/rocketcan + rocketlib
 * (can_message/msg_common.h, msg_gps.c), reimplemented here so this project
 * has no dependency on rocketlib:
 *
 *   29-bit ID:  [28:27] priority  [26:20] message type
 *               [19:14] board type [13:8] board instance  [7:0] metadata
 *   data[0..1]: 16-bit timestamp (ms, big endian)
 *
 *   GPS_LATITUDE  (0x0F) / GPS_LONGITUDE (0x10), len 7:
 *       data[2] degrees, data[3] minutes, data[4..5] minutes*10000 fraction,
 *       data[6] 'N'/'S' or 'E'/'W'
 *   GPS_ALTITUDE  (0x11), len 7:
 *       data[2..5] whole metres (u32 BE), data[6] hundredths of a metre
 *       (the GPS board sends the NMEA GGA altitude, which is metres MSL)
 *   GPS_INFO      (0x12), len 4: data[2] num sats, data[3] fix quality
 *
 * Pure C, no HAL: builds on the target and in the host unit tests.
 */
#ifndef ROCKET_CAN_H
#define ROCKET_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include "azel.h"

#define RC_MSG_GPS_TIMESTAMP  0x0Eu
#define RC_MSG_GPS_LATITUDE   0x0Fu
#define RC_MSG_GPS_LONGITUDE  0x10u
#define RC_MSG_GPS_ALTITUDE   0x11u
#define RC_MSG_GPS_INFO       0x12u

#define RC_BOARD_TYPE_GPS     0x07u
#define RC_PRIO_HIGH          0x1u

typedef struct {
    uint32_t id;        /* 29-bit extended identifier */
    uint8_t  len;
    uint8_t  data[8];
    uint32_t rx_cycles; /* DWT cycle count stamped in the RX ISR (0 if unknown) */
} can_frame_t;

typedef enum {
    RC_OK = 0,
    RC_WRONG_TYPE,
    RC_BAD_LEN,
    RC_BAD_FIELD,
} rc_status_t;

static inline uint8_t rc_msg_type(uint32_t id)    { return (uint8_t)((id >> 20) & 0x7Fu); }
static inline uint8_t rc_board_type(uint32_t id)  { return (uint8_t)((id >> 14) & 0x3Fu); }
static inline uint8_t rc_board_inst(uint32_t id)  { return (uint8_t)((id >> 8)  & 0x3Fu); }

uint32_t rc_build_id(uint8_t prio, uint8_t msg_type, uint8_t board_type,
                     uint8_t board_inst, uint8_t metadata);

/* Decoders: on anything other than RC_OK the output is left untouched. */
rc_status_t rc_decode_coord(const can_frame_t *f, uint8_t expected_type, double *deg);
rc_status_t rc_decode_alt(const can_frame_t *f, double *alt_m);
rc_status_t rc_decode_info(const can_frame_t *f, uint8_t *num_sat, uint8_t *quality);

/* Encoders: produce frames byte-identical to rocketlib's build_gps_*_msg.
 * Used by the `sim` console command and the unit tests. */
void rc_encode_coord(uint8_t msg_type, double deg, uint16_t ts, uint8_t board_type, can_frame_t *f);
void rc_encode_alt(double alt_m, uint16_t ts, uint8_t board_type, can_frame_t *f);
void rc_encode_info(uint8_t num_sat, uint8_t quality, uint16_t ts, uint8_t board_type, can_frame_t *f);

/* ---------------------------------------------------------------------------
 * Fix assembler: lat, lon and alt come in as separate frames. Emits one
 * complete GeoPos once all three have arrived from the same board within
 * window_ms of each other.
 */
typedef struct {
    uint8_t  source_board;   /* 0 = accept any board type */
    uint32_t window_ms;

    double   lat, lon, alt;
    uint32_t lat_ms, lon_ms, alt_ms;
    bool     have_lat, have_lon, have_alt;

    uint8_t  num_sat, quality;
    bool     have_info;

    uint32_t frames_ok, frames_rejected, fixes, fixes_rejected;
} gps_assembler_t;

void gps_asm_init(gps_assembler_t *a, uint8_t source_board, uint32_t window_ms);

/* Returns true and fills *fix when this frame completes a fix. */
bool gps_asm_feed(gps_assembler_t *a, const can_frame_t *f, uint32_t now_ms, GeoPos *fix);

#endif /* ROCKET_CAN_H */
