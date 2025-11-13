#ifndef SRC_AZEL_H_
#define SRC_AZEL_H_

#include <stdbool.h>

    // geodetic position
    typedef struct {
        float lat_deg;   // latitude (degrees)
        float lon_deg;   // longitude, east positive (degrees)
        float h_m;       // height (meters)
    } GeoPos;

    /** Line-of-sight solution in the local ENU frame at the observer. */
    typedef struct {
    	float az_deg;    // azimuth [0,360) deg, clockwise from true north
    	float el_deg;    // elevation [-90,+90] deg, up from local horizon
    	float range_m;   // slant range (meters)
    } AzEl;

    /**
     * compute azimuth/elevation/range from observer to target (WGS-84).
     *
     * returns true on success; false if any output pointer is NULL.
     */
    bool azel_compute(const GeoPos *observer, const GeoPos *target, AzEl *result);

#endif /* SRC_AZEL_H_ */
