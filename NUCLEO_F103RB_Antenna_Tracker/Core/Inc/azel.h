#pragma once
#ifndef AZEL_H
#define AZEL_H

#include <stdbool.h>

    /** geodetic position */
    typedef struct {
        double lat_deg;   /* latitude (degrees) */
        double lon_deg;   /* longitude, east positive (degrees) */
        double h_m;       /* height (meters) */
    } GeoPos;

    /** Line-of-sight solution in the local ENU frame at the observer. */
    typedef struct {
        double az_deg;    /* azimuth [0,360) deg, clockwise from true north */
        double el_deg;    /* elevation [-90,+90] deg, up from local horizon */
        double range_m;   /* slant range (meters) */
    } AzEl;

    /**
     * Compute azimuth/elevation/range from observer to target (WGS-84).
     *
     * Returns true on success; false if any output pointer is NULL.
     */
    bool azel_compute(const GeoPos *observer, const GeoPos *target, AzEl *result);

#endif /* AZEL_H */
