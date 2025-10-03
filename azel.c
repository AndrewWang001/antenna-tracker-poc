#include "azel.h"
#include <math.h>


// --- WGS-84 param ------------------------------
typedef struct {
    double a, f, e2;
} Ellipsoid;

static const Ellipsoid WGS84 = {
    .a  = 6378137.0,
    .f  = 1.0 / 298.257223563,
    .e2 = (1.0 / 298.257223563) * (2.0 - (1.0 / 298.257223563))
};

// --- helpers -----------------------------------------------------------
static double deg2rad(double d) {
    return d * (M_PI / 180.0);
}
static double rad2deg(double r) {
    return r * (180.0 / M_PI);
}
static double wrap_360(double d) {
    double r = fmod(d, 360.0);
    return (r < 0.0) ? (r + 360.0) : r;
}

typedef struct {
    double x, y, z;
} Vec3;

static Vec3 geodetic_to_ecef(const GeoPos *g, const Ellipsoid *E) {

    const double lat = deg2rad(g->lat_deg);
    const double lon = deg2rad(g->lon_deg);
    const double  h  = g->h_m;

    const double sphi = sin(lat), cphi = cos(lat);
    const double slam = sin(lon), clam = cos(lon);

    const double N = E->a / sqrt(1.0 - E->e2 * sphi * sphi);

    Vec3 p;
    p.x = (N + h) * cphi * clam;
    p.y = (N + h) * cphi * slam;
    p.z = (N * (1.0 - E->e2) + h) * sphi;
    return p;
}

static Vec3 ecef_to_enu(const Vec3 v, const GeoPos *origin) {

    const double lat = deg2rad(origin->lat_deg);
    const double lon = deg2rad(origin->lon_deg);

    const double sphi = sin(lat), cphi = cos(lat);
    const double slam = sin(lon), clam = cos(lon);

    const double e = -slam * v.x +  clam * v.y;
    const double n = -sphi*clam * v.x - sphi*slam * v.y + cphi * v.z;
    const double u =  cphi*clam * v.x +  cphi*slam * v.y + sphi * v.z;

    Vec3 enu = { e, n, u };
    return enu;
}

bool azel_compute(const GeoPos *observer, const GeoPos *target, AzEl *out) {

    if (!observer || !target || !out) return false;

    const Vec3 p0 = geodetic_to_ecef(observer, &WGS84);
    const Vec3 p1 = geodetic_to_ecef(target,   &WGS84);

    const Vec3 v  = { p1.x - p0.x, p1.y - p0.y, p1.z - p0.z };
    const Vec3 enu = ecef_to_enu(v, observer);

    const double r_h   = hypot(enu.x, enu.y);
    const double range = hypot(r_h, enu.z);
    const double el    = atan2(enu.z, r_h);
    const double az    = atan2(enu.x, enu.y); // atan2(E, N)

    out->az_deg   = wrap_360(rad2deg(az));
    out->el_deg   = rad2deg(el);
    out->range_m  = range;
    return true;
}
