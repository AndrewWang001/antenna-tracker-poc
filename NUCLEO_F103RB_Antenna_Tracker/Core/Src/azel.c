#include "azel.h"
#include <math.h>

typedef struct {
    float a, f, e2;
} Ellipsoid;

static const Ellipsoid WGS84 = {
    .a  = 6378137.0f,
    .f  = 1.0f / 298.257223563f,
    .e2 = (1.0f / 298.257223563f) * (2.0f - (1.0f / 298.257223563f))
};

static inline float deg2rad(float d)
{
	return d * (M_PI / 180.0f);
}

static inline float rad2deg(float r)
{
	return r * (180.0f / M_PI);
}

static inline float wrap_360(float d)
{
    float r = fmodf(d, 360.0f);
    return (r < 0.0f) ? (r + 360.0f) : r;
}

typedef struct { float x, y, z; } Vec3;

static Vec3 geodetic_to_ecef(const GeoPos *g, const Ellipsoid *E)
{
    float lat = deg2rad(g->lat_deg);
    float lon = deg2rad(g->lon_deg);
    float h   = g->h_m;

    float sphi = sinf(lat), cphi = cosf(lat);
    float slam = sinf(lon), clam = cosf(lon);

    float N = E->a / sqrtf(1.0f - E->e2 * sphi * sphi);

    Vec3 p;
    p.x = (N + h) * cphi * clam;
    p.y = (N + h) * cphi * slam;
    p.z = (N * (1 - E->e2) + h) * sphi;
    return p;
}

static Vec3 ecef_to_enu(const Vec3 v, const GeoPos *origin)
{
    float lat = deg2rad(origin->lat_deg);
    float lon = deg2rad(origin->lon_deg);

    float sphi = sinf(lat), cphi = cosf(lat);
    float slam = sinf(lon), clam = cosf(lon);

    float e = -slam * v.x +  clam * v.y;
    float n = -sphi*clam * v.x - sphi*slam * v.y + cphi * v.z;
    float u =  cphi*clam * v.x +  cphi*slam * v.y + sphi * v.z;

    Vec3 enu = { e, n, u };
    return enu;
}

bool azel_compute(const GeoPos *observer, const GeoPos *target, AzEl *out)
{
    if (!observer || !target || !out) return false;

    Vec3 p0  = geodetic_to_ecef(observer, &WGS84);
    Vec3 p1  = geodetic_to_ecef(target,   &WGS84);

    Vec3 v   = { p1.x - p0.x, p1.y - p0.y, p1.z - p0.z };
    Vec3 enu = ecef_to_enu(v, observer);

    float r_h   = sqrtf(enu.x*enu.x + enu.y*enu.y);
    float range = sqrtf(r_h*r_h + enu.z*enu.z);
    float el    = atan2f(enu.z, r_h);
    float az    = atan2f(enu.x, enu.y);

    out->az_deg  = wrap_360(rad2deg(az));
    out->el_deg  = rad2deg(el);
    out->range_m = range;

    return true;
}
