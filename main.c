#include <stdio.h>
#include "azel.h"

int main(void) {
    GeoPos obs = {43.4723, -80.5449, 300.0};    // Waterloo
    GeoPos tgt = {43.6426, -79.3871, 553.3};    // CN Tower top

    AzEl r;
    if (!azel_compute(&obs, &tgt, &r)) {
        fprintf(stderr, "azel_compute failed\n");
        return 1;
    }

    printf("Az: %.3f deg  El: %.3f deg  Range: %.1f m\n", r.az_deg, r.el_deg, r.range_m);
    return 0;
}
