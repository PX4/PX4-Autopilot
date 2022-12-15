#include "vn/conv.h"
#include <math.h>
#include "vn/const.h"

#define C_E2        (0.006694379990141)
#define C_EPSILON   (0.996647189335253)
#define C_ABAR      (42.69767270717997)
#define C_BBAR      (42.84131151331357)
#define C_A         (6378.137)

vec3d ecef_to_lla_v3d(vec3d ecef)
{
    double x, y, z, r, c_phi, c_phi0, s_phi,
           s_phi0, tau, lat, lon, /*alt,*/ eta, h;

    const double Rthresh = 0.001;   /* Limit on distance from pole in km to switch calculation. */

    x = ecef.c[0];
    y = ecef.c[1];
    z = ecef.c[2];

    r = sqrt(x*x + y*y);

    if (r < Rthresh)
    {
        c_phi0 = 0;
        s_phi0 = (z > 0) - (z < 0); /* computes sign */
    }
    else
    {
        double tau0 = z / (C_EPSILON * r);
        c_phi0 = 1 / sqrt(1 + tau0 * tau0);
        s_phi0 = tau0 * c_phi0;
    }

    tau = (z + C_BBAR * s_phi0 * s_phi0 * s_phi0) / (r - C_ABAR * c_phi0 * c_phi0 * c_phi0);
    lat = atan(tau);

    if (r < Rthresh)
    {
        c_phi = 0;
        s_phi = (z > 0) - (z < 0); /* computes sign */
    }
    else
    {
        c_phi = 1 / sqrt(1 + tau * tau);
        s_phi = tau * c_phi;
    }

    eta = sqrt(1 - C_E2 * s_phi * s_phi);
    h = r * c_phi + z * s_phi - C_A * eta;

    lon = atan2(y, x);

    return create_v3d(
            lat * 180 / VNC_PI_D,
            lon * 180 / VNC_PI_D,
            h * 1000
    );
}

vec3d lla_to_ecef_v3d(vec3d lla)
{
    double lat, lon, alt;
    double n, x, y, z;
    double t1;  /* TEMPS */

    lat = lla.c[0] * VNC_PI_D / 180;   /* Geodetic latitude in radians. */
    lon = lla.c[1] * VNC_PI_D / 180;   /* Longitude in radians. */
    alt = lla.c[2] / 1000;             /* Altitude above WGS84 in km. */

    t1 = sin(lat);
    n = C_A / sqrt(1 - C_E2 * t1 * t1);

    t1 = alt + n;
    x = t1 * cos(lat) * cos(lon);
    y = t1 * cos(lat) * sin(lon);
    z = (t1 - C_E2 * n) * sin(lat);

    return create_v3d(x, y, z);
}
