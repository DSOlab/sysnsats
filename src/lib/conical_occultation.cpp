#include "occultation.hpp"
#include <cmath>

double dso::conical_occultation(const Eigen::Vector3d &rsat,
                                const Eigen::Vector3d &rsun, double Re,
                                double Rs) noexcept
{
    /* distance (geocentric) of satellite */
    const double r = rsat.norm();

    /* apparent radius of the Earth */
    const double te = std::asin(Re / r);

    /* apparent radius of the Sun */
    const double ts = std::asin(Rs / (rsun - rsat).norm());

    /* apparent seperation */
    const double tes =
        std::acos(-rsat.dot(rsun - rsat) / (r * (rsun - rsat).norm()));

    /* determine f = S/(π*θ_s^2) based on Fig. 5, see reference */
    double f = 0;
    if (tes >= te + ts)
    {
        /* no occultation */
        f = 0;
    }
    else if (tes <= te - ts)
    {
        /* total occultation */
        // const double S = M_PI * ts * ts;
        f = 1e0;
    }
    else if (tes <= ts - te)
    {
        /* partial occultation */
        f = std::pow(te / ts, 2.);
    }
    else if ((tes > std::abs(te - ts)) && (tes < te + ts))
    {
        /* partial occultation Eq. (7) */
        const double caf =
            std::acos((ts * ts + tes * tes - te * te) / (2e0 * ts * tes));
        const double cbd =
            std::acos((te * te + tes * tes - ts * ts) / (2e0 * te * tes));
        const double Safc = .5e0 * caf * ts * ts;
        const double Saec = .5e0 * ts * std::sin(caf) * ts * std::cos(caf);
        const double Sbdc = .5e0 * cbd * te * te;
        const double Sbec = .5e0 * te * std::sin(cbd) * te * std::cos(cbd);
        const double S = 2e0 * (Safc - Saec) + 2e0 * (Sbdc - Sbec);
        f = S / (ts * M_PI * ts);
    }

    return 1e0 - f;
}