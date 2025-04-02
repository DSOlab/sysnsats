#ifndef __DSO_SATELLITE_ATTITUDE_GEN_HPP__
#define __DSO_SATELLITE_ATTITUDE_GEN_HPP__

namespace dso {

/** @brief Base class, does nothing usefull! */
class SatelliteAttitude {}; /* SatelliteAttitude */

/** @brief Measured attitude, i.e. a number of quaternions and/or angles. */
class MeasuredAttitude : public SatelliteAttitude {}; /* MeasuredAttitude */

/** @brief Phase-law attitude (not measured). */
class PhaseLawAttitude : public SatelliteAttitude {}; /* PhaseLawAttitude */

/** @brief No attitude at all. */
class NoAttitude : public SatelliteAttitude {}; /* NoAttitude */

} /* namespace dso */

#endif