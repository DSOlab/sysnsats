#ifndef __DSO_SATELLITE_PAYLOAD_WRAPPERS_HPP__
#define __DSO_SATELLITE_PAYLOAD_WRAPPERS_HPP__

#include "satellite_payload_traits.hpp"
#include "satellite.hpp"

namespace dso {

template<SATELLITE_SYSTEM Sys>
Eigen::Vector3d payload_eccentricity_bf(SATELLITE sat, const char *freq) {
  switch (sat) {
    case SATELLITE::JASON1:
      return SatellitePayloadTraits<SATELLITE::JASON1>::template
        payload_eccentricity_bf<Sys>(freq);
    case SATELLITE::JASON2:
      return SatellitePayloadTraits<SATELLITE::JASON2>::template
        payload_eccentricity_bf<Sys>(freq);
    case SATELLITE::JASON3:
      return SatellitePayloadTraits<SATELLITE::JASON3>::template
        payload_eccentricity_bf<Sys>(freq);
  }
  throw std::runtime_error("Unhandled Satellite enum value");
}

} /* namespace dso */

#endif
