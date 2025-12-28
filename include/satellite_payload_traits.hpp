#ifndef __DSO_SATELLITE_PAYLOAD_GENERIC_TRAITS_HPP__
#define __DSO_SATELLITE_PAYLOAD_GENERIC_TRAITS_HPP__

#include "satellite_core.hpp"

namespace dso {
/** @brief Base class for SatellitePayloadTraits.
 *
 * We will specialize this class for each known satellite, so that we
 * have metadata available at compile-time.
 */
template <SATELLITE S>
struct SatellitePayloadTraits {}; /* struct SatellitePayloadTraits */
} /* namespace dso */

#endif
