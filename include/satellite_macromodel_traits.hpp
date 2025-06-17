#ifndef __DSO_SATELLITE_MACROMODEL_GENERIC_TRAITS_HPP__
#define __DSO_SATELLITE_MACROMODEL_GENERIC_TRAITS_HPP__

#include "satellite_core.hpp"

namespace dso {
/** @brief Base class for SatelliteMacromodelsTraits.
 *
 * We will specialize this class for each known satellite macromodel, so that we
 * have metadata available at compile-time.
 */
template <SATELLITE S>
struct SatelliteMacromodelTraits {}; /* struct SatelliteMacromodelTraits */
} /* namespace dso */

#endif
