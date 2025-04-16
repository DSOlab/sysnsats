#ifndef __DSO_SATELLITE_GENERIC_MACROMODEL_HPP__
#define __DSO_SATELLITE_GENERIC_MACROMODEL_HPP__

#include "core/macromodel_surface_element.hpp"
#include "satellites_core.hpp"
#include <array>

namespace dso {

/** @brief Base class for SatelliteMacromodelsTraits.
 *
 * We will specialize this class for each known satellite macromodel, so that we
 * have metadata available at compile-time.
 */
template <SATELLITE S>
struct SatelliteMacromodelTraits {}; /* struct SatelliteMacromodelTraits */

/** @brief SatelliteMacromodelImpl utilities API. */
template <SATELLITE S> struct SatelliteMacromodelImpl {
  using Traits = SatelliteMacromodelTraits<S>;
};

} /* namespace dso */

#endif
