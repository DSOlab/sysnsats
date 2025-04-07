/** @file
 *
 * This header file defines the SatelliteMacromodel class, which in essence is a
 * type-erased wrapper. Here is a basic breakdown of how this class/hierarchy is
 * meant to work:
 * - Concept is an abstract interface → gives us dynamic dispatch at runtime.
 * - Model<Impl> wraps a specific satellite
 * (SatelliteMacromodelImpl<SATELLITE::JASON3> for example).
 * - This should give the user a stable API (SatelliteMacromodel), regardless
 * of underlying type.
 */

#ifndef __DSO_SATELLITE_MACROMODEL_FINAL_HPP__
#define __DSO_SATELLITE_MACROMODEL_FINAL_HPP__

#include "satellites/jason1.hpp"
#include "satellites/jason2.hpp"
#include "satellites/jason3.hpp"
#include "satellites/sentinel3a.hpp"
#include "satellites/sentinel3b.hpp"

namespace dso {

class SatelliteMacromodel {

  /** @brief Step 1:  Base "concept" interface (virtual, abstract). */
  struct Concept {
    virtual ~Concept() = default;
    virtual int num_plates() const noexcept = 0;
  }; /* SatelliteMacromodel::Concept */

  /** @brief  Step 2: Templated model that wraps any SatelliteMacromodel<S> */
  template <typename Impl> struct Model : Concept {
    Impl impl;
    Model(Impl i) noexcept : impl(std::move(i)) {};
    int num_plates() const noexcept { return impl.num_plates(); }
  }; /* SatelliteMacromodel::Model<Impl> */

  /** @brief Step 3 : Raw pointer to concept (type - erased storage) */
  Concept *self;

public:
  /** @brief Constructor: creates a concrete model (Step 4). */
  template <typename Impl>
  SatelliteMacromodel(Impl impl) : self(new Model<Impl>(std::move(impl)));

  ~SatelliteMacromodel() { delete self; }

  /** @brief No copy constructor. */
  SatelliteMacromodel(const SatelliteMacromodel &) = delete;

  /** @brief No assignment operator.  */
  SatelliteMacromodel &operator=(const SatelliteMacromodel &) = delete;

  /** Step 5: Forwarding interface. **/

  int num_plates() const noexcept { return self->num_plates(); }
};

/** @brief Factory */
SatelliteMacromodel createSatelliteMacromodel(SATELLITE sat) {
  switch (sat) {
  case SATELLITE::JASON1:
    return SatelliteMacromodel(SatelliteMacromodelImpl<SATELLITE::JASON1>{});
  case SATELLITE::JASON2:
    return SatelliteMacromodel(SatelliteMacromodelImpl<SATELLITE::JASON2>{});
  case SATELLITE::JASON3:
    return SatelliteMacromodel(SatelliteMacromodelImpl<SATELLITE::JASON3>{});
  case SATELLITE::SENTINEL3A:
    return SatelliteMacromodel(
        SatelliteMacromodelImpl<SATELLITE::SENTINEL3A>{});
  case SATELLITE::SENTINEL3B:
    return SatelliteMacromodel(
        SatelliteMacromodelImpl<SATELLITE::SENTINEL3B>{});
  default:
    throw std::runtime_error("Unknown animal");
  }
}

} /* namespace dso */

#endif