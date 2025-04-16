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
    virtual std::vector<MacromodelSurfaceElement> rotate_macromodel(
        [[maybe_unused]] const Eigen::Quaterniond *qbody,
        [[maybe_unused]] const double *thetas,
        [[maybe_unused]] const Eigen::Vector3d *vecs = nullptr) const noexcept {
      return std::vector<MacromodelSurfaceElement>{};
    };
  }; /* SatelliteMacromodel::Concept */

  /** @brief  Step 2: Templated model that wraps any SatelliteMacromodel<S> */
  template <typename Impl> struct Model : public Concept {
    Impl impl;
    Model(Impl i) noexcept : impl(std::move(i)) {};
    std::vector<MacromodelSurfaceElement>
    rotate_macromodel(const Eigen::Quaterniond *qbody, const double *thetas,
                      const Eigen::Vector3d *vecs = nullptr) const noexcept {
      return impl.rotate_macromodel(qbody, thetas, vecs);
    }
  }; /* SatelliteMacromodel::Model<Impl> */

  /** @brief Step 3 : Raw pointer to concept (type - erased storage) */
  Concept *self;

public:
  /** @brief Constructor: creates a concrete model (Step 4). */
  template <typename Impl>
  SatelliteMacromodel(Impl impl) : self(new Model<Impl>(std::move(impl))){};

  ~SatelliteMacromodel() noexcept { delete self; }

  /** @brief No copy constructor. */
  SatelliteMacromodel(const SatelliteMacromodel &) = delete;

  /** @brief No assignment operator.  */
  SatelliteMacromodel &operator=(const SatelliteMacromodel &) = delete;

  /** Step 5: Forwarding interface. **/
  std::vector<MacromodelSurfaceElement>
  rotate_macromodel(const Eigen::Quaterniond *qbody, const double *thetas,
                    const Eigen::Vector3d *vecs = nullptr) const noexcept {
    return self->rotate_macromodel(qbody, thetas, vecs);
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
}; /*class SatelliteMacromodel */

} /* namespace dso */

#endif