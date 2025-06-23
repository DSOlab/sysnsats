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

#include "macromodel_surface_element.hpp"
#include "satellite.hpp"

namespace dso {
/** @brief SatelliteMacromodelImpl utilities API. */
template <SATELLITE S> struct SatelliteMacromodelImpl {
  using Traits = SatelliteMacromodelTraits<S>;

  static std::vector<MacromodelSurfaceElement>
  rotate_macromodel(const Eigen::Quaterniond *qbody, const double *angles,
                    const Eigen::Vector3d *vecs) noexcept {
    return Traits::rotate_macromodel(qbody, angles, vecs);
  };

  static double satellite_mass() noexcept { return Traits::initial_mass(); }
  static double srp_cannonball_area() noexcept {
    return Traits::srp_cannonball_area();
  }
};

class SatelliteMacromodel {
  /** @brief Step 1:  Base "concept" interface (virtual, abstract). */
  struct Concept {
    virtual ~Concept() = default;
    virtual std::vector<MacromodelSurfaceElement> rotate_macromodel(
        [[maybe_unused]] const Eigen::Quaterniond *qbody,
        [[maybe_unused]] const double *thetas,
        [[maybe_unused]] const Eigen::Vector3d *vecs = nullptr) const noexcept {
      return std::vector<MacromodelSurfaceElement>{};
    }
    virtual double satellite_mass() const noexcept { return 0e0; }
    virtual double srp_cannonball_area() const noexcept { return 0e0; }
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
    double satellite_mass() const noexcept { return impl.satellite_mass(); }
    double srp_cannonball_area() const noexcept {
      return impl.srp_cannonball_area();
    }
  }; /* SatelliteMacromodel::Model<Impl> */

  /** @brief Step 3 : Raw pointer to concept (type - erased storage) */
  Concept *self;
  /** @brief Mass correction for a given epoch */
  double mdmass{0e0};

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
  }

  double satellite_mass() const noexcept {
    return self->satellite_mass() + mdmass;
  }

  double srp_cannoball_area() const noexcept {
    return self->srp_cannonball_area();
  }

  int load_satellite_mass_correction(const char *cnes_fn,
                                     const dso::MjdEpoch &t) {
    Eigen::Vector3d dummy;
    return dso::cnes_satellite_correction(cnes_fn, t, mdmass, dummy);
  }

  /** @brief Factory */
  static SatelliteMacromodel createSatelliteMacromodel(SATELLITE sat) {
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
    case SATELLITE::SENTINEL6A:
      return SatelliteMacromodel(
          SatelliteMacromodelImpl<SATELLITE::SENTINEL6A>{});
    case SATELLITE::SWOT:
      return SatelliteMacromodel(SatelliteMacromodelImpl<SATELLITE::SWOT>{});
    case SATELLITE::CRYOSAT2:
      return SatelliteMacromodel(
          SatelliteMacromodelImpl<SATELLITE::CRYOSAT2>{});
    case SATELLITE::SPOT4:
      return SatelliteMacromodel(SatelliteMacromodelImpl<SATELLITE::SPOT4>{});

    default:
      throw std::runtime_error(
          "[ERROR] Unknown satellite, failed to create Macromodel!\n");
    }
  }

}; /* class SatelliteMacromodel */

} /* namespace dso */

#endif
