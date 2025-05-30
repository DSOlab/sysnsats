#include "satellite.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace dso;
constexpr const double TOLERANCE = 1e-12;

int main() {

  static_assert(SatelliteMacromodelTraits<SATELLITE::JASON3>::initial_mass() ==
                509.6e0);

  static_assert(
      SatelliteMacromodelTraits<SATELLITE::JASON3>::num_solar_arrays() == 2);

  auto dr1 = SatelliteMacromodelTraits<SATELLITE::JASON3>::doris_s1_pco();
  assert(std::abs(dr1(0) - 2.4128) < TOLERANCE);
  assert(std::abs(dr1(1) + 0.1325) < TOLERANCE);
  assert(std::abs(dr1(2) - 0.9235) < TOLERANCE);

  dr1 = SatelliteMacromodelTraits<SATELLITE::JASON3>::doris_u2_pco();
  assert(std::abs(dr1(0) - 2.4128) < TOLERANCE);
  assert(std::abs(dr1(1) + 0.1325) < TOLERANCE);
  assert(std::abs(dr1(2) - 0.7555) < TOLERANCE);

  dr1 = SatelliteMacromodelTraits<SATELLITE::JASON3>::initial_cog();
  assert(std::abs(dr1(0) - 1.0023) < TOLERANCE);
  assert(std::abs(dr1(1) - 0e0) < TOLERANCE);
  assert(std::abs(dr1(2) + 0.0021) < TOLERANCE);

  for (const auto &surface :
       SatelliteMacromodelTraits<SATELLITE::JASON3>::model) {
    assert(std::abs(surface.abs_optical() + surface.diff_optical() +
                    surface.spec_optical() - 1e0) < TOLERANCE);
  }
  return 0;
}
