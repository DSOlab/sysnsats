#include "satellites/jason2.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace dso;
constexpr const double TOLERANCE = 1e-12;

int main()
{

  static_assert(SatelliteMacromodelTraits<SATELLITE::JASON2>::initial_mass() == 505.9);

  static_assert(SatelliteMacromodelTraits<SATELLITE::JASON2>::num_plates() == 6);

  static_assert(SatelliteMacromodelTraits<SATELLITE::JASON2>::num_solar_arrays() == 2);

  auto dr1 = SatelliteMacromodelTraits<SATELLITE::JASON2>::doris_s1_pco();
  assert(std::abs(dr1(0) - 1.1940) < TOLERANCE);
  assert(std::abs(dr1(1) + 0.5980) < TOLERANCE);
  assert(std::abs(dr1(2) - 1.0220) < TOLERANCE);

  dr1 = SatelliteMacromodelTraits<SATELLITE::JASON2>::doris_u2_pco();
  assert(std::abs(dr1(0) - 1.1940) < TOLERANCE);
  assert(std::abs(dr1(1) + 0.5980) < TOLERANCE);
  assert(std::abs(dr1(2) - 0.8580) < TOLERANCE);

  dr1 = SatelliteMacromodelTraits<SATELLITE::JASON2>::initial_cog();
  assert(std::abs(dr1(0) - 0.9768) < TOLERANCE);
  assert(std::abs(dr1(1) - 0.0001) < TOLERANCE);
  assert(std::abs(dr1(2) - 0.0011) < TOLERANCE);

  return 0;
}
