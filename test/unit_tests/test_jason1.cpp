#include "satellites/jason1.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace dso;
constexpr const double TOLERANCE = 1e-12;

int main()
{

  static_assert(SatelliteMacromodel<SATELLITE::JASON1>::initial_mass() == 489.1);

  static_assert(SatelliteMacromodel<SATELLITE::JASON1>::num_plates() == 6);

  static_assert(SatelliteMacromodel<SATELLITE::JASON1>::num_solar_arrays() == 2);

  auto dr1 = SatelliteMacromodel<SATELLITE::JASON1>::doris_s1_pco();
  assert(std::abs(dr1(0) - 1.1710) < TOLERANCE);
  assert(std::abs(dr1(1) + 0.5980) < TOLERANCE);
  assert(std::abs(dr1(2) - 1.0270) < TOLERANCE);

  dr1 = SatelliteMacromodel<SATELLITE::JASON1>::doris_u2_pco();
  assert(std::abs(dr1(0) - 1.1710) < TOLERANCE);
  assert(std::abs(dr1(1) + 0.5980) < TOLERANCE);
  assert(std::abs(dr1(2) - 0.8590) < TOLERANCE);

  dr1 = SatelliteMacromodel<SATELLITE::JASON1>::initial_cog();
  assert(std::abs(dr1(0) - 0.9550) < TOLERANCE);
  assert(std::abs(dr1(1) - 0.0000) < TOLERANCE);
  assert(std::abs(dr1(2) - 0.0000) < TOLERANCE);

  return 0;
}
