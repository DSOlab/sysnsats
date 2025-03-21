#include "sentinel3a.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace dso;
constexpr const double TOLERANCE = 1e-12;

int main() {
  
  static_assert(SatelliteMacromodel<SATELLITE::SENTINEL3A>::initial_mass() == );
  
  static_assert(SatelliteMacromodel<SATELLITE::SENTINEL3A>::num_plates() == 6);
  
  static_assert(SatelliteMacromodel<SATELLITE::SENTINEL3A>::num_solar_arrays() == 2);
  
  auto dr1 = SatelliteMacromodel<SATELLITE::SENTINEL3A>::doris_s1_pco();
  assert(std::abs(dr1(0)-2.4128)<TOLERANCE);
  assert(std::abs(dr1(1)+0.1325)<TOLERANCE);
  assert(std::abs(dr1(2)-0.9235)<TOLERANCE);
  
  dr1 = SatelliteMacromodel<SATELLITE::SENTINEL3A>::doris_u2_pco();
  assert(std::abs(dr1(0)-2.4128)<TOLERANCE);
  assert(std::abs(dr1(1)+0.1325)<TOLERANCE);
  assert(std::abs(dr1(2)-0.7555)<TOLERANCE);
  
  dr1 = SatelliteMacromodel<SATELLITE::SENTINEL3A>::initial_cog();
  assert(std::abs(dr1(0)-1.0023)<TOLERANCE);
  assert(std::abs(dr1(1)-0e0)<TOLERANCE);
  assert(std::abs(dr1(2)+0.0021)<TOLERANCE);

  return 0;
}
