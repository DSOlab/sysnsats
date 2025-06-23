#include "attitude.hpp"
#include <cstdio>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

using namespace dso;

struct {
  MjdEpoch t_;
  double x, y, z;
} Data[] = {}; /* Data array */

int main(int argc, char *argv[]) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <QUATERNION FILE>\n", argv[0]);
    return 1;
  }

  /* pointer to base class; get the measured attitude stream for the satellite
   */
  Attitude *att = new MeasuredAttitude(SATELLITE::JASON1, argv[1]);

  /* create a Macromodel instance for the satellite */
  const auto macrom =
      SatelliteMacromodel::createSatelliteMacromodel(SATELLITE::JASON1);

  /* keep measured attitude data here */
  attitude_details::MeasuredAttitudeData data(
      SatelliteAttitudeTraits<SATELLITE::JASON1>::NumQuaternions,
      SatelliteAttitudeTraits<SATELLITE::JASON1>::NumAngles);

  auto t = MjdEpoch(52696, FractionalSeconds(86393.123));
  for (int i = 0; i < 1000; i++) {
    /* get attitude (into data) */
    if (att->attitude_at(t, data)) {
      fprintf(stderr, "ERROR Failed getting attitude!\n");
      return 9;
    }
    /* rotate macromodel to ECI frame */
    const auto rm = macrom.rotate_macromodel(data.quaternions(), data.angles());

    t.add_seconds(FractionalSeconds(30));
  }

  /* free memory */
  delete att;

  return 0;
}
