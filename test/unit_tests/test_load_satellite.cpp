#include "attitude.hpp"
#include <cstdio>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <random>

int main(int argc, char *argv[]) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s <SATID> <QUATERNION FILE>\n", argv[0]);
    return 1;
  }

  /* pointer to base class */
  dso::SatelliteAttitude *att;

  /* resolve satellite id */
  dso::SATELLITE sat = dso::translate_satid(argv[1]);

  try {
    /* get the measured attitude stream for the satellite */
    att = new dso::MeasuredAttitude(sat, argv[2]);
  } catch (std::exception &) {
    fprintf(stderr, "ERROR failed to get measured attitude stream\n");
    return 2;
  }

  std::uniform_real_distribution<double> unif(-10e0, 20e0);
  std::default_random_engine re;


  if (sat == dso::SATELLITE::JASON3) {
  /* keep measured attitude data here */
  dso::attitude_details::MeasuredAttitudeData data(
      dso::SatelliteAttitudeTraits<dso::SATELLITE::JASON3>::NumQuaternions,
      dso::SatelliteAttitudeTraits<dso::SATELLITE::JASON3>::NumAngles);

  auto t = dso::MjdEpoch(60679, dso::FractionalSeconds(13.123));
  for (int i = 0; i < 1000; i++) {
    dso::FractionalSeconds fsec(unif(re));
    if (i < 10 && fsec < dso::FractionalSeconds(0))
      fsec = dso::FractionalSeconds(2e0);
    t.add_seconds_inplace(fsec);
    if (att->attitude_at(t, data)) {
      fprintf(stderr, "ERROR Failed getting attitude!\n");
      return 9;
    }
    }
  }

  return 0;
}
