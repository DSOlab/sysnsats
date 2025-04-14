#include "attitude.hpp"
#include <cstdio>

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

  return 0;
}
