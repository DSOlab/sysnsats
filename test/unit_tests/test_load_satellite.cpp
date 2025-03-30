#include "attitude_stream.hpp"
#include <cstdio>

int main(int argc, char *argv[]) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s <SATID> <QUATERNION FILE>\n", argv[0]);
    return 1;
  }

  dso::Satellite sat(argv[1]);

  if (sat.load_geometry()) {
    fprintf(stderr, "ERROR. Failed loading geometry for sat %s\n", argv[1]);
    return 10;
  }

  if (sat.load_attitude(argv[2])) {
    fprintf(stderr, "ERROR Failed loading attitude for satellite %s\n",
            argv[1]);
    return 20;
  }

  return 0;
}