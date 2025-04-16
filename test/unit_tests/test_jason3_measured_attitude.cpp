#include "attitude.hpp"
#include <cstdio>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <random>

using namespace dso;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <QUATERNION FILE>\n", argv[0]);
    return 1;
  }

  /* pointer to base class */
  SatelliteAttitude *att;

  /* keep measured attitude data here */
  attitude_details::MeasuredAttitudeData data(
      SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions,
      SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles);

  /* Test 1: Testing attitude stream with normal dates. No errors */
  {
    try {
      /* get the measured attitude stream for the satellite */
      att = new MeasuredAttitude(SATELLITE::JASON3, argv[1]);
      printf("New attitude at address %p (tc 1)\n", (void*)att);
    } catch (std::exception &) {
      fprintf(stderr, "ERROR failed to get measured attitude stream\n");
      return 2;
    }

    std::uniform_real_distribution<double> unif(-10e0, 20e0);
    std::default_random_engine re;

    auto t = MjdEpoch(60679, FractionalSeconds(13.123));
    for (int i = 0; i < 1000; i++) {
      FractionalSeconds fsec(unif(re));
      if (i < 10 && fsec < FractionalSeconds(0))
        fsec = FractionalSeconds(2e0);
      t.add_seconds(fsec);
      if (att->attitude_at(t, data)) {
        fprintf(stderr, "ERROR Failed getting attitude!\n");
        return 9;
      }
    }
    /* Warning! memory should be freed, but i will create a leak here cause
     * i want to see the pointers!
     */
    // delete att;
  } /* Test 1 */

  /* Test 2: Go (way) backwards in time; should fail */
  {
    try {
      /* get the measured attitude stream for the satellite */
      att = new MeasuredAttitude(SATELLITE::JASON3, argv[1],
                                 MjdEpoch(60679, FractionalSeconds(13.123)));
      printf("New attitude at address %p (tc 2)\n", (void*)att);
    } catch (std::exception &) {
      fprintf(stderr, "ERROR failed to get measured attitude stream (tc 21)\n");
      return 2;
    }

    /* search attitude for nine days beofore; should not be in stream! */
    auto t = MjdEpoch(60670, FractionalSeconds(13.123));
    if (att->attitude_at(t, data) != 30) {
      fprintf(stderr, "ERROR Expected fail did not occur (tc22)!\n");
    }
    /* Warning! memory should be freed, but i will create a leak here cause
     * i want to see the pointers!
     */
    // delete att;
  } /* Test 2 */

  /* Test 3: Go (way) backwards in time; should fail (same as above). */
  {
    try {
      /* get the measured attitude stream for the satellite */
      att = new MeasuredAttitude(SATELLITE::JASON3, argv[1]);
      printf("New attitude at address %p (tc 3)\n", (void*)att);
    } catch (std::exception &) {
      fprintf(stderr, "ERROR failed to get measured attitude stream (tc 31)\n");
      return 2;
    }

    /* search attitude for attitude; should be in stream! */
    auto t = MjdEpoch(60679, FractionalSeconds(43.123));
    if (att->attitude_at(t, data)) {
      fprintf(stderr, "ERROR Failed getting measured attitude! (tc 32)\n");
    }

    /* search attitude for nine days beofore; should not be in stream! */
    t = MjdEpoch(60670, FractionalSeconds(13.123));
    if (att->attitude_at(t, data) != 30) {
      fprintf(stderr, "ERROR Expected fail did not occur! (tc 33)\n");
    }
    /* Warning! memory should be freed, but i will create a leak here cause
     * i want to see the pointers!
     */
    // delete att;
  } /* Test 3 */

  /* Test 4: Request for attitude after records (not in stream); should fail */
  {
    /* the last record included in the file: 60681 7.204378986591e+03 */
    try {
      /* get the measured attitude stream for the satellite */
      att = new MeasuredAttitude(
          SATELLITE::JASON3, argv[1],
          MjdEpoch(60681, FractionalSeconds(7.204378986591e+03 - 200e0)));
      printf("New attitude at address %p (tc 4)\n", (void*)att);
    } catch (std::exception &) {
      fprintf(stderr, "ERROR failed to get measured attitude stream (tc 41)\n");
      return 2;
    }

    /* search attitude, should not be in stream! */
    auto t = MjdEpoch(60681, FractionalSeconds(7.204378986591e+03 + 2e0));
    if (att->attitude_at(t, data) != 2) {
      fprintf(stderr, "ERROR Expected fail did not occur! (tc 42)\n");
    }
    /* Warning! memory should be freed, but i will create a leak here cause
     * i want to see the pointers!
     */
    // delete att;
  } /* Test 4 */

  return 0;
}
