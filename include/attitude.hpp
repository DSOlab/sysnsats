#ifndef __DSO_SATELLITE_ATTITUDE_GEN_HPP__
#define __DSO_SATELLITE_ATTITUDE_GEN_HPP__

#include "quaternion_stream.hpp"
#include "satellite.hpp"

namespace dso {
namespace satellite_details {
/* Number of measured attitude records (whatever that may be, e.g. 1 quaternion
 * and 1 angle, or 1 quaternin and 2 angles, ...) that shall be buffered at any
 * given time. */
constexpr const int MeasuredAttitudeBufferSize = 15;
} /* namespace satellite_details */

/** @brief Base class, does nothing usefull! */
class SatelliteAttitude {
  /** the satellite */
  SATELLITE msat;

public:
  /** @brief Constructor */
  SatelliteAttitude(SATELLITE s) noexcept : msat(s) {};
  /** @brief Must define a virtual destructor. */
  virtual ~SatelliteAttitude() noexcept {};
  /** @brief Get the attitude at any given epoch.
   *
   * The attitude ie returned via the attitude_details::MeasuredAttitudeData and
   * at this point the function is agnostic as to what is actually returned
   * (i.e. number of quaternions if any, number of angles if any, ...).
   *
   * This function should be overidden by any inherited class.
   */
  virtual int
  attitude_at(const MjdEpoch &t,
              attitude_details::MeasuredAttitudeData &att) noexcept = 0;
}; /* SatelliteAttitude */

/** @brief Measured attitude, i.e. a number of quaternions and/or angles. */
class MeasuredAttitude final : SatelliteAttitude {
  /** Measured attitude stream */
  DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize> matt;

public:
  /** @brief Constructor.
   * @param[in] sat The satellite.
   * @param[in] fn  The name of the input file to stream measure attitude
   * information from. These files are preprocessed by DSO and are expected to
   * have a certain format.
   */
  MeasuredAttitude(SATELLITE sat, const char *fn) : SatelliteAttitude(sat) {
    switch (sat) {
    case (SATELLITE::JASON1):
      matt = DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON1>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON1>::NumAngles);

    case (SATELLITE::JASON2):
      matt = DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON2>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON2>::NumAngles);

    case (SATELLITE::JASON3):
      matt = DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles);

    case (SATELLITE::SENTINEL3A):
      matt = DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumAngles);

    case (SATELLITE::SENTINEL3B):
      matt = DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumAngles);

    case (SATELLITE::SENTINEL6A):
      matt = DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumAngles);
    default:
      throw std::runtime_error(
          "[ERROR] Failed to construct MeasuredAttitude\n");
    }
  }

  int attitude_at(
      const MjdEpoch &t,
      attitude_details::MeasuredAttitudeData &att) noexcept override {
    return matt.attitude_at(t, att);
  }
}; /* MeasuredAttitude */

/** @brief Phase-law attitude (not measured). */
class PhaseLawAttitude final : public SatelliteAttitude {
public:
  int attitude_at(
      const MjdEpoch &t,
      attitude_details::MeasuredAttitudeData &att) noexcept override;
}; /* PhaseLawAttitude */

/** @brief No attitude at all. */
class NoAttitude final : public SatelliteAttitude {
public:
  int attitude_at([[maybe_unused]] const MjdEpoch &,
                  [[maybe_unused]] attitude_details::MeasuredAttitudeData
                      &) noexcept override {
    return 0;
  };
}; /* NoAttitude */

} /* namespace dso */

#endif