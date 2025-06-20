#ifndef __DSO_SATELLITE_ATTITUDE_GEN_HPP__
#define __DSO_SATELLITE_ATTITUDE_GEN_HPP__

/* @file
 *
 * This file is includes top-level definitions for handling satellite attitude.
 * We use inheritance/polymorphism here, to take into account different
 * possibilities:
 *
 * - MeasuredAttitude which is based on DsoAttitudeStream; this type of attitude
 * represents measured attitude (i.e. quaternions and/or rotation angles) that
 * are extracted from a respective DSO data file.
 * - PhaseLawAttitude which (in the future) will handle computing attitude based
 * on a phase law.
 * - NoAttitude when no attitude information is available.
 *
 * The base class (for all the above) is SatelliteAttitude, so that we can:
 * @code
 * SatelliteAttitude *att;
 * // qua_ja3.csv is a DSO attitude file
 * att = new MeasuredAttitude(SATELLITE::JASON3, "qua_ja3.csv");
 * // placeholder for attitude data, Jason-3
 * attitude_details::MeasuredAttitudeData data(
 *    SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions,
 *    SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles);
 * // request attitude data for some date
 * att->attitude_at(t_inTT, data);
 * @endcode
 */

#include "attitude_stream.hpp"
#include "satellite.hpp"

namespace dso {
namespace satellite_details {
/* Number of measured attitude records (whatever that may be, e.g. 1 quaternion
 * and 1 angle, or 1 quaternin and 2 angles, ...) that shall be buffered at any
 * given time. */
constexpr const int MeasuredAttitudeBufferSize = 15;
} /* namespace satellite_details */

attitude_details::MeasuredAttitudeData
measured_attitude_data_factory(SATELLITE sat);

/** @brief Base class, does nothing usefull! */
class SatelliteAttitude {
  /** the satellite */
  SATELLITE msat;

public:
  /** @brief Constructor */
  SatelliteAttitude(SATELLITE s) noexcept : msat(s) {};

  /** @brief Must define a virtual destructor. */
  virtual ~SatelliteAttitude() noexcept = default;

  SATELLITE satellite() const noexcept { return msat; }

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
  virtual int reload() = 0;
}; /* SatelliteAttitude */

/** @brief Measured attitude, i.e. a number of quaternions and/or angles. */
class MeasuredAttitude final : public SatelliteAttitude {
  using BType =
      DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>;
  /** Measured attitude stream */
  BType matt;

  BType static factory(SATELLITE sat, const char *fn,
                       const MjdEpoch &t = MjdEpoch::min()) {
    switch (sat) {
    case (SATELLITE::JASON1):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON1>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON1>::NumAngles, t);

    case (SATELLITE::JASON2):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON2>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON2>::NumAngles, t);

    case (SATELLITE::JASON3):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles, t);

    case (SATELLITE::SENTINEL3A):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumAngles, t);

    case (SATELLITE::SENTINEL3B):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumAngles, t);

    case (SATELLITE::SENTINEL6A):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumAngles, t);

    case (SATELLITE::SWOT):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SWOT>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SWOT>::NumAngles, t);

    case (SATELLITE::CRYOSAT2):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::CRYOSAT2>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::CRYOSAT2>::NumAngles, t);
    default:
      throw std::runtime_error(
          "[ERROR] Failed to construct MeasuredAttitude\n");
    }
  }

public:
  /** @brief Constructor.
   * @param[in] sat The satellite.
   * @param[in] fn  The name of the input file to stream measure attitude
   * information from. These files are preprocessed by DSO and are expected to
   * have a certain format.
   */
  MeasuredAttitude(SATELLITE sat, const char *fn,
                   const MjdEpoch &t = MjdEpoch::min())
      : SatelliteAttitude(sat), matt(MeasuredAttitude::factory(sat, fn, t)) {}

  int attitude_at(
      const MjdEpoch &t,
      attitude_details::MeasuredAttitudeData &att) noexcept override {
    return matt.attitude_at(t, att);
  }

  int reload() { return matt.reload(); }
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
