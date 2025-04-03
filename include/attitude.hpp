#ifndef __DSO_SATELLITE_ATTITUDE_GEN_HPP__
#define __DSO_SATELLITE_ATTITUDE_GEN_HPP__

#include "quaternion_stream.hpp"

namespace dso {
namespace satellite_details {
constexpr const int QuaternionStreamBufferSize = 15;
} /* namespace satellite_details */

/** @brief Base class, does nothing usefull! */
class SatelliteAttitude {
public:
  virtual int
  attitude_at(const MjdEpoch &t,
              attitude_details::MeasuredAttitudeData &att) noexcept = 0;
}; /* SatelliteAttitude */

/** @brief Measured attitude, i.e. a number of quaternions and/or angles. */
class MeasuredAttitude final : SatelliteAttitude {
  /** Measured attitude stream */
  DsoAttitudeStream<satellite_details::QuaternionStreamBufferSize> matt;

public:
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