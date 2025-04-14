#ifndef VARIABLE_SIZE_ATTITUDE_DATA

#ifndef __DSO_MEASURED_STARCAMERA_SATELLITE_ATTITUDE_HPP__
#define __DSO_MEASURED_STARCAMERA_SATELLITE_ATTITUDE_HPP__

#include "datetime/calendar.hpp"
#include "eigen3/Eigen/Geometry"
#include <cstdint>

namespace dso {

namespace attitude_details {

/** @brief Maximum characters in any dso-quaternion file line. */
constexpr const int MAX_ATTITUDE_LINE_CHARS = 248;

struct MeasuredAttitudeData {
  /* number of quaternions stored */
  static constexpr const int quat_capacity = 1;
  /* number of angles stored */
  static constexpr const int angles_capacity = 2;

  /* number of quaternions (actual, not capacity) */
  int8_t num_quat_;
  /* number of angles (actual, not capacity) */
  int8_t num_angles_;
  /* epoch of measured attitude in TT */
  MjdEpoch mtt_;
  /* array of quaternions */
  alignas(16) Eigen::Quaterniond mq_;
  /* array of angles */
  double ma_[angles_capacity] = {0e0, 0e0};

  constexpr int num_quaternions() const noexcept { return num_quat_; }

  constexpr int num_angles() const noexcept { return num_angles_; }

  /** @brief Constructor.
   *
   * @param[in] nq Number of quaternions.
   * @param[in] na Number of angles.
   * @param[in] t  (Optional) epoch of measured attitude, else min possible
   * date.
   */
  MeasuredAttitudeData(int nquaternions, int nangles,
                       const MjdEpoch &t = MjdEpoch::min()) noexcept
      : num_quat_(nquaternions), num_angles_(nangles), mtt_(t) {}

  const Eigen::Quaterniond &quaternion() const noexcept { return mq_; }
  Eigen::Quaterniond &quaternion() noexcept { return mq_; }
  Eigen::Quaterniond *quaternions() noexcept { return &mq_; }
  const Eigen::Quaterniond *quaternions() const noexcept { return &mq_; }

  const MjdEpoch &t() const noexcept { return mtt_; }
  MjdEpoch &t() noexcept { return mtt_; }

  const double *angles() const noexcept { return ma_; }
  double *angles() noexcept { return ma_; }

  double angle(int idx) const noexcept {
#ifdef DEBUG
    assert(idx < angles_capacity);
#endif
    return ma_[idx];
  }
  double &angle(int idx) noexcept {
#ifdef DEBUG
    assert(idx < angles_capacity);
#endif
    return ma_[idx];
  }

}; /* struct MeasuredAttitudeData */

} /* namespace attitude_details */

} /* namespace dso */

#endif

#endif