#ifdef VARIABLE_SIZE_ATTITUDE_DATA

#ifndef __DSO_MEASURED_STARCAMERA_SATELLITE_ATTITUDE_HPP__
#define __DSO_MEASURED_STARCAMERA_SATELLITE_ATTITUDE_HPP__

#include "datetime/calendar.hpp"
#include "eigen3/Eigen/Geometry"

namespace dso {

namespace attitude_details {

/** @brief Maximum characters in any dso-quaternion file line. */
constexpr const int MAX_ATTITUDE_LINE_CHARS = 248;

/** @brief This class is meant to hold a variable number of quaternions and
 * (rotation) angles, to represent measured attitude.
 *
 * This class can be used instead of MeasuredAttitudeData, when we need a
 * variable number of quaternins and angles to represent attitude.
 */
struct MeasuredAttitudeData {
  /* initial number of angles stored is 2 */
  static constexpr int InlineAnglesCap = 2;
  /* epoch of measured attitude in TT */
  MjdEpoch mtt;
  /* initially, we store one quaternion */
  alignas(16) Eigen::Quaterniond inline_quat_;
  /* initially, we store two doubles (angles) */
  double inline_angles_[InlineAnglesCap];
  /* we are accessing the stored quaternion(s) via this pointer */
  Eigen::Quaterniond *quat_ptr_ = &inline_quat_;
  /* we are accessing the stored angle(s) via this pointer */
  double *angles_ptr_ = inline_angles_;
  /* heap storage for quaternions (if needed) */
  Eigen::Quaterniond *heap_quat_ = nullptr;
  /* heap storage for angles (if needed) */
  double *heap_angles_ = nullptr;
  /* number of quaternins in instance (inline or heap) */
  short quat_count_ = 0;
  /* number of angles in instance (inline or heap) */
  short angles_count_ = 0;

  int num_quaternions() const noexcept { return mnq; }
  int num_angles() const noexcept { return mna; }

  /** @brief Destructor */
  ~MeasuredAttitudeData() noexcept {
    if (heap_quat_) {
      for (int i = 0; i < quat_count_; i++)
        heap_quat_[i].~Eigen::Quaterniond();
      operator delete[](heap_quat_,
                        std::align_val_t(alignof(Eigen::Quaterniond)));
    }
    if (heap_angles_) {
      for (int i = 0; i < angles_count_; i++)
        delete heap_angles_[i].~double();
      operator delete[](heap_angles_, std::align_val_t(alignof(double)));
    }
  }

  /* Copy Constructor */
  MeasuredAttitudeData(const MeasuredAttitudeData &other)
      : quat_count_(other.quat_count_), angles_count_(other.angles_count_) {
    /* Handle quaternion copy */
    if (quat_count_ > 1) {
      /* heap */
      heap_quat_ = static_cast<Eigen::Quaterniond *>(operator new[](
          quat_count_ * sizeof(Eigen::Quaterniond),
          std::align_val_t(alignof(Eigen::Quaterniond))));
      quat_ptr_ = heap_quat_;
      for (int i = 0; i < quat_count_; ++i)
        new (&heap_quat_[i]) Eigen::Quaterniond(other.quat_ptr_[i]);
    } else {
      /* stack */
      inline_quat_ = other.inline_quat_;
      quat_ptr_ = &inline_quat_;
    }

    /* Handle angles copy */
    if (angles_count_ > InlineAnglesCap) {
      /* heap */
      heap_angles_ = static_cast<double *>(operator new[](
          angles_count_ * sizeof(double), std::align_val_t(alignof(double))));
      angles_ptr_ = heap_angles_;
      std::memcpy(heap_angles_, other.angles_ptr_,
                  angles_count_ * sizeof(double));
    } else {
      /* stack */
      std::memcpy(inline_angles_, other.inline_angles_,
                  angles_count_ * sizeof(double));
      angles_ptr_ = inline_angles_;
    }
  }

  /* Copy Assignment */
  MeasuredAttitudeData &operator=(const MeasuredAttitudeData &other) {
    if (this != &other) {
      /* Use copy-and-swap idiom */
      MeasuredAttitudeData temp(other);
      swap(*this, temp);
    }
    return *this;
  }

  /* Move Constructor */
  MeasuredAttitudeData(MeasuredAttitudeData &&other) noexcept
      : inline_quat_(std::move(other.inline_quat_)),
        quat_ptr_(other.quat_ptr_ == &other.inline_quat_ ? &inline_quat_
                                                         : other.quat_ptr_),
        heap_quat_(other.heap_quat_), angles_count_(other.angles_count_),
        quat_count_(other.quat_count_) {
    std::memcpy(inline_angles_, other.inline_angles_,
                InlineAnglesCap * sizeof(double));
    angles_ptr_ = (other.angles_ptr_ == other.inline_angles_)
                      ? inline_angles_
                      : other.heap_angles_;
    /* Null out other's heap pointers */
    other.heap_quat_ = nullptr;
    other.heap_angles_ = nullptr;
    other.quat_ptr_ = &other.inline_quat_;
    other.angles_ptr_ = other.inline_angles_;
    other.quat_count_ = 0;
    other.angles_count_ = 0;
  }

  /* Move Assignment */
  MeasuredAttitudeData &operator=(MeasuredAttitudeData &&other) noexcept {
    if (this != &other) {
      /* Destroy current contents */
      this->~MeasuredAttitudeData();
      /* Placement new move */
      new (this) MeasuredAttitudeData(std::move(other));
    }
    return *this;
  }

  MeasuredAttitudeData(int nq, int na) : quat_count_(nq), angles_count_(na) {
    if (nq > 1) {
      /* allocate memory, no constructor called ...*/
      heap_quat_ = (Eigen::Quaternion *)(operator new[](
          nq * sizeof(Eigen::Quaterniond),
          std::align_val_t(alignof(Eigen::Quaterniond))));
      /* construct the quaternions */
      quat_ptr_ = heap_quat_;
      for (int i = 0; i < nq; i++)
        new (&heap_quat_[i]) Eigen::Quaterniond();
    }
    if (na > InlineAnglesCap) {
      heap_angles_ = (double *)(operator new[](
          na * sizeof(double), std::align_val_t(alignof(double))));
      /* construct angles */
      angles_ptr_ = heap_angles_;
      for (int i = 0; i < na; i++)
        new (&angles_ptr_[i]) double(0e0);
    }
  }

  /* Swap helper (used in copy-assignment) */
  friend void swap(MeasuredAttitudeData &a, MeasuredAttitudeData &b) noexcept {
    using std::swap;
    swap(a.inline_quat_, b.inline_quat_);
    for (int i = 0; i < InlineAnglesCap; ++i)
      swap(a.inline_angles_[i], b.inline_angles_[i]);

    swap(a.quat_ptr_, b.quat_ptr_);
    swap(a.angles_ptr_, b.angles_ptr_);
    swap(a.heap_quat_, b.heap_quat_);
    swap(a.heap_angles_, b.heap_angles_);
    swap(a.quat_count_, b.quat_count_);
    swap(a.angles_count_, b.angles_count_);
  }

  size_t quaternion_count() const { return quat_count_; }
  size_t angles_count() const { return angles_count_; }

  Eigen::Quaterniond &quaternion(size_t idx = 0) {
#ifdef DEBUG
    assert(idx < quat_count_);
#endif
    return quat_ptr_[idx];
  }

  const Eigen::Quaterniond &quaternion(size_t idx = 0) const {
#ifdef DEBUG
    assert(idx < quat_count_);
#endif
    return quat_ptr_[idx];
  }

  double angle(size_t idx = 0) const {
#ifdef DEBUG
    assert(idx < angles_count_);
#endif
    return angles_ptr_[idx];
  }

  double &angle(size_t idx = 0) const {
#ifdef DEBUG
    assert(idx < angles_count_);
#endif
    return angles_ptr_[idx];
  }

}; /* struct MeasuredAttitudeData */

} /* namespace attitude_details */

} /* namespace dso */

#endif

#endif
