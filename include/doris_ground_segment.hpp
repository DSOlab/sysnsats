#ifndef __DSO_SATELLITE_SYSTEM_DORIS_GROUND_SEGMENT_HPP__
#define __DSO_SATELLITE_SYSTEM_DORIS_GROUND_SEGMENT_HPP__

#include "eigen3/Eigen/Eigen"
#include "satellite.hpp"
#include <limits>
#include <stdexcept>

/* Reference Document:
 * https://ids-doris.org/documents/BC/stations/DORIS_System_Ground_Segment_Models.pdf
 */

namespace dso {

enum class DORIS_GROUND_ANTENNA_TYPE : int_fast8_t {
  Alcatel,
  StarecB,
  StarecC
};

inline DORIS_GROUND_ANTENNA_TYPE char2doris_ground_antenna_type(char c) {
  switch (c) {
  case 'A':
    return DORIS_GROUND_ANTENNA_TYPE::Alcatel;
  case 'B':
    return DORIS_GROUND_ANTENNA_TYPE::StarecB;
  case 'C':
    return DORIS_GROUND_ANTENNA_TYPE::StarecC;
    throw std::runtime_error(
        "[ERROR] Failed to translate given char to a valid DORIS ground "
        "antenna type! (traceback: char2doris_ground_antenna_type)\n");
  }
}

template <DORIS_GROUND_ANTENNA_TYPE> struct DorisGroundAntennaTraits {};

template <>
struct DorisGroundAntennaTraits<DORIS_GROUND_ANTENNA_TYPE::Alcatel> {
  /* eccentricity from antenna reference plane to 2GHz Phase center in
   * topocentric (e, n, u) compoenents, [m]
   */
  static Eigen::Vector3d pco_enu_2GHz_ecc() noexcept {
    Eigen::Vector3d ecc;
    ecc << 0e0, 0e0, 510e-3;
    return ecc;
  }
  /* eccentricity from antenna reference plane to 400MHz Phase center in
   * topocentric (e, n, u) compoenents, [m]
   */
  static Eigen::Vector3d pco_enu_400MHz_ecc() noexcept {
    Eigen::Vector3d ecc;
    ecc << 0e0, 0e0, 335 - 3;
    return ecc;
  }
}; /* DorisGroundAntennaTraits<DORIS_GROUND_ANTENNA_TYPE::Alcatel> */

template <>
struct DorisGroundAntennaTraits<DORIS_GROUND_ANTENNA_TYPE::StarecB> {
  /* eccentricity from antenna reference plane to 2GHz Phase center in
   * topocentric (e, n, u) compoenents, [m]
   */
  static Eigen::Vector3d pco_enu_2GHz_ecc() noexcept {
    Eigen::Vector3d ecc;
    ecc << 0e0, 0e0, 487e-3;
    return ecc;
  }
  /* eccentricity from antenna reference plane to 400MHz Phase center in
   * topocentric (e, n, u) compoenents, [m]
   */
  static Eigen::Vector3d pco_enu_400MHz_ecc() noexcept {
    return Eigen::Vector3d::Zero();
  }
}; /* DorisGroundAntennaTraits<DORIS_GROUND_ANTENNA_TYPE::StarecB> */

template <>
struct DorisGroundAntennaTraits<DORIS_GROUND_ANTENNA_TYPE::StarecC> {
  /* eccentricity from antenna reference plane to 2GHz Phase center in
   * topocentric (e, n, u) compoenents, [m]
   */
  static Eigen::Vector3d pco_enu_2GHz_ecc() noexcept {
    Eigen::Vector3d ecc;
    ecc << 0e0, 0e0, 487e-3;
    return ecc;
  }
  /* eccentricity from antenna reference plane to 400MHz Phase center in
   * topocentric (e, n, u) compoenents, [m]
   */
  static Eigen::Vector3d pco_enu_400MHz_ecc() noexcept {
    return Eigen::Vector3d::Zero();
  }
}; /* DorisGroundAntennaTraits<DORIS_GROUND_ANTENNA_TYPE::StarecC> */

void antenna_pco_enu(DORIS_GROUND_ANTENNA_TYPE type, Eigen::Vector3d &denu_2GHz,
                     Eigen::Vector3d &denu_400MHz) {
  switch (type) {
  case DORIS_GROUND_ANTENNA_TYPE::Alcatel:
    denu_2GHz = DorisGroundAntennaTraits<
        DORIS_GROUND_ANTENNA_TYPE::Alcatel>::pco_enu_2GHz_ecc();
    denu_400MHz = DorisGroundAntennaTraits<
        DORIS_GROUND_ANTENNA_TYPE::Alcatel>::pco_enu_400MHz_ecc();
    break;
  case DORIS_GROUND_ANTENNA_TYPE::StarecB:
    denu_2GHz = DorisGroundAntennaTraits<
        DORIS_GROUND_ANTENNA_TYPE::StarecB>::pco_enu_2GHz_ecc();
    denu_400MHz = DorisGroundAntennaTraits<
        DORIS_GROUND_ANTENNA_TYPE::StarecB>::pco_enu_400MHz_ecc();
    break;
  case DORIS_GROUND_ANTENNA_TYPE::StarecC:
    denu_2GHz = DorisGroundAntennaTraits<
        DORIS_GROUND_ANTENNA_TYPE::StarecC>::pco_enu_2GHz_ecc();
    denu_400MHz = DorisGroundAntennaTraits<
        DORIS_GROUND_ANTENNA_TYPE::StarecC>::pco_enu_400MHz_ecc();
    break;
  }
  return;
}

} /* namespace dso */

#endif
