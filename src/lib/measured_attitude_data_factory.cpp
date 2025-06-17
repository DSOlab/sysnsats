#include "attitude.hpp"

dso::attitude_details::MeasuredAttitudeData dso::measured_attitude_data_factory(dso::SATELLITE sat) {
  switch (sat) {
        case (SATELLITE::JASON1):
      return dso::attitude_details::MeasuredAttitudeData(
          SatelliteAttitudeTraits<SATELLITE::JASON1>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON1>::NumAngles);

    case (SATELLITE::JASON2):
      return dso::attitude_details::MeasuredAttitudeData(
          SatelliteAttitudeTraits<SATELLITE::JASON2>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON2>::NumAngles);

    case (SATELLITE::JASON3):
      return dso::attitude_details::MeasuredAttitudeData(
          SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles);

    case (SATELLITE::SENTINEL3A):
      return dso::attitude_details::MeasuredAttitudeData(
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumAngles);

    case (SATELLITE::SENTINEL3B):
      return dso::attitude_details::MeasuredAttitudeData(
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumAngles);

    case (SATELLITE::SENTINEL6A):
      return dso::attitude_details::MeasuredAttitudeData(
          SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumAngles);
    
    case (SATELLITE::SWOT):
      return dso::attitude_details::MeasuredAttitudeData(
          SatelliteAttitudeTraits<SATELLITE::SWOT>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SWOT>::NumAngles);
    default:
      throw std::runtime_error(
          "[ERROR] Failed to construct MeasuredAttitude\n");
    }
}
