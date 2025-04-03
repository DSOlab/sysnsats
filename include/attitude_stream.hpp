#include "attitude.hpp"

namespace dso {

class Satellite {
private:
  SATELLITE msat;
  DsoAttitudeStream<satellite_details::QuaternionStreamBufferSize> *matt =
      nullptr;
  // dso::SatelliteMacromodel<> *macrom;
  // satellite_details::BaseMacromodel *msgm = nullptr;

  // satellite_details::BaseMacromodel *load_geometry_impl() const noexcept {
  //   switch (msat) {
  //   case (SATELLITE::JASON1):
  //     return new SatelliteMacromodel<SATELLITE::JASON1>();
  //   case (SATELLITE::JASON2):
  //     return new SatelliteMacromodel<SATELLITE::JASON2>();
  //   case (SATELLITE::JASON3):
  //     return new SatelliteMacromodel<SATELLITE::JASON3>();
  //   case (SATELLITE::SENTINEL3A):
  //     return new SatelliteMacromodel<SATELLITE::SENTINEL3A>();
  //   case (SATELLITE::SENTINEL3B):
  //     return new SatelliteMacromodel<SATELLITE::SENTINEL3B>();
  //   case (SATELLITE::SENTINEL6A):
  //     return new SatelliteMacromodel<SATELLITE::SENTINEL6A>();
  //   default:
  //     return nullptr;
  //   }
  // }

  DsoAttitudeStream<satellite_details::QuaternionStreamBufferSize> *
  init_attitude_stream(const char *fn) const noexcept {
    switch (msat) {
    case (SATELLITE::JASON1):
      return new DsoAttitudeStream<
          satellite_details::QuaternionStreamBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON1>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON1>::NumAngles);
    case (SATELLITE::JASON2):
      return new DsoAttitudeStream<
          satellite_details::QuaternionStreamBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON2>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON2>::NumAngles);
    case (SATELLITE::JASON3):
      return new DsoAttitudeStream<
          satellite_details::QuaternionStreamBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles);
    case (SATELLITE::SENTINEL3A):
      return new DsoAttitudeStream<
          satellite_details::QuaternionStreamBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumAngles);
    case (SATELLITE::SENTINEL3B):
      return new DsoAttitudeStream<
          satellite_details::QuaternionStreamBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumAngles);
    case (SATELLITE::SENTINEL6A):
      return new DsoAttitudeStream<
          satellite_details::QuaternionStreamBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumAngles);
    default:
      return nullptr;
    }
  }

public:
  Satellite(const char *sat_name) : msat(translate_satid(sat_name)) {}
  ~Satellite() noexcept {
    if (matt)
      delete matt;
  }

  int load_attitude(const char *fn) noexcept {
    return (this->matt = this->init_attitude_stream(fn)) == nullptr;
  }

  int attitude_at(const MjdEpoch &tt, Eigen::Quaterniond *quaternions,
                  double *angles) noexcept {
    if (matt) {
      return matt->angles_at(tt, quaternions, angles);
    }

    return -100;
  }
};

} /* namespace dso */
