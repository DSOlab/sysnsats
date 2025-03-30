#include "quaternion_stream.hpp"
#include "satellite.hpp"

namespace dso {

namespace satellite_details {
constexpr const int QuaternionStreamBufferSize = 15;
} /* namespace satellite_details */

class Satellite {
private:
  SATELLITE msat;
  // dso::QuaternionStream<> *qstream;
  satellite_details::DsoAttitudeStreamBase *matt = nullptr;
  // dso::SatelliteMacromodel<> *macrom;
  satellite_details::BaseMacromodel *msgm = nullptr;

  satellite_details::BaseMacromodel *load_geometry_impl() const noexcept {
    switch (msat) {
    case (SATELLITE::JASON1):
      return new SatelliteMacromodel<SATELLITE::JASON1>();
    case (SATELLITE::JASON2):
      return new SatelliteMacromodel<SATELLITE::JASON2>();
    case (SATELLITE::JASON3):
      return new SatelliteMacromodel<SATELLITE::JASON3>();
    case (SATELLITE::SENTINEL3A):
      return new SatelliteMacromodel<SATELLITE::SENTINEL3A>();
    case (SATELLITE::SENTINEL3B):
      return new SatelliteMacromodel<SATELLITE::SENTINEL3B>();
    case (SATELLITE::SENTINEL6A):
      return new SatelliteMacromodel<SATELLITE::SENTINEL6A>();
    default:
      return nullptr;
    }
  }

  satellite_details::DsoAttitudeStreamBase *
  init_attitude_stream(const char *fn) const noexcept {
    switch (msat) {
    case (SATELLITE::JASON1):
      return new DsoQuaternionStream<
          satellite_details::QuaternionStreamBufferSize, 1, 2>(fn);
    case (SATELLITE::JASON2):
      return new DsoQuaternionStream<
          satellite_details::QuaternionStreamBufferSize, 1, 2>(fn);
    case (SATELLITE::JASON3):
      return new DsoQuaternionStream<
          satellite_details::QuaternionStreamBufferSize, 1, 2>(fn);
    case (SATELLITE::SENTINEL3A):
      return new DsoQuaternionStream<
          satellite_details::QuaternionStreamBufferSize, 1, 0>(fn);
    case (SATELLITE::SENTINEL3B):
      return new DsoQuaternionStream<
          satellite_details::QuaternionStreamBufferSize, 1, 0>(fn);
    case (SATELLITE::SENTINEL6A):
      return new DsoQuaternionStream<
          satellite_details::QuaternionStreamBufferSize, 1, 0>(fn);
    default:
      return nullptr;
    }
  }

public:
  Satellite(const char *sat_name) : msat(translate_satid(sat_name)) {}
  ~Satellite() noexcept {
    if (matt)
      delete matt;
    if (msgm)
      delete msgm;
  }

  int load_geometry() noexcept {
    return (this->msgm = this->load_geometry_impl()) == nullptr;
  }

  int load_attitude(const char *fn) noexcept {
    return (this->matt = this->init_attitude_stream(fn)) == nullptr;
  }
};

} /* namespace dso */
