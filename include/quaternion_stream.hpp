#ifndef __DSO_ATTITUDE_QUATERNION_STREAM_HPP__
#define __DSO_ATTITUDE_QUATERNION_STREAM_HPP__

#include <fstream>
#include "eigen3/Eigen/Geometry"
#include "datetime/calendar.hpp"
#include <charconv>
#include <array>
#include <cstring>

namespace dso {

namespace attitude_details {
  /** @brief Maximum characters in any dso-quaternion file line. */
  constexpr const int MAX_ATTITUDE_LINE_CHARS = 248;

  inline const char * skipws(const char *line) noexcept {
    const char *str = line;
    while (*str && (*str==' ' || *str=='+') ++str;
    return str;
  }
} /* namespace attitude_details */

/** @brief A quaternion record.
 *
 * Holds any number of quaternions and angles (e.g. solar array angles).
 */
template<int NumQuaternions, int NumAngles>
struct DsoQuaternionRecord {
  dso::MjdEpoch t_tt;
  std::array<Eigen::Quaterniond, NumQuaternions> mq;
  std::array<double, NumAngles> ma;
}; /* struct DsoQuaternionRecord<NumQuaternions, NumAngles> */

/** @brief A quaternion record (specialization)
 *
 * Holds a single quaternion and andy number of angles (e.g. solar array 
 * angles).
 */
template<int NumAngles>
struct DsoQuaternionRecord<1, NumAngles> {
  dso::MjdEpoch t_tt;
  Eigen::Quaterniond mq;
  std::array<double, NumAngles> ma;
}; /* struct DsoQuaternionRecord<1, NumAngles> */

template<int NumQuaternions, int NumAngles>
parse_attitude_line(const char *line, DsoQuaternionRecord<NumQuaternions, NumAngles> &rec) noexcept {
  const int sz = std::strlen(line);
  const char *str = line;
  int error=0;

  /* read quaternions first */
  for (int q=0; q<NumQuaternions; q++) {
    double data[4];
    for (int i=0; i<4; i++) {
      auto res = std::from_chars(attitude_details::skipws(str), line+sz, data[i]);
      if (res.ec != std::errc{}) ++error;
      str = res.ptr;
    }
    rec.mq[q] = Eigen::Quaterniond(data[0], data[1], data[2], data[3]);
  }

  /* read angles */
  for (int a=0; a<NumAngles; a++) {
    auto res = std::from_chars(attitude_details::skipws(str), line+sz, rec.ma[a]);
    if (res.ec != std::errc{}) ++error;
    str = res.ptr;
  }

  return (!error);
}

template<int BufferSize, int NumQuaternions, int NumAngles>
class DsoQuaternionStream {
  private:
  std::ifstream mstream;
  std::array<DsoQuaternionRecord<NumQuaternions, NumAngles>, BufferSize> mbuf;

  int stream_get() noexcept {
  }

  int initialize() noexcept {
    if (!mstream.is_open()) {
      return 100;
    }
    char line[attitude_details::MAX_ATTITUDE_LINE_CHARS];
    for (int i=0; i<BufferSize; i++) {
      int error = stream_get(i);
    }

  }
  
  public:
    DsoQuaternionStream(const char *fn) : mstream(fn), mbuf() {};
    DsoQuaternionStream(const DsoQuaternionStream &other) = delete;
    DsoQuaternionStream& operator=(const DsoQuaternionStream &other) = delete;
    DsoQuaternionStream(DsoQuaternionStream &&other) noexcept; // TODO
    DsoQuaternionStream& operator=(const DsoQuaternionStream &other); // TODO
}; /* DsoQuaternionStream */

} /* namespace dso */

#endif
