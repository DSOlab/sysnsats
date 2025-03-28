#ifndef __DSO_ATTITUDE_QUATERNION_STREAM_HPP__
#define __DSO_ATTITUDE_QUATERNION_STREAM_HPP__

#include "datetime/calendar.hpp"
#include "eigen3/Eigen/Geometry"
#include <algorithm>
#include <array>
#include <charconv>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace dso {

namespace attitude_details {
/** @brief Maximum characters in any dso-quaternion file line. */
constexpr const int MAX_ATTITUDE_LINE_CHARS = 248;

/** @brief Left trim whitespace and '+' characters. */
inline const char *skipws(const char *line) noexcept {
  const char *str = line;
    while (*str && (*str==' ' || *str=='+') ++str;
    return str;
}
} /* namespace attitude_details */

/** @brief A quaternion record.
 *
 * Holds any number of quaternions and angles (e.g. solar array angles).
 */
template <int NumQuaternions, int NumAngles> struct DsoQuaternionRecord {
  /* reference time in TT */
  dso::MjdEpoch mtt;
  /* a list of quaternions */
  std::array<Eigen::Quaterniond, NumQuaternions> mq;
  /* a list or (rotation) angles */
  std::array<double, NumAngles> ma;
}; /* struct DsoQuaternionRecord<NumQuaternions, NumAngles> */

/** @brief A quaternion record (specialization)
 *
 * Holds a single quaternion and and any number of angles (e.g. solar array
 * angles).
 */
template <int NumAngles> struct DsoQuaternionRecord<1, NumAngles> {
  /* reference time in TT */
  dso::MjdEpoch mtt;
  /* a (single) quaternion */
  Eigen::Quaterniond mq;
  /* a list or (rotation) angles */
  std::array<double, NumAngles> ma;
}; /* struct DsoQuaternionRecord<1, NumAngles> */

/** @brief Parse a record line from a DSO quaternion file.
 *
 * @tparam NumQuaternions Number of quaternions the file holds per line.
 * @tparam NumAngles      Number of angles the file holds per line.
 * @param[in] line The record line.
 * @param[out] rec The resolved instance, if the line was successefully parsed.
 * @return Anything other than 0 denotes an error.
 */
template <int NumQuaternions, int NumAngles>
int parse_attitude_line(
    const char *line,
    DsoQuaternionRecord<NumQuaternions, NumAngles> &rec) noexcept {
  const int sz = std::strlen(line);
  const char *str = line;
  int error = 0;

  /* read quaternions first */
  for (int q = 0; q < NumQuaternions; q++) {
    double data[4];
    for (int i = 0; i < 4; i++) {
      auto res =
          std::from_chars(attitude_details::skipws(str), line + sz, data[i]);
      if (res.ec != std::errc{})
        ++error;
      str = res.ptr;
    }
    if constexpr (NumQuaternions == 1) {
      rec.mq = Eigen::Quaterniond(data[0], data[1], data[2], data[3]);
    } else {
      rec.mq[q] = Eigen::Quaterniond(data[0], data[1], data[2], data[3]);
    }
  }

  /* read angles */
  for (int a = 0; a < NumAngles; a++) {
    auto res =
        std::from_chars(attitude_details::skipws(str), line + sz, rec.ma[a]);
    if (res.ec != std::errc{})
      ++error;
    str = res.ptr;
  }

  return (!error);
}

template <int BufferSize, int NumQuaternions, int NumAngles>
class DsoQuaternionStream {
private:
  using BufferEntryType = DsoQuaternionRecord<NumQuaternions, NumAngles>;
  using BufferType = std::array<BufferEntryType, BufferSize + 1>;

  /** The (input) stream; opens at construction. */
  std::ifstream mstream;
  /** Current index in mbuf. */
  int cj;
  /** A buffer holding DsoQuaternionRecord's. */
  BufferType mbuf;

  enum class BufferSearchResult : char {
    BeforeFirstRecord,
    AfterLastRecord,
    RangeInBuffer
  }; /* BufferSearchResult */

  [[deprecated]]
  BufferSearchResult range_in_buffer(const MjdEpoch &t, int &idx) noexcept {
    auto it =
        std::lower_bound(mbuf.begin(), mbuf.end() - 1, t,
                         [](const MjdEpoch &tt, const BufferEntryType &be) {
                           return tt < be.mtt;
                         });
    if (it == mbuf.begin()) {
      return BufferSearchResult::BeforeFirstRecord;
    } else if (it == mbuf.end() - 1) {
      return BufferSearchResult::AfterLastRecord;
    } else [[likely]] {
      idx = std::distance(mbuf.begin(), it) - 1;
    }
    return BufferSearchResult::RangeInBuffer;
  }

  /** @brief Read and parse the next line of the (member) stream.
   * The resolved instance will be stored in mbuf[idx].
   *
   * @param[in] idx The index of the instance in mbuf which will hold the
   *                instance resolved. E.g. parse_next_record(4), will read
   *                the next line off from the stream, resolve it a
   *                DsoQuaternionRecord<NumQuaternions, NumAngles>, and assign
   *                this to mbuf[4].
   *  @return Anything other than zero denotes an error.
   */
  int parse_next_record(int idx) noexcept {
    char line[attitude_details::MAX_ATTITUDE_LINE_CHARS];
    mstream.getline(line, attitude_details::MAX_ATTITUDE_LINE_CHARS);
    if (mstream.good()) {
      return parse_attitude_line<NumQuaternions, NumAngles>(line, mbuf[idx]);
    }
    fprintf(
        stderr,
        "[WRNNG] Failed reading line from quaternion stream (traceback: %s)\n",
        __func__);
    return 10;
  }

  /** @brief Read BufferSize records (i.e. lines).
   *
   * Read, parse and store the BufferSize records (i.e. lines) first
   * encountered in the current stream position. Store them the instance's
   * mbuf member var.
   *
   * @return Anything other than zero denotes an error.
   */
  int initialize() noexcept {
    if (!mstream.is_open()) {
      return 100;
    }
    char line[attitude_details::MAX_ATTITUDE_LINE_CHARS];
    int i = 0, error = 0;
    while (i < BufferSize && (!error)) {
      error = parse_next_record(i);
      ++i;
    }
    mbuf[BufferSize] = mbuf[BufferSize - 1];
    /* set last buffer element */
    mbuf[BufferSize].mtt = MjdEpoch::min();
    return error;
  }

  /** @brief Fill the buffer with BufferSize-1 new quaternions from stream.
   *
   * The last quaternion currently buffered (i.e. the one residing in index
   * BufferSize-1) will be placed first at the buffer. Then, we are collecting
   * BufferSize-1 new quaternions to fill the index range
   * [Buffersize+1, BufferSize).
   *
   * The current index, i.e. cj, is set to 0.
   *
   * @return Anything other than zero denotes an error.
   */
  int collect_new_batch() noexcept {
    mbuf[0] = mbuf[BufferSize - 1];
    int i = 0, error = 0;
    while ((i < BufferSize - 1) && (!error)) {
      error = parse_next_record(i + 1);
      ++i;
    }
    cj = 0;
    return (!error);
  }

  int collect_range(const MjdEpoch &t) noexcept {
    if (mbuf[0].mtt < t) {
      fprintf(
          stderr,
          "[ERROR] Cannot read quaternion stream backwards! (traceback: %s)\n",
          __func__);
      return 1;
    }

    int error = 0;
    while (!error) {
      auto it =
          std::lower_bound(mbuf.start(), mbuf.start() + BufferSize - 1,
                           [](const MjdEpoch &tt, const BufferEntryType &bt) {
                             return tt < bt.mtt;
                           });
      if (it < mbuf.start() + BufferSize - 1) {
        cj = std::distance(mbuf.start() + it - 1);
        break;
      } else {
        error = collect_new_batch();
      }
    }

    return error;
  }

  int hunt(const MjdEpoch &t) noexcept {
    /* quick return */
    if ((t >= mbuf[cj].mtt) && (t < mbuf[cj + 1].mtt))
      return 0;

    /* no quick return; search the buffer from this point forward */
    auto it =
        std::lower_bound(mbuf.start() + cj, mbuf.start() + BufferSize - 1,
                         [](const MjdEpoch &tt, const BufferEntryType &bt) {
                           return tt < bt.mtt;
                         });
    if (it < mbuf.start() + BufferSize - 1) {
      cj = std::distance(it, mbuf.start()) - 1;
      return 0;
    }

    /* shit, interval not buffered; two possiblities:
     * a) either t > latest_buffered_quaternion, or
     * b) t < current_quaternion
     */
    if (t >= mbuf[BufferSize - 1].mtt) {
      /* case A above; collect newer quaternions */
      if (collect_new_batch()) {
        return 2;
      }
      /* quick return if we have the right interval, else hunt */
      return ((t >= mbuf[cj].mtt) && (t < mbuf[cj + 1].mtt)) ? 0 : hunt(t);
    } else {
      /* case B above */
      if (t <= mbuf[0].mtt) {
        fprintf(stderr,
                "[ERROR] Request for quaternion which is prior to current "
                "buffered block! (traceback: %s)\n",
                __func__);
        return 30;
      }
      /* search for interval from the top of the buffer */
      auto it =
          std::lower_bound(mbuf.start(), mbuf.start() + cj + 1,
                           [](const MjdEpoch &tt, const BufferEntryType &bt) {
                             return tt < bt.mtt;
                           });
      assert(it < mbuf.start() + cj + 1);
      cj = std::distance(it, mbuf.start()) - 1;
      return 0;
    }
    return 80;
  }

public:
  DsoQuaternionStream(const char *fn) : mstream(fn), mbuf() {
    if (initialize()) {
      throw std::runtime_error(
          "[ERROR] Failed to initialize quternion stream from file " +
          std::string(fn) + std::string("\n"));
    }
  };
  DsoQuaternionStream(const char *fn, const MjdEpoch &t) : mstream(fn), mbuf() {
    int error = initialize();
    if (error || collect_range(t)) {
      throw std::runtime_error(
          "[ERROR] Failed to initialize quternion stream from file " +
          std::string(fn) + std::string("\n"));
    }
  };
  DsoQuaternionStream(const DsoQuaternionStream &other) = delete;
  DsoQuaternionStream &operator=(const DsoQuaternionStream &other) = delete;
  DsoQuaternionStream(DsoQuaternionStream &&other) noexcept;        // TODO
  DsoQuaternionStream &operator=(const DsoQuaternionStream &other); // TODO
}; /* DsoQuaternionStream */

} /* namespace dso */

#endif
