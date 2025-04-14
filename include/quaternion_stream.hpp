#ifndef __DSO_ATTITUDE_QUATERNION_STREAM_HPP__
#define __DSO_ATTITUDE_QUATERNION_STREAM_HPP__

#include "core/measured_attitude_data.hpp"
#include "datetime/calendar.hpp"
#include "datetime/datetime_write.hpp"
#include "eigen3/Eigen/Geometry"
#include <algorithm>
#include <array>
#include <charconv>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace dso {

/** @brief Parse a record line from a DSO quaternion file.
 *
 * @param[in] num_quaternions Number of quaternions the file holds per line.
 * @param[in] num_angles      Number of angles the file holds per line.
 * @param[in] line The record line.
 * @param[out] tt The epoch record of the line resolved.
 * @param[out] qout An array of Eigen::Quaterniond's of size at least
 * num_quaternions. At output (if successeful), the first num_quaternions will
 * hold the quaternions parsed.
 * @param[out] aout An array of doubles of size at least num_angles. At output
 * (if successeful), the first num_angles will hold the angles parsed.
 * @return Anything other than 0 denotes an error.
 */
int parse_attitude_line(const char *line,
                        attitude_details::MeasuredAttitudeData &data) noexcept;

template <int BufferSize> class DsoAttitudeStream {
private:
  /** The (input) stream; opens at construction. */
  std::ifstream mstream;
  /** Current index in mbuf. */
  int cj;
  /** A buffer holding MeasuredAttitudeData's. */
  std::array<attitude_details::MeasuredAttitudeData, BufferSize> mbuf;

  /** @brief Interpolate the quaternions stored in mbuf[cj] and mbuf[cj+1].
   *
   * This function will use the quaternions stored in mbuf[cj] and mbuf[cj+1] to
   * interpolate quaternion values for the epoch tt. Note that the epoch should
   * be between mbuf[cj] and mbuf[cj+1], i.e. mbuf[cj].mtt <= tt < mbuf[cj+1]
   * but this is not checked here!
   *
   * The interpolation is performed using the SLERP algorithm. The number of
   * different quaternion (pairs) to be interpolated are found by the passed
   * in attitude_details::MeasuredAttitudeData instance (i.e. att.mnq).
   *
   * The computed quaternions are store in the att instance.attitude_details
   *
   * @param[in] tt Epoch of interpolation request, TT.
   * @param[in] att A attitude_details::MeasuredAttitudeData instance. The
   * number of quaternions stored in here (i.e. att.mnq) is the number of
   * quaternions we are going to interpolate. Hence, att.mnq should be <= to the
   * attitude_details::MeasuredAttitudeData instances hold in buffer. Most
   * probably, the equality should hold. At output, the epoch of the instance
   * will be set to tt, and the stored quaternion will hold the computed values.
   * @return Always zero.
   */
  int qslerp(const MjdEpoch &tt,
             attitude_details::MeasuredAttitudeData &att) const noexcept {
#ifdef DEBUG
    assert(tt >= mbuf[cj].mtt && tt < mbuf[cj + 1].mtt);
#endif
    /* t2 - t1 in seconds */
    const auto interval_sec =
        mbuf[cj + 1]
            .t()
            .template diff<DateTimeDifferenceType::FractionalSeconds>(
                mbuf[cj].t());
    /* tt - t1 in seconds */
    const auto part_sec =
        tt.diff<DateTimeDifferenceType::FractionalSeconds>(mbuf[cj].t());
    /* interpolation factor (0.0 gives q1, 1.0 gives q2) */
    const double f = part_sec.seconds() / interval_sec.seconds();
    /* slerp interpolation; assign to att */
    for (int q = 0; q < att.num_quaternions(); q++) {
      att.quaternions()[q] =
          mbuf[cj].quaternions()[q].slerp(f, mbuf[cj + 1].quaternions()[q]);
    }

    return 0;
  }

  /** @brief Interpolate the angles stored in mbuf[cj] and mbuf[cj+1].
   *
   * This function will use the angles stored in mbuf[cj] and mbuf[cj+1] to
   * interpolate values for the epoch tt. Note that the epoch should
   * be between mbuf[cj] and mbuf[cj+1], i.e. mbuf[cj].mtt <= tt < mbuf[cj+1]
   * but this is not checked here!
   *
   * The interpolation is performed using a linear interpolation algorithm. The
   * number of different angles (pairs) to be interpolated are found by the
   * passed in attitude_details::MeasuredAttitudeData instance (i.e. att.mna).
   *
   * The computed angles are stored in the att instance.attitude_details
   *
   * @param[in] tt Epoch of interpolation request, TT.
   * @param[in] att A attitude_details::MeasuredAttitudeData instance. The
   * number of angles stored in here (i.e. att.mna) is the number of
   * angles we are going to interpolate. Hence, att.mna should be <= to the
   * attitude_details::MeasuredAttitudeData instances hold in buffer. Most
   * probably, the equality should hold. At output, the epoch of the instance
   * will be set to tt, and the stored angles will hold the computed values.
   * @return Always zero.
   */
  int aintrpl(const MjdEpoch &tt,
              attitude_details::MeasuredAttitudeData &att) const noexcept {
#ifdef DEBUG
    assert(tt >= mbuf[cj].mtt && tt < mbuf[cj + 1].mtt);
#endif
    /* t2 - t1 in seconds */
    const auto interval_sec =
        mbuf[cj + 1]
            .t()
            .template diff<DateTimeDifferenceType::FractionalSeconds>(
                mbuf[cj].t());
    /* tt - t1 in seconds */
    const auto part_sec =
        tt.diff<DateTimeDifferenceType::FractionalSeconds>(mbuf[cj].t());
    /* interpolation factor (0.0 gives q1, 1.0 gives q2) */
    const double f = part_sec.seconds() / interval_sec.seconds();
    /* linear interpolation */
    for (int a = 0; a < att.num_angles(); a++) {
      att.angles()[a] = mbuf[cj].angles()[a] +
                        f * (mbuf[cj].angles()[a] - mbuf[cj + 1].angles()[a]);
    }
    return 0;
  }

  /** @brief Read and parse the next line of the (member) stream.
   * The resolved instance will be stored in mbuf[idx].
   *
   * @param[in] idx The index of the instance in mbuf which will hold the
   *                instance resolved. E.g. parse_next_record(4), will read
   *                the next line off from the stream, resolve it to a
   *                attitude_details::MeasuredAttitudeData, and assign
   *                this to mbuf[4].
   *  @return Anything other than zero denotes an error.
   */
  int parse_next_record(attitude_details::MeasuredAttitudeData &data) noexcept {
    char line[attitude_details::MAX_ATTITUDE_LINE_CHARS];

    if (mstream.getline(line, attitude_details::MAX_ATTITUDE_LINE_CHARS)) {
      return parse_attitude_line(line, data);
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
      fprintf(stderr,
              "[ERROR] Stream is closed! Cannot initialize (traceback: %s)\n",
              __func__);
      return 100;
    }
    int i = 0, error = 0;
    while (i < BufferSize && (!error)) {
      /* parse record and store at index i */
      error = parse_next_record(mbuf[i]);
      ++i;
    }

    if (error) {
      fprintf(stderr,
              "[ERROR] Failed parsing quaternion line (traceback: %s)\n",
              __func__);
    }
    return error;
  }

  /** @brief Fill the buffer with n new records from stream.
   *
   * Read and collect the next n records off from the stream and store them in
   * the instance's buffer. For these new records to be stored, the recods
   * already in the buffer are "left-shifted" by the right amount. The n new
   * records read will be placed at the rightmost indexes of the buffer.
   *
   * The current index, i.e. cj, is set to BufferSize - n - 1, i.e. just before
   * the first new record.
   *
   * @param[n] Number of new records to read from the store and store. The
   * following must hold: 0 < n <= BufferSize.
   * @return Anything other than zero denotes an error.
   */
  int collect_new_batch(int n = BufferSize - 1) noexcept {
#ifdef DEBUG
    assert(n > 0 && n <= BufferSize);
#endif
    /* left shift */
    for (int i = 0; i < n; i++) {
      mbuf[i] = std::move(mbuf[BufferSize - n + i]);
    }
    // if (n < BufferSize / 2 - 1) {
    //   /* non-overlapping */
    //   std::memcpy(mbuf.data(), mbuf.data() + BufferSize - n,
    //               sizeof(attitude_details::MeasuredAttitudeData) * n);
    // } else {
    //   /* overlapping */
    //   std::memmove(mbuf.data(), mbuf.data() + BufferSize - n,
    //                sizeof(attitude_details::MeasuredAttitudeData) * n);
    // }
    int i = BufferSize - n;
    int error = 0;
    while ((i < BufferSize) && (!error)) {
      error = parse_next_record(mbuf[i]);
      ++i;
    }
    cj = BufferSize - n - 1;
    return (!error);
  }

  /** @brief Read data off from the stream untill we reach (and pass) t.
   *
   * The function will keep on reading and collecting new data off from the
   * stream, untill the instance's buffer holds an interval for which
   * buffer[j].mtt <= t < buffer[j+1].mtt; i.e. the epoch is availble within
   * the buffer (e.g. for interpolation).
   *
   * @param[in] t The epoch of request in TT.
   * @return Anything other than 0, denotes an error (could also be EOF).
   */
  int collect_range(const MjdEpoch &t) noexcept {
    if (mbuf[0].t() < t) {
      fprintf(
          stderr,
          "[ERROR] Cannot read quaternion stream backwards! (traceback: %s)\n",
          __func__);
      return 1;
    }

    int error = 0;
    while (!error) {
      /* check if we already have the epoch before the last element */
      auto it =
          std::lower_bound(mbuf.begin() + cj, mbuf.begin() + BufferSize - 1, t,
                           [](const attitude_details::MeasuredAttitudeData &bt,
                              const MjdEpoch &tval) { return bt.t() < tval; });
      if (it < mbuf.begin() + BufferSize - 1) {
        /* ok, got it! */
        cj = std::distance(mbuf.begin(), it) - 1;
        break;
      } else {
        /* nope, go further in the stream */
        error = collect_new_batch(2 * BufferSize / 3);
      }
    }

    return error;
  }

  /** @brief Place the instance's index cj at an interval for which:
   * buf[cj].t<=t<buf[cj+1].t
   *
   * If needed, the function may use the stream to load new data to the buffer.
   * At success, on function output, cj will be conviniently placed such that:
   * buf[cj].t<=t<buf[cj+1].t
   *
   * @param[in] t The epoch of request in TT.
   * @return Anything other than 0, denotes an error (could also be EOF).
   */
  int hunt(const MjdEpoch &t) noexcept {
#ifdef DEBUG
    assert(cj < BufferSize - 1);
#endif
    /* quick return */
    if ((t >= mbuf[cj].t()) && (t < mbuf[cj + 1].t()))
      return 0;

    /* no quick return; search the buffer from this point forward */
    auto it =
        std::lower_bound(mbuf.begin() + cj, mbuf.begin() + BufferSize - 1, t,
                         [](const attitude_details::MeasuredAttitudeData &bt,
                            const MjdEpoch &tval) { return bt.t() < tval; });
    if (it < mbuf.begin() + BufferSize - 1) {
      /* got it, place the index and return */
      cj = std::distance(it, mbuf.begin()) - 1;
      return 0;
    }

    /* shit, interval not buffered; two possiblities:
     * a) either t >= latest_buffered_quaternion, or
     * b) t < current_quaternion
     */
    if (t >= mbuf[BufferSize - 1].t()) {
      /* case A above; collect newer quaternions */
      if (collect_new_batch(2 * BufferSize / 3)) {
        return 2;
      }
      /* quick return if we have the right interval, else hunt */
      return ((t >= mbuf[cj].t()) && (t < mbuf[cj + 1].t())) ? 0 : hunt(t);
    } else {
      /* case B above */
      if (t <= mbuf[0].t()) {
        fprintf(stderr,
                "[ERROR] Request for quaternion which is prior to current "
                "buffered block! (traceback: %s)\n",
                __func__);
        return 30;
      }
      /* search for interval from the top of the buffer */
      it = std::lower_bound(mbuf.begin(), mbuf.begin() + cj + 1, t,
                            [](const attitude_details::MeasuredAttitudeData &bt,
                               const MjdEpoch &tval) { return bt.t() < tval; });
      assert(it < mbuf.begin() + cj + 1);
      cj = std::distance(it, mbuf.begin()) - 1;
      return 0;
    }

    /* should never reach this point! */
    return 80;
  }

  /** @brief Helper function to create mbuf.
   *
   * mbuf cannot be default initialized in the constructor, cause
   * atiitude_details::MeasuredAttitudeData does not have a default constrctor.
   * This function, helps construct such an array so that it can be used in the
   * constructor body.
   * It only serves this purpose and should not be used otherwise.
   */
  template <typename T, std::size_t... Is>
  constexpr std::array<T, sizeof...(Is)>
  make_array_impl(int q, int a, std::index_sequence<Is...>) {
    return {{(static_cast<void>(Is), T(q, a))...}};
  }

  template <typename T, std::size_t N>
  constexpr std::array<T, N> make_array(int q, int a) {
    return make_array_impl<T>(q, a, std::make_index_sequence<N>{});
  }

public:
  DsoAttitudeStream(const char *fn, int numq, int numa,
                    const MjdEpoch &t = MjdEpoch::min())
      : mstream(fn), cj(-1),
        mbuf(make_array<attitude_details::MeasuredAttitudeData, BufferSize>(
            numq, numa)) {
    // mbuf(DsoAttitudeStream::constructBuffer(numq, numa)) {
    /* initialize first BufferSize instances from stream */
    if (initialize()) {
      throw std::runtime_error(
          "[ERROR] Failed to initialize quternion stream from file " +
          std::string(fn) + std::string("\n"));
    }
    /* if a date is passed in, collect the required data for the epoch */
    if (t != MjdEpoch::min()) {
      if (collect_range(t)) {
        throw std::runtime_error(
            "[ERROR] Failed to initialize quternion stream from file " +
            std::string(fn) + "for given epoch\n");
      }
    }
  }

  /** @brief No copy constructor. */
  DsoAttitudeStream(const DsoAttitudeStream &other) = delete;
  /** @brief No assignment operator. */
  DsoAttitudeStream &operator=(const DsoAttitudeStream &other) = delete;
  /** @brief Move constructor. */
  DsoAttitudeStream(DsoAttitudeStream &&other) noexcept
      : mstream(std::move(other.mstream)), cj(other.cj),
        mbuf(std::move(other.mbuf)) {}
  /** @brief Move assignment operator. */
  DsoAttitudeStream &operator=(DsoAttitudeStream &&other) noexcept {
    mstream = std::move(other.mstream);
    cj = other.cj;
    mbuf = std::move(other.mbuf);
    return *this;
  }

  /** @brief Get the (measured) attitude at a given epoch from the stream.
   *
   * @param[in] tt The epoch of request, in TT.
   * @return Anything other than 0 denotes an error.
   */
  int attitude_at(const MjdEpoch &tt,
                  attitude_details::MeasuredAttitudeData &att) noexcept {
    /* lets get at the right interval (in buffer) */
    if (this->hunt(tt)) {
      char buf[64];
      fprintf(stderr,
              "[ERROR] Failed to get measured attitude for epoch: %s (TT) "
              "(traceback: %s)\n",
              to_char<YMDFormat::YYYYMMDD, HMSFormat::HHMMSSF>(tt, buf),
              __func__);
      return 1;
    }

    /* interpolate quaternion(s) */
    this->qslerp(tt, att);

    /* interpolate angle(s) */
    this->aintrpl(tt, att);

    return 0;
  }

}; /* DsoAttitudeStream */

} /* namespace dso */

#endif
