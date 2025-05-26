#ifndef __DSO_ATTITUDE_QUATERNION_STREAM_HPP__
#define __DSO_ATTITUDE_QUATERNION_STREAM_HPP__

/* @file
 *
 * This file is meant to implement a class that will enable streaming-like,
 * forward parsing of attitude data. "streaming-like" means that the class
 * can walk forward in the data file, parse records and store them in a
 * buffer. This buffer can then be used to query satellite specific attitude
 * data, i.e. quaternions and/or rotation angles.
 *
 * The data files can be different for each satellite; we need different number
 * of quaternions and angles depending on satellite structure. The data files
 * are created using the 'attitude' project (see
 * https://github.com/xanthospap/attitude).
 */

#include "datetime/calendar.hpp"
#include "datetime/datetime_write.hpp"
#include "eigen3/Eigen/Geometry"
#include "measured_attitude_data.hpp"
#include "vmeasured_attitude_data.hpp"
#include <algorithm>
#include <array>
#include <charconv>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace dso {

/** @brief Parse a record line from a DSO quaternion file.
 *
 * @param[in] line  The record line (from a DSO measured attitude file, see
 *                  https://github.com/xanthospap/attitude).
 * @param[out] data A attitude_details::MeasuredAttitudeData instance that
 *                  will hold (at output) the parsed data. The number of
 *                  quaternions/angles to be read off from the line, are
 *                  extracted from this instance.
 * @return Anything other than 0 denotes an error.
 */
int parse_attitude_line(const char *line,
                        attitude_details::MeasuredAttitudeData &data) noexcept;

/** @brief A class to enable streaming-like functionality for DSO attitude
 * files.
 *
 * @tparam BufferSize The number of attitude records to store in the internal
 * buffer.
 *
 * @example test_jason3_measured_attitude.cpp
 */
template <int BufferSize> class DsoAttitudeStream {
private:
  /** The (input) stream; opens at construction. */
  std::ifstream mstream;
  /** Current index in mbuf (always in range [0, BufferSize) except if otherwise
   * noted). */
  int cj;
  /** A buffer holding MeasuredAttitudeData's. */
  std::array<attitude_details::MeasuredAttitudeData, BufferSize> mbuf;

#ifdef DEBUG
  void debug_report(const MjdEpoch &tt) const noexcept {
    char buf[64];
    printf("DEBUG Report:\n Here is the buffer: ");
    int i = 0;
    for (const auto &d : mbuf) {
      printf("Buffer[%2d] -> %s\n", i++,
             to_char<YMDFormat::YYYYMMDD, HMSFormat::HHMMSSF>(d.t(), buf));
    }
    printf("Epoch requested: %s\n",
           to_char<YMDFormat::YYYYMMDD, HMSFormat::HHMMSSF>(tt, buf));
    printf("Index is:        %d\n", cj);
    printf("DEBUG end of report\n");
  }
#endif

  /** @brief Interpolate the quaternions stored in mbuf[cj] and mbuf[cj+1].
   *
   * This function will use the quaternions stored in mbuf[cj] and mbuf[cj+1] to
   * interpolate quaternion values for the epoch tt. Note that the epoch should
   * be between mbuf[cj] and mbuf[cj+1], i.e. mbuf[cj].mtt <= tt < mbuf[cj+1]
   * but this is not checked here!
   *
   * The interpolation is performed using the SLERP algorithm. The number of
   * different quaternion (pairs) to be interpolated are found by the passed
   * in attitude_details::MeasuredAttitudeData instance (i.e.
   * att.num_quaternions).
   *
   * The computed quaternions are store in the passed in att instance.
   *
   * @param[in] tt Epoch of interpolation request, TT.
   * @param[in] att A attitude_details::MeasuredAttitudeData instance. The
   * number of quaternions stored in here (i.e. att.num_quaternions) is the
   * number of quaternions we are going to interpolate. Hence,
   * att.num_quaternions should be
   * <= to the attitude_details::MeasuredAttitudeData instances hold in buffer.
   * Most probably, the equality should hold. At output, the epoch of the
   * instance will be set to tt, and the stored quaternion will hold the
   * computed values.
   * @return Always zero.
   */
  int qslerp(const MjdEpoch &tt,
             attitude_details::MeasuredAttitudeData &att) const noexcept {
#ifdef DEBUG
    assert(tt >= mbuf[cj].t() && tt < mbuf[cj + 1].t());
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
   * number of different angles to be interpolated are found by the
   * passed in attitude_details::MeasuredAttitudeData instance (i.e.
   * att.num_angles).
   *
   * The computed angles are stored in the passed in att instance.
   *
   * @param[in] tt Epoch of interpolation request, TT.
   * @param[in] att A attitude_details::MeasuredAttitudeData instance. The
   * number of angles stored in here (i.e. att.num_angles) is the number of
   * angles we are going to interpolate. Hence, att.num_angles should be <= to
   * the attitude_details::MeasuredAttitudeData instances hold in buffer. Most
   * probably, the equality should hold. At output, the epoch of the instance
   * will be set to tt, and the stored angles will hold the computed values.
   * @return Always zero.
   */
  int aintrpl(const MjdEpoch &tt,
              attitude_details::MeasuredAttitudeData &att) const noexcept {
#ifdef DEBUG
    assert(tt >= mbuf[cj].t() && tt < mbuf[cj + 1].t());
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

  /** @brief Read and parse the next line of the stream.
   *
   * @param[out] data A attitude_details::MeasuredAttitudeData instance, where
   * the parsed data will be stored at.
   * @return Anything other than zero denotes an error.
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

  /** @brief Read BufferSize records (i.e. lines) from the current stream
   * position.
   *
   * Read, parse and store the BufferSize records (i.e. lines) first
   * encountered in the current stream position. Store them the instance's
   * mbuf member var.
   * If the buffer is filled (successefully), current index (i.e. cj) will be
   * set to 0.
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

    /* set the current index to 0 */
    cj = 0;
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
   * @warning To save space and time, this function does not work incrementally,
   * i.e. try to leaft shit, parse and store n times. It works in "batch" mode.
   * Hence, if e.g. n=5, and the first 4 records are correctly parsed and
   * stored, but EOF is encountered and we cannot read the 5th line, the
   * function will signal an error. This inhibits the reading of the last lines
   * in a data file.
   *
   * @param[n] Number of new records to read from the stream and store in
   * buffer. The following must hold: 0 < n <= BufferSize.
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
    int i = BufferSize - n;
    int error = 0;
    while ((i < BufferSize) && (!error)) {
      error = parse_next_record(mbuf[i]);
      ++i;
    }
    cj = BufferSize - n - 1;
#ifdef DEBUG
    assert(cj >= 0 && cj < BufferSize - 1);
#endif
    return error;
  }

  /** @brief Read data off from the stream untill we reach (and pass) `t`.
   *
   * The function will keep on reading and collecting new data off from the
   * stream, untill the instance's buffer holds an interval for which
   * `buffer[j].mtt <= t < buffer[j+1].mtt`; i.e. the epoch is availble within
   * the buffer (e.g. for interpolation).
   *
   * @warning If `t` is one of the last lines in the data file, then we may be
   * unable to read a valid interval. See the warning in collect_new_batch.
   *
   * @param[in] t The epoch of request in TT.
   * @return Anything other than 0, denotes an error (could also be EOF).
   */
  int collect_range(const MjdEpoch &t) noexcept {
    if (t <= mbuf[0].t()) {
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

  /** @brief Place the instance's index `cj` at an interval for which:
   * `buf[cj].t<=t<buf[cj+1].t`
   *
   * If needed, the function may use the stream to load new data to the buffer.
   * At success, on function output, `cj` will be conviniently placed such that:
   * `buf[cj].t()<=t<buf[cj+1].t()`
   *
   * @param[in] t The epoch of request in TT.
   * @return Anything other than 0, denotes an error (could also be EOF).
   *
   * Exit Codes:
   *  0 : Success.
   *  2 : Failed collecting next measured attitude from stream (e.g. parsing
   *      error, EOF, etc).
   * 30 : Given date is not buffered and is placed before the current stream
   *      position. We cannot go backwards in the file/stream.
   * 80 : An error that should never happen.
   */
  int hunt(const MjdEpoch &t) noexcept {
#ifdef DEBUG
    assert((cj >= 0) && (cj < BufferSize - 1));
#endif
    /* quick return */
    if ((t >= mbuf[cj].t()) && (t < mbuf[cj + 1].t())) {
      return 0;
    }

    /* No quick return! If we are requesting a latter date than the current
     * interval, there are two possibilities:
     *
     * a) the interval we are looking for exists latter in the current buffer;
     *    in this case, we are searching the buffer from this point onwards...
     */
    if (t >= mbuf[cj].t()) {
      auto it =
          std::lower_bound(mbuf.begin() + cj, mbuf.end(), t,
                           [](const attitude_details::MeasuredAttitudeData &bt,
                              const MjdEpoch &tval) { return bt.t() < tval; });
      if (it != mbuf.end()) {
        /* got it, place the index and return */
        cj = std::distance(mbuf.begin(), it) - 1;
        return 0;
      } else {
        /* b) the interval is not buffered; keep reading from the stream untill
         *    we either buffer it or reach EOF
         */
        if (collect_new_batch(2 * BufferSize / 3)) {
          fprintf(stderr,
                  "[ERROR] Failed collecting new/next data from stream "
                  "(traceback: %s)\n",
                  __func__);
          return 2;
        }
        /* return if we have the right interval, else hunt */
#ifdef DEBUG
        assert(cj < BufferSize - 1);
#endif
        return ((t >= mbuf[cj].t()) && (t < mbuf[cj + 1].t())) ? 0 : hunt(t);
      }
      /* end block: requesting latter time */
    } else {

      /* Ok, reached this pont, thus the interval is prior to the current
       * (buffered) interval. Two posibilities here:
       *
       * a) search for the interval from the begining of the buffer (to here),
       *    assuming the requested date is buffered (i.e. mbuf[0].t() < t):
       */
      if (t > mbuf[0].t()) {
        /* search for interval from the top of the buffer */
        auto it = std::lower_bound(
            mbuf.begin(), mbuf.begin() + cj + 1, t,
            [](const attitude_details::MeasuredAttitudeData &bt,
               const MjdEpoch &tval) { return bt.t() < tval; });
        assert((it < mbuf.begin() + cj + 1) && (it < mbuf.end()));
        cj = std::distance(mbuf.begin(), it) - 1;
        return 0;
      } else {
        /* b) the requested date is prior to the first date buffered! cannot
         *    go backwards in the stream, sorry ....
         */
        fprintf(stderr,
                "[ERROR] Request for quaternion which is prior to current "
                "buffered block! (traceback: %s)\n",
                __func__);
        return 30;
      }
      /* end block: requesting prior time */
    }

    /* should never reach this point! */
    fprintf(stderr,
            "[ERROR] x2 : An error that should never happen! (traceback: %s)\n",
            __func__);
    return 80;
  }

  /** @brief Helper function to create mbuf (part I).
   *
   * mbuf cannot be default initialized in the constructor, cause
   * atiitude_details::MeasuredAttitudeData does not have a default constrctor.
   * This function, helps construct such an array so that it can be used in the
   * constructor body.
   *
   * It only serves this purpose and should not be used otherwise.
   */
  template <typename T, std::size_t... Is>
  constexpr std::array<T, sizeof...(Is)>
  make_array_impl(int q, int a, std::index_sequence<Is...>) {
    return {{(static_cast<void>(Is), T(q, a))...}};
  }

  /** @brief Helper function to create mbuf (part II).
   * @see make_array_impl
   */
  template <typename T, std::size_t N>
  constexpr std::array<T, N> make_array(int q, int a) {
    return make_array_impl<T>(q, a, std::make_index_sequence<N>{});
  }

public:
  /** @brief Constructor.
   * @param[in] fn A DSO measured attitude data file (see
   * https://github.com/xanthospap/attitude)
   * @param[in] numq Number of quaternions we will need to parse/store. This
   * should normally signalled by the respective satellite's traits, e.g.
   * SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions
   * @param[in] numa Number of (rotation) angles we will need to parse/store.
   * This should normally signalled by the respective satellite's traits, e.g.
   * SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles
   * @param[in] t An epoch in TT; if set, then at instance construction, the
   * file will be searched for a suitable interval to hold `t`. If the default
   * value is  used for `t`, i.e. MjdEpoch::min(), then the buffer will just be
   * filled with the first BufferSize records.
   */
  DsoAttitudeStream(const char *fn, int numq, int numa,
                    const MjdEpoch &t = MjdEpoch::min())
      : mstream(fn), cj(-1),
        mbuf(make_array<attitude_details::MeasuredAttitudeData, BufferSize>(
            numq, numa)) {
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
   * The quaternions (if any) are interpolated using the slerp method. Angles
   * (if any) are interpolated using linear interpolation with the two
   * surrounding values.
   *
   * This is the most vital function of the class. Upon creation, one should
   * probably only need this function to navigate through the file (in a forward
   * manner) and get the attitude for any epoch of request. The function will
   * handle everything (file navigation, streaming, etc).
   *
   * @param[in] tt The epoch of request, in TT.
   * @return Anything other than 0 denotes an error. If the return value is
   * other thatn zero (i.e. an error), the error codes are returned as in the
   * hunt method.
   *
   * @see hunt
   */
  int attitude_at(const MjdEpoch &tt,
                  attitude_details::MeasuredAttitudeData &att) noexcept {
    /* lets get at the right interval (in buffer) */
    if (int error = this->hunt(tt)) {
      char buf[64];
      fprintf(stderr,
              "[ERROR] Failed getting measured attitude for epoch: %s (TT) "
              "(traceback: %s)\n",
              to_char<YMDFormat::YYYYMMDD, HMSFormat::HHMMSSF>(tt, buf),
              __func__);
      return error;
    }

    /* interpolate quaternion(s) */
    this->qslerp(tt, att);

    /* interpolate angle(s) */
    this->aintrpl(tt, att);

    return 0;
  }

  /** @brief Reload the attitude stream.
   *
   * Dump current attitude data; got to the top of the file and re-initialize
   * the instance (reading data again from the top of the file).attitude_details
   *
   * @return Anything other than 0 denotes an error.
   */
  int reload() {
    /* got to the top of the file */
    mstream.seekg(0);
    /* and reinitialize */
    return initialize();
  }

}; /* DsoAttitudeStream */

} /* namespace dso */

#endif
