#include "quaternion_stream.hpp"

int dso::parse_attitude_line(const char *line, int num_quaternions,
                             int num_angles, dso::MjdEpoch &tt,
                             Eigen::Quaterniond *qout, double *aout) noexcept {
  const int sz = std::strlen(line);
  const char *str = line;
  int error = 0;

  /* parse date */
  {
    int mjday;
    double secday;
    auto res = std::from_chars(attitude_details::skipws(str), line + sz, mjday);
    if (res.ec != std::errc{})
      ++error;
    str = res.ptr;
    res = std::from_chars(attitude_details::skipws(str), line + sz, secday);
    if (res.ec != std::errc{})
      ++error;
    str = res.ptr;
    /* TODO
     * we are --should be-- using a non-normalizing c'tor here, since seconds
     * are [0,86400)
     */
    tt = dso::MjdEpoch(mjday, dso::FractionalSeconds(secday));
  }

  /* read quaternions first */
  for (int q = 0; q < num_quaternions; q++) {
    double data[4];
    for (int i = 0; i < 4; i++) {
      auto res =
          std::from_chars(attitude_details::skipws(str), line + sz, data[i]);
      if (res.ec != std::errc{})
        ++error;
      str = res.ptr;
    }
    qout[q] = Eigen::Quaterniond(data[0], data[1], data[2], data[3]);
  }

  /* read angles */
  for (int a = 0; a < num_angles; a++) {
    auto res =
        std::from_chars(attitude_details::skipws(str), line + sz, aout[a]);
    if (res.ec != std::errc{})
      ++error;
    str = res.ptr;
  }

  return error;
}