#include "datetime/calendar.hpp"
#include "satellite_core.hpp"
#include <charconv>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace {
const char *skipws(const char *strin) noexcept {
  const char *str = strin;
  while (*str && ((*str == ' ') || (*str == '+')))
    ++str;
  return str;
}
} /* anonymous namespace */

int dso::cnes_satellite_correction(const char *satmass_fn, MjdEpoch &t,
                                   double &dmass,
                                   Eigen::Matrix<double, 3, 1> &dxyz) noexcept {
  std::ifstream fin(satmass_fn);
  if (!fin.is_open()) {
    fprintf(
        stderr,
        "[ERROR] Failed openning CNES satellite info file %s (traceback: %s)\n",
        satmass_fn, __func__);
    return 1;
  }

  /* max line length in chars */
  constexpr const int SZ = 128;

  char line[SZ];
  int error = 0;
  int cjd = 0;
  double data[5];
  const char *str;
  dso::MjdEpoch tp = dso::MjdEpoch::max();
  dso::MjdEpoch tn = dso::MjdEpoch::max();
  dxyz(0) = dxyz(1) = dxyz(2) = 0e0;
  dmass = 0e0;

  // JD   Sec         Dmass       Dx       Dy       Dz
  while (fin.getline(line, SZ) && (!error)) {
    if (!(line[0] == 'C' || line[0] == '/')) {
      str = line;
      const auto sz = std::strlen(str);

      /* resolve CNES JDay (int) */
      auto res = std::from_chars(skipws(str), line + sz, cjd);
      error += (res.ec != std::errc{}) * 100;
      str = res.ptr;

      /* resolve seconds of day */
      res = std::from_chars(skipws(str + 1), line + sz, data[0]);
      error += (res.ec != std::errc{}) * 100;
      str = res.ptr;

      /* set current date as tn */
      tn = dso::MjdEpoch::from_cnes_jd((double)cjd)
               .add_seconds(dso::FractionalSeconds(data[0]));
      // tn.add_seconds(dso::FractionalSeconds(data[0]));

      /* stop if we are on the right interval */
      if ((!error) && (t >= tp && t < tn)) {
        break;
      }
      /* if the given date is before the first record, we should stop */
      if ((!error) && ((tp == dso::MjdEpoch::max()) && (t < tn))) {
        break;
      }

      /* reset current date for next iteration */
      tp = tn;

      /* resolve dMass and dOffset */
      for (int i = 0; i < 4; i++) {
        res = std::from_chars(skipws(str + 1), line + sz, data[i + 1]);
        error += (res.ec != std::errc{});
        str = res.ptr;
      }

    } /* data line */
  } /* end looping file */

  if (error || ((!fin.good()) && (!fin.eof()))) {
    fprintf(stderr,
            "[ERROR] Failed parsing CNES satellite info file %s (traceback: "
            "%s)\n[ERROR] Last line read was: %s [error=%d](traceback: %s)\n",
            satmass_fn, __func__, line, error, __func__);
    return 1;
  }

  /* all good; set return values */
  dmass = data[1];
  dxyz(0) = data[2];
  dxyz(1) = data[3];
  dxyz(2) = data[4];

  return 0;
}
