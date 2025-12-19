#ifndef __DSO_SATELLITE_SYSTEM_DORIS_HPP__
#define __DSO_SATELLITE_SYSTEM_DORIS_HPP__

#include "satellite.hpp"
#include <limits>

namespace dso {
template <> struct SatelliteSystemTraits<SATELLITE_SYSTEM::DORIS> {
  /* In DORIS RINEX files, the receiver clock offset may be missing for
   * some/all epochs; this value signifies a missing epoch Receiver clock
   * offset value.
   */
  static constexpr double RECEIVER_CLOCK_OFFSET_MISSING =
      std::numeric_limits<double>::min();

  /* In DORIS RINEX files, the observation value may be missing for some/all
   * epochs; this value signifies a missing observation value.
   */
  static constexpr double OBSERVATION_VALUE_MISSING =
      std::numeric_limits<double>::min();

  /* @brief the 2 GHz fundamental DORIS frequency (aka S1) in [MHz]. */
  static constexpr double DORIS_FREQ1_MHZ = 2.036250e3;

  /* @brief the 400 MHz fundamental DORIS frequency (aka U2) in [MHz]. */
  static constexpr double DORIS_FREQ2_MHZ = 401.250e0;

  /* @brief the (freq1 / freq2) factor (normally used for iono-free l.
   * combination).
   */
  static constexpr double GAMMA_FACTOR_SQRT = DORIS_FREQ1_MHZ / DORIS_FREQ2_MHZ;

  /* @brief the (freq1 / freq2)^2 factor (normally used for iono-free l.
   * combination).
   */
  static constexpr double GAMMA_FACTOR = GAMMA_FACTOR_SQRT * GAMMA_FACTOR_SQRT;

  /* @brief F0, aka USO frequency in [Hz]. */
  static constexpr double USO_F0 = 5e6;

  /* @brief Compute the S1 and U2 (aka 2GHz and 400 MHz) nominal frequencies
   *       for a DORIS beacon.
   *
   * @param[in] shift_factor The beacon's shift factor (e.g. as extracted from
   *        the 'STATION REFERENCE' field from a DORIS RINEX file)
   * @param[out] s1_freq The S1 (aka 2GHz) nominal frequency [Hz]
   * @param[out] u2_freq The U2 (aka 400MHz) nominal frequency in [Hz]
   */
  static void beacon_nominal_frequency(int shift_factor, double &s1_freq,
                                       double &u2_freq) noexcept {
    const long two26 = std::pow(2, 26);
    constexpr double fac1 = USO_F0 * 0.75e0;
    const double fac2 =
        (USO_F0 * (87e0 * shift_factor)) / (5e0 * static_cast<double>(two26));
    s1_freq = 543e0 * fac1 + 543e0 * fac2;
    u2_freq = 107e0 * fac1 + 107e0 * fac2;
  }

}; /* class SatelliteSystemTraits<SATELLITE_SYSTEM::DORIS> */

template <> struct SatelliteSystemObservationType<SATELLITE_SYSTEM::DORIS> {
  /* DORIS Observation Types as defined in RINEX DORIS 3.0 (Issue 1.7) */
  enum class ObservationType : int_fast8_t {
    /* L in [cycles] */
    phase,
    /* C in [m] */
    pseudorange,
    /* W power level received at each frequency, in [dBm] */
    power_level,
    /* F relative frequency offset of the receiver’s  oscillator
     * (f-f0) / f0, unit 10e-11
     */
    frequency_offset,
    /* P ground pressure at the station, unit [100 Pa (mBar)] */
    ground_pressure,
    /* T ground temperature at the station, in [Celsius] */
    ground_temperature,
    /* H ground humidity at the station, in [%] */
    ground_humidity,
  }; /* ObservationType */

  /* @brief Translate an ObservationType to a character.
   *
   * @param[in] obs An ObservationType to translate to char.
   * @throw std::runtime_error if no corresponding char is found for the given
   *        ObservationType.
   */
  static char obst2char(ObservationType obs);

  /*  @brief Translate a character to an ObservationType.
   *
   *  @param[in] c A char to translate to an ObservationType
   *  @throw std::runtime_error if no corresponding ObservationType is found to
   *         match the given character.
   */
  static ObservationType char2obst(char c);
}; /* class SatelliteSystemObservationType<SATELLITE_SYSTEM::DORIS> */

} /* namespace dso */

#endif
