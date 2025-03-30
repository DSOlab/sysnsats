#ifndef __DSO_SPACE_GEODESY_SATELLITES_CORE__HPP__
#define __DSO_SPACE_GEODESY_SATELLITES_CORE__HPP__

#include "datetime/calendar.hpp"
#include "eigen3/Eigen/Eigen"
#include <stdexcept>

namespace dso
{

  enum class SATELLITE_SYSTEM
  {
    DORIS,
    SLR,
    GPS
  }; /* enum SATELLITE_SYSTEM */

  template <SATELLITE_SYSTEM>
  struct SatelliteSystemTraits
  {
  };

  template <SATELLITE_SYSTEM>
  struct SatelliteSystemObservationType
  {
  };

  enum class SATELLITE
  {
    JASON1,
    JASON2,
    JASON3,
    SENTINEL3A,
    SENTINEL3B,
    SENTINEL6A
  }; /* enum SATELLITE */

  /** @brief Trnaslate a 3-char satellite identifier string to a SATELLITE.
   *
   * The translation follows the standard naming conventions that can be found
   * at CDDIS: https://cddis.nasa.gov/Data_and_Derived_Products/DORIS/doris_idsorbit.html
   *
   * @paramp[in] satid The satellite id. Does not have to be null-terminated,
   *                   and can have any number of leading whitespaces. The
   *                   function will discard leading whitespaces, and check the
   *                   3 characters that follow. These are considered as the
   *                   input string.
   * @return The corresponding SATELLITE.
   */
  SATELLITE translate_satid(const char *satid);

  namespace satellite_details
  {
    /* empty base class to assist inheritance. */
    class BaseMacromodel
    {
    }; /* BaseMacromodel */
  } /* namespace satellite_details */

  template <SATELLITE S>
  class SatelliteMacromodel : public satellite_details::BaseMacromodel
  {
  };

  /** @brief Get satellite mass and CoG correction from a CNS mass file.
   *
   * These files are available at:
   * ftp://ftp.ids-doris.org/pub/ids/satellites/
   * as <SATID>mass.txt
   *
   * The file mass.readme contains information on the file format (under the
   * same ftp site).
   *
   * Given a date, this function will parse the satellite-specific file and
   * return the (commulative) correction that has to be applied at the initial
   * Mass and CoG to get the 'correct' mass and CoG (in the body-fixed ref.
   * frame) of the satellite at the epoch of request.
   *
   * @param[in] satmass_fn The filename of the satellite mass file.
   * @param[in] t          The epoch for which we want the corrections.
   * @param[out] dmass     (Commulative) mass correction, in [kg] to be applied
   *                       to the initial mass value.
   * @param[out] dxyz      (Commulative) offset correction, in [m] to be
   *                       applied to the initial CoG point, in body-fixed RF.
   * @return Anything other than zero denotes an error.
   */
  int cnes_satellite_correction(const char *satmass_fn, MjdEpoch &t,
                                double &dmass,
                                Eigen::Matrix<double, 3, 1> &dxyz) noexcept;

} /* namespace dso */
#endif
