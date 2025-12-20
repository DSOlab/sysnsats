#ifndef __DSO_SATELLITE_ATTITUDE_GEN_HPP__
#define __DSO_SATELLITE_ATTITUDE_GEN_HPP__

/* @file
 *
 * This file is includes top-level definitions for handling satellite attitude.
 * We use inheritance/polymorphism here, to take into account different
 * possibilities:
 *
 * - MeasuredAttitude which is based on DsoAttitudeStream; this type of attitude
 * represents measured attitude (i.e. quaternions and/or rotation angles) that
 * are extracted from a respective DSO data file.
 * - PhaseLawAttitude which (in the future) will handle computing attitude based
 * on a phase law.
 * - NoAttitude when no attitude information is available.
 *
 * The base class (for all the above) is SatelliteAttitude, so that we can:
 * @code
 * SatelliteAttitude *att;
 * // qua_ja3.csv is a DSO attitude file
 * att = new MeasuredAttitude(SATELLITE::JASON3, "qua_ja3.csv");
 * // placeholder for attitude data, Jason-3
 * attitude_details::MeasuredAttitudeData data(
 *    SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions,
 *    SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles);
 * // request attitude data for some date
 * att->attitude_at(t_inTT, data);
 * @endcode
 */

#include "attitude_stream.hpp"
#include "satellite.hpp"

namespace dso {
namespace satellite_details {
/* Number of measured attitude records (whatever that may be, e.g. 1 quaternion
 * and 1 angle, or 1 quaternin and 2 angles, ...) that shall be buffered at any
 * given time. */
constexpr const int MeasuredAttitudeBufferSize = 15;
} /* namespace satellite_details */

/** @brief Create a MeasuredAttitudeData instance for a given satellite. */
attitude_details::MeasuredAttitudeData
measured_attitude_data_factory(SATELLITE sat);

/** @brief Base class, does nothing usefull! */
class SatelliteAttitudeModel {
  /** the satellite */
  SATELLITE msat;

public:
  /** @brief Constructor */
  SatelliteAttitudeModel(SATELLITE s) noexcept : msat(s) {};
  /** @brief Must define a virtual destructor. */
  virtual ~SatelliteAttitudeModel() noexcept = default;
  SATELLITE satellite() const noexcept { return msat; }

  virtual attitude_details::MeasuredAttitudeData *
  attitude_at(const MjdEpoch &) noexcept = 0;
  virtual const Eigen::Quaterniond *measured_quaternions() const noexcept {
    return nullptr;
  };
  virtual const double *measured_angles() const noexcept { return nullptr; };
  virtual int reload() = 0;
}; /* SatelliteAttitude */

/** @brief MeasuredAttitude, for satellites with (measured) attitude data.
 *
 * This class implies :
 * 1. a satellite macromodel,
 * 2. a data stream to get measured attitude data from
 * 3. a MeasuredAttitudeData instance to assist data extraction
 *
 * Measured attitude is provided by data files in DSO format, that can hold
 * a number of quaternions and/or angles for specific epochs. The number of
 * quaternions and angles needed to define the attitude is variable for each
 * satellite.
 *
 * i.e. a number of quaternions and/or angles. */
class MeasuredAttitude final : public SatelliteAttitudeModel {
  /* alias for attitude data stream */
  using BType =
      DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>;
  /** Measured attitude stream */
  BType matt;
  /** An instance of MeasuredAttitudeData for the given satellite */
  attitude_details::MeasuredAttitudeData mdata;

  /** @brief Factory; create and return a Measured attitude stream.
   *
   * This function is normally used by the (MeasuredAttitude) constructor to
   * create a MeasuredAttitude instance's matt member variable.
   *
   * @param[in] sat The satellite (as SATELLITE enum)
   * @param[in] fn  The measured attitude data filename. Should be in DSO
   * format and depending on the satellite can have a variable number of
   * quaternions and/or rotation angles. Time-stamps in the file are expected in
   * [TT] timescale.
   * @param[in] t   Position the stream in this instant (optional).
   */
  BType static factory(SATELLITE sat, const char *fn,
                       const MjdEpoch &t = MjdEpoch::min()) {
    switch (sat) {
    case (SATELLITE::JASON1):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON1>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON1>::NumAngles, t);

    case (SATELLITE::JASON2):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON2>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON2>::NumAngles, t);

    case (SATELLITE::JASON3):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::JASON3>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::JASON3>::NumAngles, t);

    case (SATELLITE::SENTINEL3A):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3A>::NumAngles, t);

    case (SATELLITE::SENTINEL3B):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL3B>::NumAngles, t);

    case (SATELLITE::SENTINEL6A):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SENTINEL6A>::NumAngles, t);

    case (SATELLITE::SWOT):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::SWOT>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::SWOT>::NumAngles, t);

    case (SATELLITE::CRYOSAT2):
      return DsoAttitudeStream<satellite_details::MeasuredAttitudeBufferSize>(
          fn, SatelliteAttitudeTraits<SATELLITE::CRYOSAT2>::NumQuaternions,
          SatelliteAttitudeTraits<SATELLITE::CRYOSAT2>::NumAngles, t);
    default:
      throw std::runtime_error(
          "[ERROR] Failed to construct MeasuredAttitude\n");
    }
  }

public:
  /** @brief Constructor.
   *
   * @param[in] sat The satellite.
   * @param[in] fn  The name of the input file to stream measure attitude
   * information from. These files are preprocessed by DSO and are expected to
   * have a certain format.
   */
  MeasuredAttitude(SATELLITE sat, const char *fn,
                   const MjdEpoch &t = MjdEpoch::min())
      : SatelliteAttitudeModel(sat),
        matt(MeasuredAttitude::factory(sat, fn, t)),
        mdata(measured_attitude_data_factory(sat)) {}

  /** @brief get attitude data and store it in instance's mdata member.*/
  attitude_details::MeasuredAttitudeData *
  attitude_at(const MjdEpoch &t) noexcept override {
    int error = matt.attitude_at(t, mdata);
    if (error)
      return nullptr;
    return &mdata;
  }

  /** @brief Return a pointer to measured quaternions (stored in mdata). */
  const Eigen::Quaterniond *measured_quaternions() const noexcept override {
    return mdata.quaternions();
  }

  /** @brief Return a pointer to measured angles (stored in mdata). */
  const double *measured_angles() const noexcept override { return mdata.angles(); }

  /** @brief Reload the stream, restart from top of file. */
  int reload() override { return matt.reload(); }
}; /* MeasuredAttitude */

/** @brief Phase-law attitude (not measured). */
class PhaseLawAttitude final : public SatelliteAttitudeModel {
public:
  PhaseLawAttitude(SATELLITE sat) : SatelliteAttitudeModel(sat) {};

  /** @brief No-op */
  attitude_details::MeasuredAttitudeData *
  attitude_at(const MjdEpoch &) noexcept override {
    return nullptr;
  }

  /** @brief No-op */
  int reload() override { return 0; }
}; /* PhaseLawAttitude */

/** @brief No attitude at all. */
class NoAttitude final : public SatelliteAttitudeModel {
public:
  NoAttitude(SATELLITE sat) : SatelliteAttitudeModel(sat) {};
  /** @brief No-op */
  attitude_details::MeasuredAttitudeData *
  attitude_at(const MjdEpoch &) noexcept override {
    return nullptr;
  }
  /** @brief No-op */
  int reload() override { return 0; }
}; /* NoAttitude */

class Attitude {
  /* generic function pointer */
  using RotateFnPtr = std::vector<MacromodelSurfaceElement> (*)(
      const Eigen::Quaterniond *quaternions, const double *angles,
      const Eigen::Vector3d *vectors) noexcept;

  /* the macromodel (if-any) */
  SatelliteAttitudeModel *mmodel;
  /* pointer to a function which rotates the macromodel (should be assigned at
   * construction) */
  RotateFnPtr mrotfn;
  /* initial mass */
  double mmassinit = 0e0;
  /* mass correction */
  double mdmass = 0e0;

public:
  /* @param[in] vectors A list of 3-d vectors in the order:
  vectors[0] -> sun-to-satellite vector, cartesian, inertial, geocentric [m]
  vectors[1] -> satellite position, cartesian, inertial, geocentric, [m]
  vectors[2] -> satellite velocity, cartesian, inertial, geocentric, [m]
  */
  std::vector<MacromodelSurfaceElement>
  rotated_macromodel(const MjdEpoch &tt,
                     const Eigen::Vector3d *vectors = nullptr) noexcept {
    mmodel->attitude_at(tt);
    return mrotfn(mmodel->measured_quaternions(), mmodel->measured_angles(),
                  vectors);
  }

  int load_satellite_mass_correction(const char *cnes_fn,
                                     const dso::MjdEpoch &t) {
    Eigen::Vector3d dummy;
    return dso::cnes_satellite_correction(cnes_fn, t, mdmass, dummy);
  }

  double satellite_mass() const noexcept { return mmassinit + mdmass; }

  int reload() { return mmodel->reload(); }

  Attitude(SATELLITE sat, const char *fn, const MjdEpoch &t = MjdEpoch::min()) {
    switch (sat) {
      /* satellites expecting measured attitude */
    case (SATELLITE::JASON1): {
      mmodel = new MeasuredAttitude(sat, fn, t);
      mrotfn = &SatelliteMacromodelTraits<SATELLITE::JASON1>::rotate_macromodel;
      mmassinit = SatelliteMacromodelTraits<SATELLITE::JASON1>::initial_mass();
    } break;
    case (SATELLITE::JASON2): {
      mmodel = new MeasuredAttitude(sat, fn, t);
      mrotfn = &SatelliteMacromodelTraits<SATELLITE::JASON2>::rotate_macromodel;
      mmassinit = SatelliteMacromodelTraits<SATELLITE::JASON2>::initial_mass();
    } break;
    case (SATELLITE::JASON3): {
      mmodel = new MeasuredAttitude(sat, fn, t);
      mrotfn = &SatelliteMacromodelTraits<SATELLITE::JASON3>::rotate_macromodel;
      mmassinit = SatelliteMacromodelTraits<SATELLITE::JASON3>::initial_mass();
    } break;
    case (SATELLITE::SENTINEL3A): {
      mmodel = new MeasuredAttitude(sat, fn, t);
      mrotfn =
          &SatelliteMacromodelTraits<SATELLITE::SENTINEL3A>::rotate_macromodel;
      mmassinit =
          SatelliteMacromodelTraits<SATELLITE::SENTINEL3A>::initial_mass();
    } break;
    case (SATELLITE::SENTINEL3B): {
      mmodel = new MeasuredAttitude(sat, fn, t);
      mrotfn =
          &SatelliteMacromodelTraits<SATELLITE::SENTINEL3B>::rotate_macromodel;
      mmassinit =
          SatelliteMacromodelTraits<SATELLITE::SENTINEL3B>::initial_mass();
    } break;
    case (SATELLITE::SENTINEL6A): {
      mmodel = new MeasuredAttitude(sat, fn, t);
      mrotfn =
          &SatelliteMacromodelTraits<SATELLITE::SENTINEL6A>::rotate_macromodel;
      mmassinit =
          SatelliteMacromodelTraits<SATELLITE::SENTINEL6A>::initial_mass();
    } break;
    case (SATELLITE::SWOT): {
      mmodel = new MeasuredAttitude(sat, fn, t);
      mrotfn = &SatelliteMacromodelTraits<SATELLITE::SWOT>::rotate_macromodel;
      mmassinit = SatelliteMacromodelTraits<SATELLITE::SWOT>::initial_mass();
    } break;
    case (SATELLITE::CRYOSAT2): {
      mmodel = new MeasuredAttitude(sat, fn, t);
      mrotfn =
          &SatelliteMacromodelTraits<SATELLITE::CRYOSAT2>::rotate_macromodel;
      mmassinit =
          SatelliteMacromodelTraits<SATELLITE::CRYOSAT2>::initial_mass();
    } break;
      /* satellites with phase law */
    case (SATELLITE::SPOT4):
      mmodel = new PhaseLawAttitude(sat);
      mrotfn = &SatelliteMacromodelTraits<SATELLITE::SPOT4>::rotate_macromodel;
      mmassinit = SatelliteMacromodelTraits<SATELLITE::SPOT4>::initial_mass();
      break;
      /* no attitude */
    default:
      mmodel = new NoAttitude(sat);
    }
  }

}; /* class Attitude */

} /* namespace dso */

#endif
