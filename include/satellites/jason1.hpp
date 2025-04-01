#ifndef __DSO_JASON1_MACROMODEL_HPP__
#define __DSO_JASON1_MACROMODEL_HPP__

#include "core/macromodel_surface_element.hpp"
#include "satellites/satellites_core.hpp"
#include <array>

namespace dso
{
template <>
struct SatelliteMacromodel<SATELLITE::JASON1>
    : public satellite_details::BaseMacromodel
{
    static constexpr std::array<MacromodelSurfaceElement, 8> model = {
        {{1.65e0,
          {1e0, 0e0, 0e0},
          {0.0938e0, 0.2811e0, 0.2078e0},
          {0.4250e0, 0.1780e0, -.0260}},
         {1.65e0,
          {-1e0, 0e0, 0e0},
          {0.4340e0, 0.2150e0, 0.0050e0},
          {0.4080e0, 0.1860e0, -.0120}},
         {3.e0,
          {0e0, 1e0, 0e0},
          {1.1880e0, -.0113e0, -.0113e0},
          {0.3340e0, 0.3420e0, 0.2490}},
         {3.e0,
          {0e0, -1e0, 0e0},
          {1.2002e0, -.0044e0, -.0044e0},
          {0.2740e0, 0.3690e0, 0.2970}},
         {3.1e0,
          {0e0, 0e0, 1e0},
          {0.2400e0, 0.4020e0, 0.3300e0},
          {0.2360e0, 0.3820e0, 0.3090}},
         {3.1e0,
          {0e0, 0e0, -1e0},
          {0.3180e0, 0.3700e0, 0.2670e0},
          {0.2980e0, 0.3360e0, 0.2400}},
         /* solar array */
         {9.8e0,
          {1e0, 0e0, 0e0},
          {0.1940e0, 0.0060e0, 0.9470e0},
          {0.0970e0, 0.0980e0, 0.8030e0}},
         {9.8e0,
          {-1e0, 0e0, 0e0},
          {0.0040e0, 0.2980e0, 0.6970e0},
          {0.0350e0, 0.0350e0, 0.9310e0}}}};

    std::vector<MacromodelSurfaceElement>
    rotate_macromodel(Eigen::Quaterniond &qbody,
                      const double *theta) const noexcept
    {
        std::vector<MacromodelSurfaceElement> plates;
        plates.reserve(model.size());
        for (int i = 0; i < num_plates(); i++)
        {
            Eigen::Vector3d n =
                qbody * Eigen::Map<Eigen::Vector3d>(model[i].mnormal);
        }
        for (int i = 0; i < num_solar_arrays(); i++)
        {
            Eigen::Vector3d n =
                qbody *
                (Eigen::AngleAxisd(theta[i], Eigen::Vector3d::UnitY()) *
                 Eigen::Map<Eigen::Vector3d>(model[num_plates() + i].mnormal));
        }
    }

    /* number of body-frame plates in macromodel */
    static constexpr int num_plates()
    {
        return 6;
    }

    /* number of solar array plates in macromodel */
    static constexpr int num_solar_arrays()
    {
        return 2;
    }

    /* DORIS 2GHz (S1) Phase center in satellite reference frame in [m] (xyz) */
    static Eigen::Matrix<double, 3, 1> doris_s1_pco() noexcept
    {
        Eigen::Matrix<double, 3, 1> p;
        p << 1.171e0, -0.598e0, 1.027e0;
        return p;
    }

    /* DORIS 400MHz (U2) Phase center in satellite reference frame in [m] (xyz)
     */
    static Eigen::Matrix<double, 3, 1> doris_u2_pco() noexcept
    {
        Eigen::Matrix<double, 3, 1> p;
        p << 1.171, -0.598, 0.859e0;
        return p;
    }

    /* Initial value of mass in [kg] (note value extracted from ja1mass.txt,
     * available at ftp://ftp.ids-doris.org/pub/ids/satellites/) */
    static constexpr double initial_mass()
    {
        return 489.1e0;
    }

    /* Center of gravity coordinates in satellite reference frame [m] (xyz).
     * Note: extracted from ja1mass.txt, available at
     * ftp://ftp.ids-doris.org/pub/ids/satellites/
     */
    static Eigen::Matrix<double, 3, 1> initial_cog() noexcept
    {
        Eigen::Matrix<double, 3, 1> p;
        p << 0.955e0, 0e0, 0e0;
        return p;
    }
}; /* MacroModel<SATELLITE::Jason1> */

} /* namespace dso */

#endif
