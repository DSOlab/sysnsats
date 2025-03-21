#ifndef __DSO_SATELLITE_MACROMODEL_SURFACE_ELEMENT_HPP__
#define __DSO_SATELLITE_MACROMODEL_SURFACE_ELEMENT_HPP__

#include "eigen3/Eigen/Eigen"
#include <array>

namespace dso {

struct MacromodelSurfaceElement {
//public:
/* surface area in m^2 */
double marea;
/* normal vector in satellite ref. frame */
double mnormal[3];
/* optical properties in the order: spec, diff, abs */
double moptical[3];
/* infrared properties in the order: spec, diff, abs */
double minfrared[3];
  
double area() const noexcept {return marea;}
double &area() noexcept {return marea;}
// const Eigen::Matrix<double,3,1> &normal() const noexcept {return mnormal;}
// Eigen::Matrix<double,3,1> &normal() noexcept {return mnormal;}
double spec_optical() const noexcept {return moptical[0];}
double &spec_optical() noexcept {return moptical[0];}
double diff_optical() const noexcept {return moptical[1];}
double &diff_optical() noexcept {return moptical[1];}
double abs_optical() const noexcept {return moptical[2];}
double &abs_optical() noexcept {return moptical[2];}
double spec_infrared() const noexcept {return minfrared[0];}
double &spec_infrared() noexcept {return minfrared[0];}
double diff_infrared() const noexcept {return minfrared[1];}
double &diff_infrared() noexcept {return minfrared[1];}
double abs_infrared() const noexcept {return minfrared[2];}
double &abs_infrared() noexcept {return minfrared[2];}
}; /* class MacromodelSurfaceElement */

}/* namespace dso */

#endif
