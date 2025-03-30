#include "satellites/satellites_core.hpp"
#include <cstdio>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

constexpr const double TOLERANCE = 1e-12;

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    fprintf(stderr, "Usage: %s [ja3mass.txt]\n", argv[0]);
    return 1;
  }

  double dmass;
  Eigen::Matrix<double, 3, 1> dxyz;

  {
    dso::MjdEpoch t(dso::year(2016), dso::month(1), dso::day_of_month(19), 56255.0);
    if (dso::cnes_satellite_correction(argv[1], t, dmass, dxyz))
    {
      fprintf(stderr, "ERROR. Failed\n");
      return 1;
    }
    printf("%.12e %.12e %.12e %.12e\n", dmass, dxyz(0), dxyz(1), dxyz(2));
    assert(std::abs(dmass - 0e0) < TOLERANCE);
    assert(std::abs(dxyz(0) - 0e0) < TOLERANCE);
    assert(std::abs(dxyz(1) - 0e0) < TOLERANCE);
    assert(std::abs(dxyz(2) - 0e0) < TOLERANCE);
  }

  {
    dso::MjdEpoch t(dso::year(2016), dso::month(1), dso::day_of_month(19), 56256.0);
    if (dso::cnes_satellite_correction(argv[1], t, dmass, dxyz))
    {
      fprintf(stderr, "ERROR. Failed\n");
      return 1;
    }
    assert(dmass == -0.273);
    assert(dxyz(0) == 0e0);
    assert(dxyz(1) == 0e0);
    assert(dxyz(2) == 0e0);
  }

  {
    dso::MjdEpoch t(dso::year(2016), dso::month(2), dso::day_of_month(19), 24597.000e0);
    if (dso::cnes_satellite_correction(argv[1], t, dmass, dxyz))
    {
      fprintf(stderr, "ERROR. Failed\n");
      return 1;
    }
    assert(dmass == -5.329);
    assert(dxyz(0) == 7e-3);
    assert(dxyz(1) == 0e0);
    assert(dxyz(2) == 0e0);
  }

  {
    dso::MjdEpoch t(dso::year(2016), dso::month(2), dso::day_of_month(19), 24599.000e0);
    if (dso::cnes_satellite_correction(argv[1], t, dmass, dxyz))
    {
      fprintf(stderr, "ERROR. Failed\n");
      return 1;
    }
    assert(dmass == -5.298);
    assert(dxyz(0) == 7e-3);
    assert(dxyz(1) == 0e0);
    assert(dxyz(2) == 0e0);
  }

  {
    dso::MjdEpoch t(dso::year(2024), dso::month(10), dso::day_of_month(27), 22706.000e0);
    if (dso::cnes_satellite_correction(argv[1], t, dmass, dxyz))
    {
      fprintf(stderr, "ERROR. Failed\n");
      return 1;
    }
    assert(dmass == -9.895);
    assert(dxyz(0) == 13e-3);
    assert(dxyz(1) == 0e0);
    assert(dxyz(2) == 0e0);
  }

  {
    dso::MjdEpoch t(dso::year(2024), dso::month(10), dso::day_of_month(27), 22708.000e0);
    if (dso::cnes_satellite_correction(argv[1], t, dmass, dxyz))
    {
      fprintf(stderr, "ERROR. Failed\n");
      return 1;
    }
    assert(dmass == -9.915);
    assert(dxyz(0) == 13e-3);
    assert(dxyz(1) == 0e0);
    assert(dxyz(2) == 0e0);
  }

  {
    dso::MjdEpoch t(dso::year(2025), dso::month(3), dso::day_of_month(16), 22417.000e0);
    if (dso::cnes_satellite_correction(argv[1], t, dmass, dxyz))
    {
      fprintf(stderr, "ERROR. Failed\n");
      return 1;
    }
    assert(dmass == -13.156);
    assert(dxyz(0) == 17e-3);
    assert(dxyz(1) == 0e0);
    assert(dxyz(2) == 0e0);
  }

  {
    dso::MjdEpoch t(dso::year(2025), dso::month(3), dso::day_of_month(16), 22419.000e0);
    if (dso::cnes_satellite_correction(argv[1], t, dmass, dxyz))
    {
      fprintf(stderr, "ERROR. Failed\n");
      return 1;
    }
    assert(dmass == -13.145);
    assert(dxyz(0) == 17e-3);
    assert(dxyz(1) == 0e0);
    assert(dxyz(2) == 0e0);
  }

  return 0;
}
