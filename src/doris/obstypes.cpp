#include "systems/doris.hpp"
#include <stdexcept>

char dso::SatelliteSystemObservationType<dso::SATELLITE_SYSTEM::DORIS>::
    obst2char(dso::SatelliteSystemObservationType<
              dso::SATELLITE_SYSTEM::DORIS>::ObservationType o)
{
  using ObsType = dso::SatelliteSystemObservationType<
      dso::SATELLITE_SYSTEM::DORIS>::ObservationType;
  switch (o)
  {
  case (ObsType::phase):
    return 'L';
  case (ObsType::pseudorange):
    return 'C';
  case (ObsType::power_level):
    return 'W';
  case (ObsType::frequency_offset):
    return 'F';
  case (ObsType::ground_pressure):
    return 'P';
  case (ObsType::ground_temperature):
    return 'T';
  case (ObsType::ground_humidity):
    return 'H';
  default:
    throw std::runtime_error(
        "[ERROR] Cannot translate ObservationType to char");
  }
}

dso::SatelliteSystemObservationType<
    dso::SATELLITE_SYSTEM::DORIS>::ObservationType
dso::SatelliteSystemObservationType<dso::SATELLITE_SYSTEM::DORIS>::char2obst(
    char c)
{
  using ObsType = dso::SatelliteSystemObservationType<
      dso::SATELLITE_SYSTEM::DORIS>::ObservationType;
  switch (c)
  {
  case ('L'):
    return ObsType::phase;
  case ('C'):
    return ObsType::pseudorange;
  case ('W'):
    return ObsType::power_level;
  case ('F'):
    return ObsType::frequency_offset;
  case ('P'):
    return ObsType::ground_pressure;
  case ('T'):
    return ObsType::ground_temperature;
  case ('H'):
    return ObsType::ground_humidity;
  default:
    throw std::runtime_error(
        "[ERROR] Cannot translate char to ObservationType");
  }
}
