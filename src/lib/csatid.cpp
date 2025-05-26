#include "satellite_core.hpp"
#include <stdexcept>
#include <string>

dso::SATELLITE dso::translate_satid(const char *satid) {
  constexpr const int MAX_CHARS_IN_STRING = 1024;
  int nc = 0;
  while (*satid && (*satid == ' ' && nc++ < MAX_CHARS_IN_STRING))
    ++satid;
  if (!std::strncmp(satid, "ja1", 3))
    return dso::SATELLITE::JASON1;
  else if (!std::strncmp(satid, "ja2", 3))
    return dso::SATELLITE::JASON2;
  else if (!std::strncmp(satid, "ja3", 3))
    return dso::SATELLITE::JASON3;
  else if (!std::strncmp(satid, "s3a", 3))
    return dso::SATELLITE::SENTINEL3A;
  else if (!std::strncmp(satid, "s3b", 3))
    return dso::SATELLITE::SENTINEL3B;
  else if (!std::strncmp(satid, "s6a", 3))
    return dso::SATELLITE::SENTINEL6A;
  /* should never reach this point ... */
  throw std::runtime_error("[ERROR] Failed to translate satellite id " +
                           std::string(satid) +
                           " (traceback: " + std::string(__func__) + ")\n");
}

dso::SATELLITE dso::translate_sp3_satid(const char *satid) {
  using dso::SATELLITE;
  constexpr const int MAX_CHARS_IN_STRING = 1024;
  int nc = 0;
  while (*satid && (*satid == ' ' && nc++ < MAX_CHARS_IN_STRING))
    ++satid;
  if (!std::strncmp("L08", satid, 3))
    return SATELLITE::JASON1;
  else if (!std::strncmp("L27", satid, 3))
    return SATELLITE::JASON2;
  else if (!std::strncmp("L39", satid, 3))
    return SATELLITE::JASON3;
  else if (!std::strncmp("L40", satid, 3))
    return SATELLITE::SENTINEL6A;
  else if (!std::strncmp("L74", satid, 3))
    return SATELLITE::SENTINEL3A;
  else if (!std::strncmp("L75", satid, 3))
    return SATELLITE::SENTINEL3B;
  else
    throw std::runtime_error("[ERROR] Failed to match Sp3 satellite id " +
                             std::string(satid, 3) +
                             " (traceback: " + std::string(__func__) + ")\n");
}
