#include "satellites/satellites_core.hpp"
#include <stdexcept>
#include <string>

dso::SATELLITE dso::translate_satid(const char *satid)
{
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
  // else if (!std::strncmp(satid, "cs2", 3)) return SATELLITE::;
  // else if (!std::strncmp(satid, "en1", 3)) return SATELLITE::;
  // else if (!std::strncmp(satid, "h2a", 3)) return SATELLITE::;
  // else if (!std::strncmp(satid, "sp2", 3)) return SATELLITE::;
  // else if (!std::strncmp(satid, "sp3", 3)) return SATELLITE::;
  // else if (!std::strncmp(satid, "sp4", 3)) return SATELLITE::;
  // else if (!std::strncmp(satid, "sp5", 3)) return SATELLITE::;
  // else if (!std::strncmp(satid, "srl", 3)) return SATELLITE::;
  // else if (!std::strncmp(satid, "top", 3)) return SATELLITE::;

  /* should never reach this point ... */
  throw std::runtime_error("[ERROR] Failed to translate satellite id " + std::string(satid) + " (traceback: " + std::string(__func__) + ")\n");
}
