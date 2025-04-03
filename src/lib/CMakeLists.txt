target_sources(sysnsats
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cnes_satellite_correction.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/csatid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/conical_occultation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/parse_dso_attitude_line.cpp
)
