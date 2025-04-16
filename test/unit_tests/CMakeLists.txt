add_compile_options(-Wno-unused-but-set-variable)
add_compile_options(-Wno-unused-variable)

add_executable(test_parsing_mass_files test_parsing_mass_files.cpp)
target_link_libraries(test_parsing_mass_files PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
add_test(NAME parse_cnes_mass COMMAND test_parsing_mass_files
  ${CMAKE_SOURCE_DIR}/test/data/ja3mass.txt
)

add_executable(test_jason3 test_jason3.cpp)
target_link_libraries(test_jason3 PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
add_test(NAME info_jason3 COMMAND test_jason3)

add_executable(test_jason2 test_jason2.cpp)
target_link_libraries(test_jason2 PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
add_test(NAME info_jason2 COMMAND test_jason2)

add_executable(test_jason1 test_jason1.cpp)
target_link_libraries(test_jason1 PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
add_test(NAME info_jason1 COMMAND test_jason1)

add_executable(test_satid2satellite satid2satellite.cpp)
target_link_libraries(test_satid2satellite PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
add_test(NAME satid2satellite COMMAND test_satid2satellite)

add_executable(load_satellite test_load_satellite.cpp)
target_link_libraries(load_satellite PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
find_file(JA3Q
  NAMES qua_ja3.csv
  PATHS ${CMAKE_SOURCE_DIR}/data
  NO_DEFAULT_PATH
  REQUIRED
)
add_test(NAME load_jason3 COMMAND load_satellite "ja3" ${JA3Q})

add_executable(ja3_measured_attitude test_jason3_measured_attitude.cpp)
target_link_libraries(ja3_measured_attitude PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
find_file(JA3Q
  NAMES qua_ja3.csv
  PATHS ${CMAKE_SOURCE_DIR}/data
  NO_DEFAULT_PATH
  REQUIRED
)
add_test(NAME ja3_measured_attitude COMMAND ja3_measured_attitude ${JA3Q})

add_executable(ja2_measured_attitude test_jason2_measured_attitude.cpp)
target_link_libraries(ja2_measured_attitude PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
find_file(JA2Q
  NAMES qua_ja2.csv
  PATHS ${CMAKE_SOURCE_DIR}/data
  NO_DEFAULT_PATH
  REQUIRED
)
add_test(NAME ja2_measured_attitude COMMAND ja2_measured_attitude ${JA2Q})

add_executable(ja1_measured_attitude test_jason1_measured_attitude.cpp)
target_link_libraries(ja1_measured_attitude PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
find_file(JA1Q
  NAMES qua_ja1.csv
  PATHS ${CMAKE_SOURCE_DIR}/data
  NO_DEFAULT_PATH
  REQUIRED
)
add_test(NAME ja1_measured_attitude COMMAND ja1_measured_attitude ${JA1Q})

add_executable(s3a_measured_attitude test_sentinel3a_measured_attitude.cpp)
target_link_libraries(s3a_measured_attitude PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
find_file(S3AQ
  NAMES qua_s3a.csv
  PATHS ${CMAKE_SOURCE_DIR}/data
  NO_DEFAULT_PATH
  REQUIRED
)
add_test(NAME s3a_measured_attitude COMMAND s3a_measured_attitude ${S3AQ})

add_executable(s3b_measured_attitude test_sentinel3b_measured_attitude.cpp)
target_link_libraries(s3b_measured_attitude PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
find_file(S3BQ
  NAMES qua_s3b.csv
  PATHS ${CMAKE_SOURCE_DIR}/data
  NO_DEFAULT_PATH
  REQUIRED
)
add_test(NAME s3b_measured_attitude COMMAND s3b_measured_attitude ${S3BQ})

add_executable(s6a_measured_attitude test_sentinel6a_measured_attitude.cpp)
target_link_libraries(s6a_measured_attitude PRIVATE sysnsats ${PROJECT_DEPENDENCIES})
find_file(S6AQ
  NAMES qua_s6a.csv
  PATHS ${CMAKE_SOURCE_DIR}/data
  NO_DEFAULT_PATH
  REQUIRED
)
add_test(NAME s6a_measured_attitude COMMAND s6a_measured_attitude ${S6AQ})