add_compile_options(-Wno-unused-but-set-variable)
add_compile_options(-Wno-unused-variable)

add_executable(test_parsing_mass_files test_parsing_mass_files.cpp)
target_link_libraries(test_parsing_mass_files PRIVATE satmsys ${PROJECT_DEPENDENCIES})
add_test(NAME parse_cnes_mass COMMAND test_parsing_mass_files
  ${CMAKE_SOURCE_DIR}/test/data/ja3mass.txt
)

add_executable(test_jason3 test_jason3.cpp)
target_link_libraries(test_jason3 PRIVATE satmsys ${PROJECT_DEPENDENCIES})
add_test(NAME info_jason3 COMMAND test_jason3)

add_executable(test_jason2 test_jason2.cpp)
target_link_libraries(test_jason2 PRIVATE satmsys ${PROJECT_DEPENDENCIES})
add_test(NAME info_jason2 COMMAND test_jason2)

add_executable(test_jason1 test_jason1.cpp)
target_link_libraries(test_jason1 PRIVATE satmsys ${PROJECT_DEPENDENCIES})
add_test(NAME info_jason1 COMMAND test_jason1)
