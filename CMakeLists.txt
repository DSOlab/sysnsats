cmake_minimum_required(VERSION 3.8)

project(
  satmsys
  VERSION 1.1.0
  DESCRIPTION "Satellite systems and specifics"
  LANGUAGES CXX
)

# Ensure required libraries are available
find_package(Eigen3   REQUIRED)
find_package(geodesy  REQUIRED)
find_package(datetime REQUIRED)

# Pass the library dependencies to subdirectories
set(PROJECT_DEPENDENCIES Eigen3::Eigen geodesy datetime)

# Enable clang-tidy option
option(ENABLE_CLANG_TIDY "Enable clang-tidy checks" OFF)

# Define an option for building tests (defaults to ON)
option(BUILD_TESTING "Enable building of tests" ON)

# compiler flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)
add_compile_options(-Wall 
  -Wextra 
  -Werror 
  -pedantic 
  -W 
  -Wshadow 
  $<$<CONFIG:Release>:-O2>
  $<$<CONFIG:Release>:-march=native>
  $<$<CONFIG:Debug>:-g>
  $<$<CONFIG:Debug>:-pg> 
  $<$<CONFIG:Debug>:-Wdisabled-optimization>
  )
add_compile_definitions(
  $<$<CONFIG:Debug>:DEBUG>
)

# clang-tidy (targets that follow will be checked)
if(ENABLE_CLANG_TIDY)
  set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=bugprone-*,\
  clang-analyzer-*,\
  cppcoreguidelines-*,\
  llvm-*,\
  modernize-*,\
  performance-*,\
  -modernize-use-trailing-return-type,\
  -cppcoreguidelines-pro-bounds-pointer-arithmetic,\
  -cppcoreguidelines-pro-bounds-constant-array-index,\
  -cppcoreguidelines-pro-type-vararg")
  message(STATUS "clang-tidy is enabled.")
else()
  message(STATUS "clang-tidy is disabled. To enable it, use -DENABLE_CLANG_TIDY=ON.")
endif()

# the library and includes
add_library(satmsys)
target_include_directories(satmsys
PUBLIC
  # Public headers during build time
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  # Public headers during install time
  $<INSTALL_INTERFACE:include/satmsys>
PRIVATE
  # Private/internal headers only during build time (src/core)
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/core>
  # Internal/private headers during install time (from include/satmsys/core)
  $<INSTALL_INTERFACE:include/satmsys/core>
)

# library source code
add_subdirectory(src/lib)
add_subdirectory(src/doris)

# disable clang-tidy (targets that follow will not be checked)
set(CMAKE_CXX_CLANG_TIDY "")

# The tests
if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(test/unit_tests)
  enable_testing()
endif()

# install library
install(TARGETS satmsys
        EXPORT satmsysTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install public headers at: $PREFIX/satmsys/...
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
	DESTINATION include/satmsys
)

# Install private headers to a separate directory $PREFIX/satmsys/core
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/core/ 
        DESTINATION include/satmsys/core
)

install(EXPORT satmsysTargets
        FILE satmsysTargets.cmake
        NAMESPACE dso::
        DESTINATION lib/cmake
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "satmsysConfigVersion.cmake"
  VERSION ${satmsys_version}
  COMPATIBILITY AnyNewerVersion
)

install(FILES "satmsysConfig.cmake" "${CMAKE_CURRENT_BINARY_DIR}/satmsysConfigVersion.cmake"
  DESTINATION lib/cmake
)
