cmake_minimum_required(VERSION 3.8)
project(datmo)

# Default to C99.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require that dependencies from package.xml be available.
find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
#MultiThreading Library
OPTION (USE_OpenMP "Use OpenMP" OFF)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
ENDIF()
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/datmo.cpp
  src/cluster.cpp
  src/l_shape_tracker.cpp
  src/kalman.cpp
)
# Create this project as an executable.
ament_auto_add_executable(${PROJECT_NAME}_node_exe
  ${${PROJECT_NAME}_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
