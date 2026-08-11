cmake_minimum_required(VERSION 3.15...3.25)
project(
  "panda_py"
  LANGUAGES CXX)
#  VERSION "${SKBUILD_PROJECT_VERSION}")

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Franka REQUIRED)

# Derive the preprocessor version guard from the libfranka package that was
# actually found, so it can never disagree with the headers we compile against.
# It can still be overridden explicitly, e.g. -DLIBFRANKA_VER=0x000902.
if(NOT DEFINED LIBFRANKA_VER)
  if(NOT Franka_VERSION)
    message(FATAL_ERROR
      "find_package(Franka) reported no version. Pass the version explicitly, "
      "e.g. -DLIBFRANKA_VER=0x000902 for libfranka 0.9.2.")
  endif()
  string(REPLACE "." ";" _franka_version_parts "${Franka_VERSION}")
  list(GET _franka_version_parts 0 _franka_major)
  list(GET _franka_version_parts 1 _franka_minor)
  list(GET _franka_version_parts 2 _franka_patch)
  math(EXPR LIBFRANKA_VER
       "(${_franka_major} << 16) + (${_franka_minor} << 8) + ${_franka_patch}"
       OUTPUT_FORMAT HEXADECIMAL)
  message(STATUS
    "Derived LIBFRANKA_VER=${LIBFRANKA_VER} from libfranka ${Franka_VERSION}")

  # libfranka does not always bump the version in its CMakeLists: the 0.14.2 tag
  # still declares 0.14.1, for instance. That is harmless while both fall inside
  # the same version guard, but a mismatch across a minor version would select
  # the wrong bindings, so say so loudly.
  if(DEFINED ENV{LIBFRANKA_VER})
    if(NOT "$ENV{LIBFRANKA_VER}" VERSION_EQUAL "${Franka_VERSION}")
      if(NOT "$ENV{LIBFRANKA_VER}" MATCHES
         "^${_franka_major}\\.${_franka_minor}\\.")
        message(WARNING
          "libfranka reports version ${Franka_VERSION} but $ENV{LIBFRANKA_VER} "
          "was requested. These differ in the minor version, so the version "
          "guards may select bindings that do not match the headers. Pass "
          "-DLIBFRANKA_VER explicitly if the reported version is wrong.")
      else()
        message(STATUS
          "Note: libfranka reports ${Franka_VERSION} for requested "
          "$ENV{LIBFRANKA_VER}; same minor version, guards unaffected.")
      endif()
    endif()
  endif()
endif()
add_compile_definitions(LIBFRANKA_VER=${LIBFRANKA_VER})
find_package(
  Python
  COMPONENTS Interpreter Development.Module
  REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

## _core module
pybind11_add_module(_core
  src/_core.cpp
  src/panda.cpp
  src/controllers/joint_limits/virtual_wall.cpp
  src/controllers/integrated_velocity.cpp
  src/controllers/joint_position.cpp
  src/controllers/cartesian_impedance.cpp
  src/controllers/applied_torque.cpp
  src/controllers/applied_force.cpp
  src/controllers/force.cpp
  src/controllers/joint_trajectory.cpp
  src/controllers/cartesian_trajectory.cpp
  src/motion/generators.cpp
  src/motion/time_optimal/trajectory.cpp
  src/motion/time_optimal/path.cpp
)

target_link_libraries(_core PUBLIC
  Threads::Threads
  ${Franka_LIBRARIES}
)

target_include_directories(_core SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

#target_compile_definitions(_core
#  PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS _core LIBRARY DESTINATION panda_py)

## libfranka module
pybind11_add_module(libfranka
  src/libfranka.cpp)

target_link_libraries(libfranka PUBLIC
  ${Franka_LIBRARIES}
)

target_include_directories(libfranka SYSTEM PUBLIC
  include
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#target_compile_definitions(libfranka
#  PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS libfranka LIBRARY DESTINATION panda_py)
