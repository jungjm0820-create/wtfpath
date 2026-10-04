cmake_minimum_required(VERSION 3.18)
project(motion_core_cpp)

set(CMAKE_CXX_STANDARD 17)

find_package(Python COMPONENTS Interpreter Development REQUIRED)

# pybind11: submodule or pip-installed; supports both
#   git submodule add https://github.com/pybind/pybind11 third_party/pybind11
#   or: pip install pybind11, then -Dpybind11_DIR=$(python -m pybind11 --cmakedir)
find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
  add_subdirectory(third_party/pybind11)
endif()

pybind11_add_module(motion_core_cpp src/motion_core.cpp)

# Output next to the addon for Blender to import:
#   copy motion_core_cpp*.pyd/.so into motion_sculpt/
install(TARGETS motion_core_cpp DESTINATION .)
