cmake_minimum_required(VERSION 3.15)
project(motion_core_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. pybind11 패키지 탐지
find_package(pybind11 CONFIG QUIET)

# 2. 만약 찾지 못했다면 fatal error 대신 명확한 안내 출력
if(NOT pybind11_FOUND)
    message(FATAL_ERROR "pybind11을 찾을 수 없습니다. 'pip install pybind11' 후 -Dpybind11_DIR 경로를 지정했는지 확인해 주세요.")
endif()

# 3. C++ 파이썬 확장 모듈 정의
pybind11_add_module(motion_core_cpp
    src/motion_core.cpp
)

# 4. Release 빌드 최적화 옵션 (MSVC 전용)
if(MSVC)
    target_compile_options(motion_core_cpp PRIVATE /O2 /EHsc)
endif()
