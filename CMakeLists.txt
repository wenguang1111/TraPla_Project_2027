cmake_minimum_required(VERSION 3.16)
project(TraPlaProject2027 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(fop_scalar
    src/polynomial.cpp
    src/cubic_spline.cpp
    src/collision.cpp
    src/cost.cpp
    src/frenet_planner.cpp
    src/scenario_io.cpp
)

target_include_directories(fop_scalar PUBLIC include)
target_compile_options(fop_scalar PRIVATE -Wall -Wextra -Wpedantic)

add_executable(fop_demo src/main.cpp)
target_link_libraries(fop_demo PRIVATE fop_scalar)

add_executable(fop_scenario src/scenario_main.cpp)
target_link_libraries(fop_scenario PRIVATE fop_scalar)
