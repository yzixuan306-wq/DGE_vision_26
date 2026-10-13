cmake_minimum_required(VERSION 3.16)

# Pure math utility, available without camera, serial, or inference dependencies.
add_library(tools_ballistics STATIC ballistics.cpp)
target_include_directories(tools_ballistics PUBLIC "${PROJECT_SOURCE_DIR}")
target_compile_features(tools_ballistics PUBLIC cxx_std_17)

if(SP_HERO_CORE_ONLY)
    return()
endif()

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)
target_link_libraries(tools PUBLIC tools_ballistics)
