cmake_minimum_required(VERSION 3.16)

# Use the OpenVINO package configured at the project root.

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception openvino::runtime)
