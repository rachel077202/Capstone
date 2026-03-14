cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard_no_gps)

add_executable(offboard_no_gps
    offboard_no_gps.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(offboard_no_gps
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(offboard_no_gps PRIVATE -Wall -Wextra)
else()
    add_compile_options(offboard_no_gps PRIVATE -W2)
endif()

if(MINGW OR MSVC)
    # For M_PI
    target_compile_definitions(offboard_no_gps PRIVATE "-D_USE_MATH_DEFINES")
endif()