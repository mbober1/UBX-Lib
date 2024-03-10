cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -pedantic -Wshadow -Wold-style-cast -Wunused -Wsign-conversion -Wuseless-cast -Wdouble-promotion -Wduplicated-cond -Wnon-virtual-dtor -Woverloaded-virtual -Wsign-conversion -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wformat=2 -g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -Wextra -pedantic -Wshadow -Wold-style-cast -Wunused -O3 -DNDEBUG ")

add_library(UBX-Lib STATIC)

target_sources(UBX-Lib
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ubx_lib.cpp
)

target_include_directories(UBX-Lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)