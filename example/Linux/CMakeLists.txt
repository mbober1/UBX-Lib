cmake_minimum_required(VERSION 3.16)

project(UBX-Linux-example LANGUAGES CXX C)


set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)

add_executable(${PROJECT_NAME} main.cpp ../../src/ubx_lib.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ../../inc)