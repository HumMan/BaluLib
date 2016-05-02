cmake_minimum_required(VERSION 2.8)

project(BaluLib)

file(GLOB_RECURSE BaluLib_SRC "Source"
    "*.h"
    "*.cpp"
)

INCLUDE_DIRECTORIES("Include")

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

add_subdirectory(Example)

add_library(BaluLib ${BaluLib_SRC})