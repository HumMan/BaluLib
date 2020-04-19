cmake_minimum_required(VERSION 2.8)

project(BaluLib)

file(GLOB_RECURSE ${PROJECT_NAME}_SRC
    "Source/*.h"
    "Source/*.cpp"
)

include_directories("Include")

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

add_definitions(-DBALULIB_DLL_EXPORT)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC})
