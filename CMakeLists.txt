## Basic cmake settings
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "Target C++ standard, required c++17 or higher")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message("${CMAKE_BUILD_TYPE}")
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type default to Debug, set to Release if needed" FORCE)
    option(CMAKE_EXPORT_COMPILE_COMMANDS "Generate compile commands" on)
endif()

project(HAND_EYE_CALIBRATION VERSION 1.0)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

option(BUILD_EYE_TO_HAND_FLAG "build eye-to-hand module" off)
if(BUILD_EYE_TO_HAND_FLAG)
    add_subdirectory(eye_to_hand)
endif()

option(BUILD_EYE_IN_HAND_FLAG "build eye-in-hand module" off)
if(BUILD_EYE_IN_HAND_FLAG)
    add_subdirectory(eye_in_hand)
endif()

find_package(Eigen3 REQUIRED)

# message("current source: "${CMAKE_CURRENT_SOURCE_DIR})
# file(GLOB COMMON_IMPL ${CMAKE_CURRENT_SOURCE_DIR}/3rdParties/ur_rtde/src/*)
# add_library(RtdeCommon SHARED ${COMMON_IMPL})
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdParties/ur_rtde/include/ur_rtde
# ${CMAKE_CURRENT_SOURCE_DIR}/3rdParties/ur_rtde/include/urcl)
# link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/3rdParties/ur_rtde/lib)


