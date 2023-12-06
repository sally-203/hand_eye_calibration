PROJECT(eye_to_hand)
option(mechmind "mechmind" on)

if(mechmind)
    add_definitions(-Dmechmind)
endif()

set(CONTROL_INTERFACE_PATH "/home/xlh/work/projects/control_interface/control_interface")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(PCL REQUIRED QUIET)
if(PCL_FOUND)
    add_definitions(${PCL_DEFINITIONS})
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARIES_DIRS})
    message("\n")
    message(STATUS "PCL FOUND!")
endif()

find_package(OpenCV REQUIRED QUIET)
if(OpenCV_FOUND)
    add_definitions(${OpenCV_DEFINITIONS})
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARIES_DIRS})
    message("\n")
    message(STATUS "OpenCV FOUND!")
endif()

list(APPEND CMAKE_MODULE_PATH "/home/xlh/work/projects/control_interface/control_interface/cmake/deps")

if(mechmind)
    message("OPEN MECHMIND")
    find_package(MechMindSDK REQUIRED)
    include_directories(${MechMindSDK_INCLUDE_DIRS})
    link_libraries(${MechMindSDK_LIBRARIES})
endif()

include_directories(
    include
    ${CONTROL_INTERFACE_PATH}/util/include
    ${CONTROL_INTERFACE_PATH}/util/third_parties
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParties/ur_rtde/include/ur_rtde
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParties/ur_rtde/include/urcl
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParties/aruco/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParties/aruco/include/fractallabelers
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParties/cnpy/include
)

link_libraries(${CONTROL_INTERFACE_PATH}/build/util/libloggerWrapper.so)

add_library(EyeToHandCalibrationCommon SHARED src/calibration_to_hand.cpp)
target_link_libraries(EyeToHandCalibrationCommon ${PCL_LIBRARIES} 
                                        ${OpenCV_LIBRARIES}
                                        ${CONTROL_INTERFACE_PATH}/build/util/libloggerWrapper.so
                                        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParties/ur_rtde/lib/librtde.so
                                        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParties/aruco/lib/libaruco.so
                                        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParties/cnpy/lib/libcnpy.so)

add_executable(test_calibration_to_hand test/test_calibration_to_hand.cpp)
target_link_libraries(test_calibration_to_hand EyeToHandCalibrationCommon)

