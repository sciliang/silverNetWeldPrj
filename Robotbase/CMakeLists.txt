cmake_minimum_required(VERSION 3.5)
project(Robotbase LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)        

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)        
endif()

find_package(Qt5 COMPONENTS  Widgets Core Gui SerialPort Network REQUIRED)
add_library(tinyxml2 SHARED ../global/tinyxml2.cpp ../global/tinyxml2.h)

file(GLOB_RECURSE HEADER_FILES 
    "${CMAKE_CURRENT_LIST_DIR}/include/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/*.hpp"
)

set(INCLUDE_DIRS "")
foreach(file ${HEADER_FILES})
    get_filename_component(dir ${file} DIRECTORY)
    list(APPEND INCLUDE_DIRS ${dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)  
message(STATUS "Final include dirs: ${INCLUDE_DIRS}")
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${Qt5SerialPort_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${INCLUDE_DIRS}  
)

file(GLOB HEADER
    "include/**/*.h"
    "include/**/*.hpp"
)

install(FILES
    "include/RobotMain/modbus.h"
    DESTINATION
    include/modbuspp
)

file(GLOB SOURCES
    "src/*.c"
    "src/*.cpp"
)

# add_library(msgPackage SHARED src/msgPackage.c)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/../log)
ADD_DEFINITIONS(-DLOG_PATH="${CMAKE_BINARY_DIR}/../log")
set(RobotLib_DIR ${CMAKE_SOURCE_DIR}/lib)
file(GLOB ALL_ROJECT_LIBRARIES "${RobotLib_DIR}/*.so")
set(PROJECT_LIBARARIES
    ${RobotLib_DIR}/libRobotLib.so 
    /usr/lib/x86_64-linux-gnu/libglog.so
    /usr/lib/x86_64-linux-gnu/libglog.so.0
    /usr/lib/x86_64-linux-gnu/libglog.so.0.0.0
    pthread
    rt
    m
    )
add_executable(${PROJECT_NAME} ${HEADER} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBARARIES} ${ALL_ROJECT_LIBRARIES} Qt5::Widgets Qt5::SerialPort Qt5::Core Qt5::Gui Qt5::Network glog)
target_compile_options(${PROJECT_NAME}  PRIVATE -fPIC)
