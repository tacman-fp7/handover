# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Giulia Vezzani <giulia.vezzani@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(localizerHandover)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(CGAL REQUIRED COMPONENTS)
find_package(Boost REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(YarpIDL)

add_definitions(${YARP_DEFINES}) 

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

set(headers headers/geometryCGAL.h headers/unscentedParticleFilter.h headers/scalingSeries.h headers/localizer.h )
set(sources src/unscentedParticleFilter.cpp src/main.cpp src/localizer.cpp src/scalingSeries.cpp) 
set(${PROJECT_NAME}_THRIFT_HDR src/idl.thrift)
yarp_add_idl(${PROJECT_NAME}_THRIFT_SRC ${${PROJECT_NAME}_THRIFT_HDR})
source_group("IDL Files" FILES src/idl.thrift)
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

include(${CGAL_USE_FILE})
include(CGAL_CreateSingleSourceCGALProgram)
include_directories(${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${headers} ${sources} ${${PROJECT_NAME}_THRIFT_SRC})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
