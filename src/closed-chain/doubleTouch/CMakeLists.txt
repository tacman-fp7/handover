# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(closedChain)

include(YarpIDL)

file(GLOB source *.cpp)
file(GLOB header *.h)
yarp_add_idl(IDL_GEN_FILES idl.thrift)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})
source_group("IDL Files" FILES idl.thrift)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${periPersonalSpace_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${source} ${header} idl.thrift ${IDL_GEN_FILES} )

target_link_libraries(${PROJECT_NAME} periPersonalSpace ${YARP_LIBRARIES} ${IPOPT_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

