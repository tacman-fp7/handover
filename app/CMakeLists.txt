# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME handover)


file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/script/*.template)
set(lua_fsm_LTR       ${CMAKE_CURRENT_SOURCE_DIR}/lua/handover_root_fsm_LTR.lua
                  ${CMAKE_CURRENT_SOURCE_DIR}/lua/handover_funcs.lua)
set(lua_bin_LTR       ${CMAKE_CURRENT_SOURCE_DIR}/lua/handover_main_LTR.lua)

set(lua_fsm_RTL      ${CMAKE_CURRENT_SOURCE_DIR}/lua/handover_root_fsm_RTL.lua
                  ${CMAKE_CURRENT_SOURCE_DIR}/lua/handover_funcs.lua)
set(lua_bin_RTL       ${CMAKE_CURRENT_SOURCE_DIR}/lua/handover_main_RTL.lua)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${ICUBCONTRIB_MODULE_PATH})
include(YarpInstallationHelpers)
include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

yarp_install(FILES    ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(PROGRAMS ${lua_fsm_RTL} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME}/lua)
yarp_install(PROGRAMS ${lua_fsm_LTR} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME}/lua)

#add_custom_target(copy_lua_in_build ALL)
#add_custom_command(TARGET copy_lua_in_build POST_BUILD
#                   COMMAND ${CMAKE_COMMAND}  copy ${lua_bin} ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
#                   COMMENT "Copying ${lua_bin} to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/")
install(PROGRAMS ${lua_bin_RTL} DESTINATION bin)
install(PROGRAMS ${lua_bin_LTR} DESTINATION bin)


