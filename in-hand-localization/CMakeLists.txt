# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

### TEMPORARY CMAKELISTS FILE
cmake_minimum_required(VERSION 2.8)
project(in-hand-localization)

find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

add_subdirectory(src)
add_subdirectory(app)

icubcontrib_add_uninstall_target()
