# Copyright 2013, Francois Keith, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-yoyoman01.
# sot-yoyoman01 is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-yoyoman01 is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-yoyoman01. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/ros.cmake)

SET(PROJECT_NAME sot-yoyoman01)
SET(PROJECT_DESCRIPTION "dynamic-graph package for yoyoman01 robot")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

# Those packages are not directly linked, but are used in the 
#  python scripts
ADD_REQUIRED_DEPENDENCY("dynamic-graph")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ADD_REQUIRED_DEPENDENCY("sot-core >= 2.7")
ADD_REQUIRED_DEPENDENCY("sot-tools")
ADD_REQUIRED_DEPENDENCY("sot-dynamic-pinocchio >= 2.6")
ADD_REQUIRED_DEPENDENCY("pinocchio")
ADD_ROSPACK_DEPENDENCY("dynamic_graph_bridge")

ADD_OPTIONAL_DEPENDENCY("yoyoman01_sot_description")
# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS filesystem system thread)
SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

# Handle rpath necessary to handle ROS multiplace packages
# libraries inclusion
SET(CMAKE_INSTALL_LIBDIR lib)
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Add subdirectories.
ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(python)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
