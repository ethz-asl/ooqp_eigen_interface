#-------------------------------------------------------------------------------
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Péter Fankhauser, Christian Gehring, Stelian Coros
# All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Autonomous Systems Lab nor ETH Zurich
#     nor the names of its contributors may be used to endorse or
#     promote products derived from this software without specific
#     prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#-------------------------------------------------------------------------------

# Author(s): Peter Fankhauser
# Date     : Sep 9, 2013

cmake_minimum_required (VERSION 2.6)  

add_definitions(-std=c++0x)
set(CMAKE_CXX_FLAGS "-fPIC")

# Set Build Type
#   * None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
#   * Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
#   * Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
#   * RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO or CMAKE_CXX_FLAGS_RELWITHDEBINFO
#   * MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL or CMAKE_CXX_FLAGS_MINSIZEREL) 

# Noisily default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	message(STATUS "Setting build type to 'Release' as none was specified.")
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set output paths
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# CMake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen REQUIRED)

# Setup testing
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gtest/")
	message(STATUS "Building GTests!")
	option(BUILD_GTEST "build gtest" ON)
	add_subdirectory(gtest gtest)
	enable_testing()
endif()

###############
### SOURCES ###
###############
set(OOQPEI_SRCS 
	${CMAKE_CURRENT_SOURCE_DIR}/src/OoqpEigenInterface.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/QuadraticProblemFormulation.cpp 
)

################
### INCLUDES ###
################
set(OOQPEI_INCL
  	${CMAKE_CURRENT_SOURCE_DIR}/include
  	/usr/include/suitesparse
	/usr/local/include/ooqp
	${EIGEN_INCLUDE_DIRS}
	#/usr/include/tcl8.5
)

#################
### LIBRARIES ###
#################
set(OOQPEI_LIBS 
	ooqpgensparse ooqpsparse ooqpgondzio ooqpbase blas ma27 gfortran
)	

#################
### UTILITIES ###
#################
include_directories(${OOQPEI_INCL})
include_directories(${EIGEN_INCLUDE_DIRS})
add_library(ooqpei ${OOQPEI_SRCS} )
target_link_libraries(ooqpei ${OOQPEI_LIBS} )

# Generate FindOOQPEI.cmake file
file(WRITE cmake/FindOOQPEI.cmake
"
set(OOQPEI_INCLUDE_DIRS
  ${CMAKE_INSTALL_PREFIX}/include/OOQPEI/include
)
set(OOQEI_LIBRARIES
  ooqpei 
  ${OOQPEI_LIBS}
)
set(OOQEI_FOUND TRUE)
"
)

# Setting for make install
install(TARGETS ooqpei DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/include/OOQPEI)")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX}/include/OOQPEI)
install(FILES cmake/FindOOQPEI.cmake DESTINATION ${CMAKE_ROOT}/Modules)

if(BUILD_GTEST)
	add_subdirectory(test)
endif()
