################Initialize the Cmake Defaults#################

cmake_minimum_required(VERSION 2.6)

#Name the project
project(jaco6_driver_app)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

#Set scl base directory
SET(JACO_BASE_DIR ../../)

###(a) Jaco API
SET(KINOVA_API_DIR ${JACO_BASE_DIR}kinova-api)

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${JACO_BASE_DIR} ${KINOVA_API_DIR}) 

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp -std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(JACO_DRV_DIR ${JACO_BASE_DIR}/applications-linux/jaco6-driver-redis/)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(jaco6_driver ${JACO_DRV_DIR}/jaco6-driver-redis-main.cpp)

###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(jaco6_driver gomp hiredis dl)


