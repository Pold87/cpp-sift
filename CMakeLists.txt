cmake_minimum_required(VERSION 2.8) 
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

find_package(OpenCV REQUIRED)
find_package(Boost 1.48 REQUIRED COMPONENTS python)


INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" "/usr/include/python2.7")
link_directories( ${CMAKE_BINARY_DIR}/bin)

ADD_LIBRARY(Relocalize SHARED src/main.cpp)
TARGET_LINK_LIBRARIES(Relocalize ${Boost_LIBRARIES})