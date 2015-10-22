cmake_minimum_required(VERSION 2.8)
PROJECT (name)
# Add OpenCV libraray
find_package(OpenCV REQUIRED core imgproc highgui calib3d videostab xfeatures2d)
message(STATUS ${OpenCV_LIBS})

#set( NAME_SRC
#    src/main.cpp    
#)

#set( NAME_HEADERS       
#     include/header.h
#)


INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
link_directories( ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#add_executable( name ${NAME_SRC} ${NAME_HEADERS} )
add_library(SIFT_LOC src/relocalize.cpp)
target_link_libraries(SIFT_LOC ${OpenCV_LIBS})
#target_link_libraries( sample_pcTest ${OpenCV_LIBS} )
