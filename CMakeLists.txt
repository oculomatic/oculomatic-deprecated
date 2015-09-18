cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
#set(CMAKE_CXX_COMPILER "icpc")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native")


project(oculomatic)

find_package (OpenCV REQUIRED)
find_package (TBB REQUIRED)
find_package (Boost REQUIRED)

include_directories(
    SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(lib)

SET(COMEDI_INSTALL "/usr/local/")
FIND_FILE(COMEDILIB_FOUND "comedilib.h" ${COMEDI_INSTALL}/include)
SET(COMEDI_INCLUDE_DIRS "${COMEDI_INSTALL}/include")
SET(COMEDI_LINK_DIRS "${COMEDI_INSTALL}/lib")
SET(COMEDI_LIBS "comedi")
FIND_LIBRARY(COMEDI_LIBRARY NAMES comedilib comedi HINTS "${COMEDI_LINK_DIRS}")

add_executable( oculomatic oculomatic.cpp )

find_library( FLYCAPTURE2 flycapture)
target_link_libraries( oculomatic ${FLYCAPTURE2} )
target_link_libraries( oculomatic ${COMEDI_LIBRARY} )
target_link_libraries( oculomatic ${OpenCV_LIBS} )
target_link_libraries( oculomatic ${TBB_LIBRARIES} )
target_link_libraries( oculomatic pupiltracker )
