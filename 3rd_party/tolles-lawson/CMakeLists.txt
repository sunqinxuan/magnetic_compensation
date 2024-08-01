# C++14 required
set (CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

file(GLOB_RECURSE SOURCE
    tl/tolles_lawson.cpp
    filter/filter.cpp
    filter/butterworth.cpp)

include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/tolles-lawson/)
add_library(tolles-lawson ${SOURCE})
