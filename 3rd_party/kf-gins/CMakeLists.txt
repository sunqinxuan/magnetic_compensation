# C++14 required
set (CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

file(GLOB_RECURSE SOURCE
    kf-gins/gi_engine.cpp
    kf-gins/insmech.cpp)

include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/kf-gins/)
add_library(kf-gins ${SOURCE})
