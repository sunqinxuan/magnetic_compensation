cmake_minimum_required(VERSION 3.5)

project(mag_compensation)

set (CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare \
    -Wno-deprecated -Wno-reorder -Wno-write-strings -Wno-unknown-pragmas \
    -Wno-unused-local-typedefs -Wno-parentheses -Wno-error -fPIC")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

include_directories(${CMAKE_HOME_DIRECTORY}/include)

# Eigen
include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/eigen-3.3.9)

# nlohmann json
include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/nlohmann/include)

## kf-gins
include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/kf-gins/)
add_subdirectory(3rd_party/kf-gins)

#####################################################################################
set(INSTALL_DIR "${CMAKE_HOME_DIRECTORY}/install")
set(INSTALL_BIN "${INSTALL_DIR}/bin")
set(INSTALL_ETC "${INSTALL_DIR}/etc")
set(INSTALL_LIB "${INSTALL_DIR}/lib")

set(CMAKE_INSTALL_PREFIX ${INSTALL_DIR})

set(SRC_FILES
    ./src/common/mag_logger.cc
    ./src/common/mag_config.cc
    ./src/mag_ins/mag_ins.cc
    ./src/mag_ins/impl/mag_kf_gins.cc
    ./src/mag_compensator/mag_compensator.cc
    ./src/mag_compensator/observer/mag_state_logger.cc
)

SET(DEPEND_LIBS kf-gins)

add_library(mag_compensation STATIC
    ${SRC_FILES}
)
target_compile_options(mag_compensation PRIVATE -std=c++11)
target_link_libraries(mag_compensation ${DEPEND_LIBS})

add_executable(mag_test test/test.cc)
target_compile_options(mag_test PRIVATE -std=c++11)
target_link_libraries(mag_test
    mag_compensation -pthread
)

install(
    TARGETS
    mag_test
    DESTINATION
    ${INSTALL_BIN}
)

install(
    FILES
    ${CMAKE_HOME_DIRECTORY}/etc/default_config.json
    DESTINATION
    ${INSTALL_ETC}
)

install(
    TARGETS
    mag_compensation
    DESTINATION
    ${INSTALL_LIB}
)

