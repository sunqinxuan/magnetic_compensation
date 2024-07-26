cmake_minimum_required(VERSION 3.5)

project(magnetic_compensation)

set (CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare \
    -Wno-deprecated -Wno-reorder -Wno-write-strings -Wno-unknown-pragmas \
    -Wno-unused-local-typedefs -Wno-parentheses -Wno-error -fPIC")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

#####################################################################################
set(INSTALL_DIR "${CMAKE_HOME_DIRECTORY}/install")
set(INSTALL_BIN "${INSTALL_DIR}/bin")
set(INSTALL_ETC "${INSTALL_DIR}/etc")
set(INSTALL_LIB "${INSTALL_DIR}/lib")

set(CMAKE_INSTALL_PREFIX ${INSTALL_DIR})
#####################################################################################

include_directories(${CMAKE_HOME_DIRECTORY}/include)

# Eigen
include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/eigen-3.3.9)

# nlohmann json
include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/nlohmann/include)

## ceres
set(CERES_DIR "${CMAKE_HOME_DIRECTORY}/3rd_party/ceres-1.12.0")
include_directories(${CERES_DIR}/include/)
link_directories(${CERES_DIR}/lib/)
install(DIRECTORY ${CERES_DIR}/lib/ DESTINATION ${INSTALL_LIB})

## glog
set(GLOG_DIR "${CMAKE_HOME_DIRECTORY}/3rd_party/glog-0.4.0")
include_directories(${GLOG_DIR}/include/)
link_directories(${GLOG_DIR}/lib/)
install(DIRECTORY ${GLOG_DIR}/lib/ DESTINATION ${INSTALL_LIB})

## gflag
set(GFLAGS_DIR "${CMAKE_HOME_DIRECTORY}/3rd_party/gflag-2.1.2")
include_directories(${GFLAGS_DIR}/include/)
link_directories(${GFLAGS_DIR}/lib/)
install(DIRECTORY ${GFLAGS_DIR}/lib/ DESTINATION ${INSTALL_LIB})

## kf-gins
include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/kf-gins/)
add_subdirectory(3rd_party/kf-gins)

## tolles-lawson
include_directories(${CMAKE_HOME_DIRECTORY}/3rd_party/tolles-lawson/)
add_subdirectory(3rd_party/tolles-lawson)


set(SRC_FILES
    ./src/common/mic_logger.cc
    ./src/common/mic_config.cc
    ./src/mic_nav_state_estimator/mic_nav_state_estimator.cc
    ./src/mic_nav_state_estimator/impl/mic_kf_ins_estimator.cc
    ./src/mic_mag_compensator/mic_mag_compensator.cc
    ./src/mic_mag_compensator/observer/mic_state_logger.cc
    ./src/mic_mag_compensator/impl/mic_ellipsoid_mag_compensator.cc
    ./src/mic_mag_compensator/impl/mic_tl_mag_compensator.cc
)

SET(DEPEND_LIBS tolles-lawson kf-gins ceres cholmod lapack blas glog gflags gomp)

add_library(magnetic_compensation SHARED
    ${SRC_FILES}
)
target_compile_options(magnetic_compensation PRIVATE -std=c++11)

add_executable(mic_test test/test.cc)
target_compile_options(mic_test PRIVATE -std=c++11)
target_link_libraries(mic_test
    magnetic_compensation ${DEPEND_LIBS} -pthread
)

install(
    TARGETS
    mic_test
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
    FILES
    ${CMAKE_HOME_DIRECTORY}/scripts/set_env.sh
    DESTINATION
    ${INSTALL_DIR}
)

install(
    TARGETS
    magnetic_compensation
    DESTINATION
    ${INSTALL_LIB}
)