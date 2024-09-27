cmake_minimum_required(VERSION 3.5)

project(magnetic_compensation)

set (CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare \
    -Wno-deprecated -Wno-reorder -Wno-write-strings -Wno-unknown-pragmas \
    -Wno-unused-local-typedefs -Wno-parentheses -Wno-error -fPIC")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(ARM 0)

set(LIB_DIR lib)
if (ARM)
    message("DEFINED ARM, USE LIB IN LIB-ARM!")
    set(LIB_DIR lib-arm)
endif()

#####################################################################################
set(INSTALL_DIR "${CMAKE_HOME_DIRECTORY}/install")
set(INSTALL_BIN "${INSTALL_DIR}/bin")
set(INSTALL_ETC "${INSTALL_DIR}/etc")
set(INSTALL_LIB "${INSTALL_DIR}/lib")
set(INSTALL_MDL "${INSTALL_DIR}/models")

set(CMAKE_INSTALL_PREFIX ${INSTALL_DIR})
#####################################################################################
include_directories(${CMAKE_HOME_DIRECTORY}/include/eigen)
include_directories(${CMAKE_HOME_DIRECTORY}/include)
link_directories(${CMAKE_HOME_DIRECTORY}/lib)
message(${CMAKE_HOME_DIRECTORY}/lib)

SET(DEPEND_LIBS tolles-lawson kf-gins ceres gomp lapack
    f77blas atlas cholmod spqr cxsparse
    gfortran amd camd colamd ccolamd glog gflags)

add_executable(mic_test mic_test.cc)
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
    ${CMAKE_HOME_DIRECTORY}/etc/config_calibration.json
    ${CMAKE_HOME_DIRECTORY}/etc/config_compensation.json
    DESTINATION
    ${INSTALL_ETC}
)

install(
    FILES
    ${CMAKE_HOME_DIRECTORY}/scripts/set_env.sh
    DESTINATION
    ${INSTALL_DIR}
)

file(GLOB SDK_LIBS ${CMAKE_HOME_DIRECTORY}/lib/*)

install(
    FILES
    ${SDK_LIBS}
    DESTINATION
    ${INSTALL_LIB}
)

file(GLOB MIC_MODELS ${CMAKE_HOME_DIRECTORY}/models/*.mdl)

install(
    FILES
    ${MIC_MODELS}
    DESTINATION
    ${INSTALL_MDL}
)