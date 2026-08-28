cmake_minimum_required(VERSION 3.16)

project(CamCalib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc highgui calib3d)
find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE CAMCALIB_SOURCES CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(camcalib ${CAMCALIB_SOURCES})

target_include_directories(camcalib
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camcalib
    PUBLIC
        ${OpenCV_LIBS}
        Eigen3::Eigen
)

add_executable(camcalib_app
    app/main.cpp
)

target_link_libraries(camcalib_app
    PRIVATE
        camcalib
        ${OpenCV_LIBS}
)

enable_testing()

add_executable(
    phase_shift_decoder_test
    tests/PhaseShiftDecoderTest.cpp
)

target_link_libraries(
    phase_shift_decoder_test
    PRIVATE
        camcalib
)

add_test(
    NAME phase_shift_decoder_test
    COMMAND phase_shift_decoder_test
)

if(MSVC)
  target_compile_options(camcalib PRIVATE /utf-8)
  target_compile_definitions(camcalib PRIVATE _USE_MATH_DEFINES)

  target_compile_options(camcalib_app PRIVATE /utf-8)
  target_compile_options(phase_shift_decoder_test PRIVATE /utf-8)
endif()
