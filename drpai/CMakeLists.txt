# YOLO DRP-AI Application for Renesas RZ/V2N
# Cross-compile for ARM Cortex-A55 with DRP-AI MERA2 runtime
#
# Build (inside Docker container):
#   mkdir -p build && cd build
#   cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain/runtime.cmake -DAPP_NAME=app_yolo_cam ..
#   make -j$(nproc)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED APP_NAME)
    set(APP_NAME "app_yolo_cam")
endif()

project(${APP_NAME} CXX)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "${default_build_type}")
endif()

# TVM_ROOT = DRP-AI TVM repository root ($TVM_ROOT env var)
set(TVM_ROOT "$ENV{TVM_ROOT}")
if(NOT TVM_ROOT)
    message(FATAL_ERROR "TVM_ROOT env var not set. Export TVM_ROOT=/drp-ai_tvm")
endif()

# TVM/MERA2 includes
include_directories(
    ${TVM_ROOT}/tvm/include
    ${TVM_ROOT}/setup/include
    ${TVM_ROOT}/tvm/3rdparty/dlpack/include
    ${TVM_ROOT}/tvm/3rdparty/dmlc-core/include
    ${TVM_ROOT}/tvm/3rdparty/compiler-rt
    ${TVM_ROOT}/apps
    ${TVM_ROOT}/apps/include
    ${TVM_ROOT}/3rdparty/spdlog/include
    ${TVM_ROOT}/3rdparty/asio/asio/include
    ${TVM_ROOT}/how-to/sample_app_v2h/common_files
    ${TVM_ROOT}/how-to/sample_app_v2h/common_files/package
)

# MERA2 runtime libraries (V2N uses v2h runtime)
set(LIBMERA_RT_PATH ${TVM_ROOT}/obj/build_runtime/v2h/lib)

add_definitions(-DMERA_DRP_RUNTIME)

# OpenCV
find_package(OpenCV REQUIRED)

# Sources: app code + MeraDrpRuntimeWrapper from TVM_ROOT
set(EXE_NAME ${APP_NAME})

add_executable(${EXE_NAME}
    src/main.cpp
    src/drpai_inference.cpp
    src/yolo_postprocess.cpp
    src/camera_capture.cpp
    src/display.cpp
    ${TVM_ROOT}/apps/MeraDrpRuntimeWrapper.cpp
    ${TVM_ROOT}/apps/PreRuntimeV2H.cpp
)

target_include_directories(${EXE_NAME} PRIVATE
    src
    ${TVM_ROOT}/apps
    ${TVM_ROOT}/apps/include
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_options(${EXE_NAME} PRIVATE -O3 -mtune=cortex-a55 -Wall)
target_compile_definitions(${EXE_NAME} PRIVATE V2H V2N KDLDRPAI)

# Link MERA2 runtime + OpenCV
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,${LIBMERA_RT_PATH} -Wl,-rpath-link,${LIBMERA_RT_PATH}")

target_link_directories(${EXE_NAME} PRIVATE ${LIBMERA_RT_PATH})
target_link_libraries(${EXE_NAME} PRIVATE
    ${OpenCV_LIBS}
    mera2_runtime
    mera2_plan_io
    drp_tvm_rt
    pthread
    rt
)

# GStreamer (optional — enables DMA camera capture)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(GSTREAMER gstreamer-1.0 gstreamer-app-1.0)
    if(GSTREAMER_FOUND)
        target_compile_definitions(${EXE_NAME} PRIVATE USE_GSTREAMER)
        target_include_directories(${EXE_NAME} PRIVATE ${GSTREAMER_INCLUDE_DIRS})
        target_link_libraries(${EXE_NAME} PRIVATE ${GSTREAMER_LIBRARIES})
        message(STATUS "GStreamer found")
    endif()
endif()

message(STATUS "TVM_ROOT: ${TVM_ROOT}")
message(STATUS "MERA2 runtime: ${LIBMERA_RT_PATH}")

install(TARGETS ${EXE_NAME} RUNTIME DESTINATION bin)
install(FILES ${CMAKE_SOURCE_DIR}/labels.txt DESTINATION bin)
