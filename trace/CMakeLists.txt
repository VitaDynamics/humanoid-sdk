cmake_minimum_required(VERSION 3.20)
# May also be built independently for host trace validation without ORT/Aorta.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(locomotion_trace LANGUAGES CXX)
endif()
# Imported targets created in demo_onnx are not visible to this sibling.
find_package(Python 3.10 EXACT COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_library(TRACE_ZSTD_LIBRARY NAMES zstd REQUIRED)
set(TRACE_FLATBUFFERS_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../demo_onnx/cpp/body_cmd_vendor"
  CACHE PATH "FlatBuffers headers matching the checked-in trace generator")
pybind11_add_module(_trace_native NO_EXTRAS WITH_SOABI native/trace.cpp)
target_compile_features(_trace_native PRIVATE cxx_std_17)
target_compile_options(_trace_native PRIVATE -Wall -Wextra -Werror)
target_include_directories(_trace_native PRIVATE "${TRACE_FLATBUFFERS_INCLUDE_DIR}" generated)
target_link_libraries(_trace_native PRIVATE Threads::Threads ZLIB::ZLIB "${TRACE_ZSTD_LIBRARY}")
install(TARGETS _trace_native LIBRARY DESTINATION locomotion_aorta/trace)
install(FILES schema/trace.fbs DESTINATION locomotion_aorta/trace/schema)
