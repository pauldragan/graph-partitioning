cmake_minimum_required(VERSION 3.0.0)
project(graph_partitioning_proj)

set(toml11_BUILD_TEST OFF CACHE BOOL "Disable build TOML11 tests.")

add_subdirectory(third_party/toml11)
add_subdirectory(lib/protos)

include_directories(include/)

add_executable(graph_partitioning src/main.cc src/onnx_reader.cc src/graph.cc src/algorithms.cc src/onnx_writer.cc src/utils.cc)

set(CMAKE_CXX_FLAGS "-O0 -Wall")
set(CMAKE_BUILD_TYPE "Debug")
set_property(TARGET graph_partitioning PROPERTY CXX_STANDARD 17)

target_link_libraries(graph_partitioning PUBLIC onnxprotos
					 PUBLIC toml11)
