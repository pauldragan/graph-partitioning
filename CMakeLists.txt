cmake_minimum_required(VERSION 3.0.0)
project(graph_partitioning_proj)

add_subdirectory(lib/protos)

include_directories(include/)

add_executable(graph_partitioning src/main.cc src/onnx_reader.cc src/graph.cc src/algorithms.cc src/onnx_writer.cc)

set(CMAKE_CXX_FLAGS "-std=gnu++17 -O0 -g -Wall")

target_link_libraries(graph_partitioning PUBLIC onnxprotos)
