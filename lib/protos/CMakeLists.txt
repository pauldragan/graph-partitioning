cmake_minimum_required(VERSION 3.0.0)
project(lib_protos_proj)

find_package(Protobuf REQUIRED)

include_directories(./)

file(GLOB_RECURSE ONNX_SRCS ./onnx *.cc)
add_library(onnxprotos ${ONNX_SRCS})

target_include_directories(onnxprotos PUBLIC ./)
target_link_libraries(onnxprotos PUBLIC ${Protobuf_LIBRARIES})
