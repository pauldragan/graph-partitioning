#!/bin/bash

protoc --proto_path third_party/onnx third_party/onnx/onnx/onnx.proto3 --cpp_out=./proto-out
protoc --proto_path third_party/onnx third_party/onnx/onnx/onnx-ml.proto3 --cpp_out=./proto-out
protoc --proto_path third_party/onnx third_party/onnx/onnx/onnx-operators.proto3 --cpp_out=./proto-out
protoc --proto_path third_party/onnx third_party/onnx/onnx/onnx-operators-ml.proto3 --cpp_out=./proto-out
