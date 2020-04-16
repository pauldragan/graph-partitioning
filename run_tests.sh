#!/bin/bash

# resnet50_onnx test
./build/graph_partitioning tests/resnet50_onnx.toml
python tests/test_onnx.py tests/resnet50_onnx.toml
